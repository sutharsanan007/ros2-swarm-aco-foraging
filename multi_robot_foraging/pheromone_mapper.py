import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

# ── Per-robot trail colors (RGB, 0-1 scale) ─────────────────────────────
#    Robot 1 = GREEN  (shortest, optimal path — leading the swarm)
#    Robot 2 = BLUE   (long path → merges into green lane on iter 3)
#    Robot 3 = RED    (medium path → merges into green lane on iter 3)
ROBOT_COLORS = {
    1: (0.0, 1.0, 0.2),   # Bright green
    2: (0.2, 0.5, 1.0),   # Bright blue
    3: (1.0, 0.25, 0.0),  # Bright red/orange
}


class PheromoneMapper(Node):
    """
    Subscribes to /robotN/pheromone_drop and builds per-robot float grids.
    Publishes colored CUBE_LIST MarkerArrays to /pheromone_trails (live)
    and /final_path_trails (frozen, boosted) after sim_complete.

    Why CUBE_LIST instead of OccupancyGrid?
    - OccupancyGrid is single-channel → only one color in RViz.
    - CUBE_LIST lets each robot have its own RGB color with alpha
      proportional to pheromone concentration — you can visually see
      which lane is strongest at a glance.
    """

    def __init__(self):
        super().__init__('pheromone_mapper')

        # ── Subscriptions ────────────────────────────────────────────────
        for rid in [1, 2, 3]:
            self.create_subscription(
                Point, f'/robot{rid}/pheromone_drop',
                lambda msg, r=rid: self.drop_cb(msg, r), 10
            )
        self.create_subscription(Bool, '/sim_complete', self.complete_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_live  = self.create_publisher(MarkerArray, '/pheromone_trails',    10)
        self.pub_final = self.create_publisher(MarkerArray, '/final_path_trails',   10)

        # ── Grid parameters ──────────────────────────────────────────────
        # 12 m × 8 m covers the arena snugly at 0.08 m/cell
        # (fewer cells = faster MarkerArray generation at each publish)
        self.res      = 0.08
        self.width    = int(12.0 / self.res)   # 150 cells
        self.height   = int( 8.0 / self.res)   # 100 cells
        self.origin_x = -6.0
        self.origin_y = -4.0

        # Separate float grid per robot
        shape = (self.height, self.width)
        self.grids = {
            1: np.zeros(shape, dtype=np.float32),
            2: np.zeros(shape, dtype=np.float32),
            3: np.zeros(shape, dtype=np.float32),
        }

        self.sim_done   = False
        self.final_snap = None   # frozen MarkerArray after sim complete

        # ── Timers ───────────────────────────────────────────────────────
        # Slow evaporation (0.5 %/s) so trails persist across iterations
        self.create_timer(1.0, self.evaporate_and_publish)
        # Final path re-published at 1 Hz indefinitely
        self.create_timer(1.0, self.publish_final)

        self.get_logger().info(
            f'PheromoneMapper online | '
            f'grid {self.width}×{self.height} @ {self.res}m | '
            f'R1=green R2=blue R3=red'
        )

    # ================================================================== #
    #  Helpers
    # ================================================================== #

    def _world_to_grid(self, wx, wy):
        col = int((wx - self.origin_x) / self.res)
        row = int((wy - self.origin_y) / self.res)
        return col, row

    def _in_bounds(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    # ================================================================== #
    #  Callbacks
    # ================================================================== #

    def drop_cb(self, msg: Point, robot_id: int):
        col, row = self._world_to_grid(msg.x, msg.y)
        if not self._in_bounds(col, row):
            return
        if self.sim_done:
            return  # no new drops after simulation ends

        # 3×3 burst — drop amount chosen so:
        # DRIVE_TO_HOME drops 3× per 50 ms = 60 drops/sec × 0.3 = 18 units/sec
        # DRIVE_TO_RESOURCE drops 1× per 50 ms = 20 drops/sec × 0.3 = 6 units/sec
        # Evaporation is 0.5 %/sec → strong paths visually saturate in ~30 s
        amt = 0.30
        g   = self.grids[robot_id]
        for r in range(max(0, row - 1), min(self.height, row + 2)):
            for c in range(max(0, col - 1), min(self.width,  col + 2)):
                g[r, c] = min(100.0, g[r, c] + amt)

    def complete_cb(self, msg: Bool):
        if msg.data and not self.sim_done:
            self.sim_done   = True
            # Boost: scale each grid so its max = 100, then build final snapshot
            boosted = {}
            for rid, g in self.grids.items():
                mx = g.max()
                boosted[rid] = (g / mx * 100.0) if mx > 0 else g.copy()
            self.final_snap = self._build_marker_array(boosted, scale_factor=1.5)
            self.get_logger().info(
                '\n\033[1;36m'
                '*** [RVIZ] Sim complete — final paths frozen & boosted! '
                'Check /final_path_trails in RViz ***'
                '\033[0m\n'
            )

    # ================================================================== #
    #  Marker array builder
    # ================================================================== #

    def _build_marker_array(self, grids: dict, scale_factor=1.0) -> MarkerArray:
        """
        Build one CUBE_LIST Marker per robot.
        Each active cell (value > 1.0) becomes one cube.
        Alpha is proportional to the pheromone concentration.
        """
        ma    = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for rid, g in grids.items():
            r, c = np.where(g > 1.0)
            if len(r) == 0:
                # Publish a delete marker to clear stale cubes
                mk            = Marker()
                mk.header.stamp    = stamp
                mk.header.frame_id = 'world'
                mk.ns              = f'robot{rid}'
                mk.id              = rid
                mk.action          = Marker.DELETE
                ma.markers.append(mk)
                continue

            color_rgb = ROBOT_COLORS[rid]
            mk                   = Marker()
            mk.header.stamp      = stamp
            mk.header.frame_id   = 'world'
            mk.ns                = f'robot{rid}'
            mk.id                = rid
            mk.type              = Marker.CUBE_LIST
            mk.action            = Marker.ADD
            mk.scale.x           = self.res * 0.95   # slight gap between cubes
            mk.scale.y           = self.res * 0.95
            mk.scale.z           = 0.03              # flat — sits on ground
            mk.pose.orientation.w = 1.0

            for row_i, col_i in zip(r, c):
                val   = float(g[row_i, col_i])
                alpha = min(1.0, (val / 100.0) * scale_factor)

                pt   = Point()
                pt.x = self.origin_x + col_i * self.res + self.res / 2
                pt.y = self.origin_y + row_i * self.res + self.res / 2
                pt.z = 0.01   # just above ground plane
                mk.points.append(pt)

                col_rgba       = ColorRGBA()
                col_rgba.r     = color_rgb[0]
                col_rgba.g     = color_rgb[1]
                col_rgba.b     = color_rgb[2]
                col_rgba.a     = alpha
                mk.colors.append(col_rgba)

            ma.markers.append(mk)

        return ma

    # ================================================================== #
    #  Timers
    # ================================================================== #

    def evaporate_and_publish(self):
        """Evaporate 0.5 %/sec and publish live colored trails."""
        if not self.sim_done:
            for g in self.grids.values():
                g *= 0.995          # 0.5 % decay per second
                np.clip(g, 0.0, 100.0, out=g)

        ma = self._build_marker_array(self.grids, scale_factor=1.0)
        self.pub_live.publish(ma)

    def publish_final(self):
        """Republish the frozen final-path snapshot indefinitely."""
        if self.final_snap is not None:
            # Update timestamps so RViz doesn't discard stale markers
            stamp = self.get_clock().now().to_msg()
            for mk in self.final_snap.markers:
                mk.header.stamp = stamp
            self.pub_final.publish(self.final_snap)


def main(args=None):
    rclpy.init(args=args)
    node = PheromoneMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()