import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import numpy as np

class PheromoneMapper(Node):
    """
    Builds a fading OccupancyGrid heatmap from /robotN/pheromone_drop breadcrumbs.

    On /sim_complete the evaporation stops and a second 'final path' grid
    is published to /final_path_map — the winning lane shown at full brightness
    indefinitely for the RViz audience.
    """

    def __init__(self):
        super().__init__('pheromone_mapper')

        # ── Pheromone subscriptions ─────────────────────────────────────
        self.create_subscription(Point, '/robot1/pheromone_drop', self.drop_cb, 10)
        self.create_subscription(Point, '/robot2/pheromone_drop', self.drop_cb, 10)
        self.create_subscription(Point, '/robot3/pheromone_drop', self.drop_cb, 10)

        # ── Sim-complete flag (published by Robot 1 in HALT state) ──────
        self.create_subscription(Bool, '/sim_complete', self.complete_cb, 10)

        # ── Publishers ──────────────────────────────────────────────────
        self.pub_live  = self.create_publisher(OccupancyGrid, '/pheromone_map',  10)
        self.pub_final = self.create_publisher(OccupancyGrid, '/final_path_map', 10)

        # ── Grid parameters ─────────────────────────────────────────────
        # 10 m × 10 m arena at 0.05 m/cell → 200 × 200 cells
        self.res      = 0.05
        self.width    = 200
        self.height   = 200
        self.origin_x = -5.0
        self.origin_y = -5.0

        self.live_grid  = np.zeros((self.height, self.width), dtype=np.float32)
        self.final_grid = None   # frozen snapshot at sim-complete

        self.sim_done = False

        # ── Timers ──────────────────────────────────────────────────────
        # Live heatmap: evaporate + publish at 1 Hz
        self.create_timer(1.0, self.publish_live)
        # Final path: publish at 2 Hz once sim complete (no evaporation)
        self.create_timer(0.5, self.publish_final)

        self.get_logger().info(
            f'PheromoneMapper online — '
            f'{self.width}×{self.height} grid @ {self.res}m/cell'
        )

    # ================================================================== #
    #  Callbacks
    # ================================================================== #

    def drop_cb(self, msg: Point):
        """Add a 3×3 pheromone burst at the robot's current real position."""
        col = int((msg.x - self.origin_x) / self.res)
        row = int((msg.y - self.origin_y) / self.res)

        if not (0 <= col < self.width and 0 <= row < self.height):
            return

        # +0.05 per call; at 20 Hz this is 1.0 unit/sec deposition.
        # Evaporation is 2 %/sec → saturation takes ~20 s on an active lane.
        burst = 0.05
        for r in range(max(0, row - 1), min(self.height, row + 2)):
            for c in range(max(0, col - 1), min(self.width, col + 2)):
                self.live_grid[r, c] = min(100.0, self.live_grid[r, c] + burst)

    def complete_cb(self, msg: Bool):
        """
        Freeze evaporation and snapshot the winning-path grid.
        Boost the final grid so the brightest lane is clearly visible.
        """
        if msg.data and not self.sim_done:
            self.sim_done = True

            # Boost: normalise the final grid to 0–100 so the hottest
            # lane saturates and the old lanes show as lighter trails.
            snapshot = self.live_grid.copy()
            max_val  = snapshot.max()
            if max_val > 0:
                self.final_grid = (snapshot / max_val * 100.0).astype(np.float32)
            else:
                self.final_grid = snapshot.copy()

            self.get_logger().info(
                '\n\033[1;36m'
                '*** [RVIZ] Simulation complete — '
                'final pheromone path frozen and highlighted! ***'
                '\033[0m\n'
            )

    # ================================================================== #
    #  Publishers
    # ================================================================== #

    def _make_grid_msg(self, grid: np.ndarray, frame: str) -> OccupancyGrid:
        msg                          = OccupancyGrid()
        msg.header.stamp             = self.get_clock().now().to_msg()
        msg.header.frame_id          = frame
        msg.info.resolution          = self.res
        msg.info.width               = self.width
        msg.info.height              = self.height
        msg.info.origin.position.x   = self.origin_x
        msg.info.origin.position.y   = self.origin_y
        msg.info.origin.orientation.w = 1.0
        int_grid = np.clip(grid, 0, 100).astype(np.int8)
        msg.data = int_grid.flatten().tolist()
        return msg

    def publish_live(self):
        """Evaporate and publish the live heatmap (stops evaporating after sim_done)."""
        if not self.sim_done:
            # 2 % decay per second (timer is 1 Hz)
            self.live_grid *= 0.98
            np.clip(self.live_grid, 0.0, 100.0, out=self.live_grid)

        self.pub_live.publish(self._make_grid_msg(self.live_grid, 'world'))

    def publish_final(self):
        """Publish the frozen winning-path grid indefinitely after sim complete."""
        if self.final_grid is not None:
            self.pub_final.publish(self._make_grid_msg(self.final_grid, 'world'))


def main(args=None):
    rclpy.init(args=args)
    node = PheromoneMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()