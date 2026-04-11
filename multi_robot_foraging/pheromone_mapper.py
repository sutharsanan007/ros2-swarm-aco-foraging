"""
pheromone_mapper.py
===================
Renders colored pheromone trail MarkerArrays with ITERATION-BASED THICKNESS.

Live trails (/pheromone_trails):
  - Iteration 1 → thin   (0.02 m)
  - Iteration 2 → medium (0.05 m)
  - Iteration 3 → thick  (0.09 m)
  Colors: Robot 1=Green, Robot 2=Red, Robot 3=Blue

Final frozen path (/final_path_trails):  published after ALL 3 robots halt.
  Goal: Robot 1's lane shows ALL THREE COLORS stacked so the ACO convergence
  is visually obvious.

  Layout on Robot 1's lane (x ≈ -1.5):
    GREEN  (Robot 1, all 3 iterations) — thickest, gold in final, z=0.01
    WHITE  (Robot 2, iteration 3 merged section) — medium thick, z=0.03
    CYAN   (Robot 3, iteration 3 merged section) — medium thick, z=0.05

  Other robot own-lane trails (iterations 1 & 2):
    Original color, very thin (0.015 m), alpha=0.25 → "evaporated" look.

Pheromone encoding (pt.z field):
  robot_id * 10 + iteration  →  11=R1I1, 12=R1I2, 13=R1I3, 21, 22, 23, 31, 32, 33
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from collections import deque

# ── Colors ───────────────────────────────────────────────────────────────
COLORS = {
    1: (0.1, 0.95, 0.2),   # Green  (Robot 1)
    2: (0.95, 0.2, 0.1),   # Red    (Robot 2)
    3: (0.2,  0.4, 1.0),   # Blue   (Robot 3)
}

# ── Line widths per iteration (live view) ───────────────────────────────
ITER_WIDTH = {1: 0.02, 2: 0.05, 3: 0.09}

# ── Pheromone persistence ─────────────────────────────────────────────────
PHEROMONE_LIFETIME = 300.0   # 5 minutes — persists entire 3-iteration run

# ── Lane-merge detection ─────────────────────────────────────────────────
LANE1_X     = -1.5     # Robot 1's world X lane
LANE_THRESH =  0.30    # ±30 cm tolerance


class PheromoneMapper(Node):

    def __init__(self):
        super().__init__('pheromone_mapper')

        # ── Subscriptions ────────────────────────────────────────────────
        for rn in (1, 2, 3):
            self.create_subscription(
                Point, f'/robot{rn}/pheromone_drop', self.drop_cb, 10
            )
        self.create_subscription(Int32, '/robot_halted', self.halt_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_live  = self.create_publisher(MarkerArray, '/pheromone_trails',   10)
        self.pub_final = self.create_publisher(MarkerArray, '/final_path_trails',  10)

        # ── Storage: deque of (x, y, timestamp, robot_id, iteration) ────
        # Keyed by (robot_id, iteration) for easy per-bucket access
        # robot_id in {1,2,3}, iteration in {1,2,3}
        self.buckets: dict[tuple, deque] = {}
        for r in (1, 2, 3):
            for i in (1, 2, 3):
                self.buckets[(r, i)] = deque()

        # ── Halt counting ─────────────────────────────────────────────────
        self.halted: set  = set()
        self.sim_done     = False
        self.final_array  = None

        # ── Timers ───────────────────────────────────────────────────────
        self.create_timer(0.25, self.publish_live)   # 4 Hz
        self.create_timer(0.5,  self.publish_final)  # 2 Hz

        self.get_logger().info(
            f'PheromoneMapper online | lifetime={PHEROMONE_LIFETIME}s | '
            f'waiting for 3/3 halts before freezing final path'
        )

    # ================================================================== #
    #  Drop callback
    # ================================================================== #

    def drop_cb(self, msg: Point):
        code      = int(round(msg.z))
        robot_id  = code // 10       # e.g. 21 // 10 = 2
        iteration = code  % 10       # e.g. 21 %  10 = 1

        if robot_id not in (1, 2, 3) or iteration not in (1, 2, 3):
            return

        now = self.get_sim_time()
        self.buckets[(robot_id, iteration)].append((msg.x, msg.y, now))

    # ================================================================== #
    #  Halt callback — freeze only after ALL 3 robots complete
    # ================================================================== #

    def halt_cb(self, msg: Int32):
        self.halted.add(msg.data)
        self.get_logger().info(
            f'[Mapper] Robot {msg.data} halted — {len(self.halted)}/3 done'
        )
        if len(self.halted) >= 3 and not self.sim_done:
            self.sim_done    = True
            self.final_array = self._build_final()
            self.get_logger().info(
                '\n\033[1;36m'
                '*** [RVIZ] All 3 robots complete! '
                'Final path frozen → check /final_path_trails ***'
                '\033[0m\n'
            )

    # ================================================================== #
    #  Helpers
    # ================================================================== #

    def get_sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _evaporate(self):
        if self.sim_done:
            return
        cutoff = self.get_sim_time() - PHEROMONE_LIFETIME
        for dq in self.buckets.values():
            while dq and dq[0][2] < cutoff:
                dq.popleft()

    @staticmethod
    def _make_marker(stamp, ns, mid, width, frame='world') -> Marker:
        m                    = Marker()
        m.header.stamp       = stamp
        m.header.frame_id    = frame
        m.ns                 = ns
        m.id                 = mid
        m.type               = Marker.LINE_STRIP
        m.action             = Marker.ADD
        m.scale.x            = width
        m.pose.orientation.w = 1.0
        return m

    @staticmethod
    def _color(r, g, b, a) -> ColorRGBA:
        c = ColorRGBA(); c.r=float(r); c.g=float(g); c.b=float(b); c.a=float(a)
        return c

    # ================================================================== #
    #  Live marker builder — thickness grows each iteration
    # ================================================================== #

    def _build_live(self) -> MarkerArray:
        now   = self.get_sim_time()
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid   = 0

        for robot_id in (1, 2, 3):
            r, g, b = COLORS[robot_id]
            for iteration in (1, 2, 3):
                dq = self.buckets[(robot_id, iteration)]
                if not dq:
                    continue

                width = ITER_WIDTH[iteration]
                m     = self._make_marker(stamp, f'r{robot_id}_i{iteration}',
                                          mid, width)
                mid  += 1

                for (px, py, t) in dq:
                    pt     = Point(); pt.x=px; pt.y=py; pt.z=0.01*iteration
                    age    = now - t
                    frac   = age / PHEROMONE_LIFETIME
                    # Cubic fade: stays bright for first 80%, fades near end
                    alpha  = max(0.15, 1.0 - frac ** 3)
                    m.points.append(pt)
                    m.colors.append(self._color(r, g, b, alpha))

                array.markers.append(m)

        return array

    # ================================================================== #
    #  Final marker builder — called ONCE after all 3 robots halt
    #
    #  Visual goal: Robot 1's lane clearly shows GREEN + WHITE + CYAN
    #  showing that all 3 robots converged on the optimal path.
    #  Own-lane iter 1&2 trails are thin + very faded = "evaporated".
    # ================================================================== #

    def _build_final(self) -> MarkerArray:
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid   = 100   # offset to avoid namespace collision with live markers

        for robot_id in (1, 2, 3):
            r, g, b = COLORS[robot_id]

            for iteration in (1, 2, 3):
                dq = self.buckets[(robot_id, iteration)]
                if not dq:
                    continue

                # ── Split points: own lane vs merged lane ────────────────
                own_pts    = []
                merged_pts = []
                for (px, py, _) in dq:
                    on_lane1 = abs(px - LANE1_X) < LANE_THRESH
                    if robot_id != 1 and on_lane1:
                        merged_pts.append((px, py))
                    else:
                        own_pts.append((px, py))

                # ── Own lane segment ─────────────────────────────────────
                if own_pts:
                    if robot_id == 1:
                        # Robot 1: GOLD, thickness grows with iteration
                        fr, fg, fb = 1.0, 0.80, 0.0
                        width = {1: 0.04, 2: 0.07, 3: 0.12}[iteration]
                        alpha = {1: 0.55, 2: 0.75, 3: 1.00}[iteration]
                        z_off = 0.01
                    else:
                        # Other robots own lanes: faded original color
                        # Thinner per iteration (evaporation narrative)
                        fr, fg, fb = r, g, b
                        width = {1: 0.015, 2: 0.015, 3: 0.020}[iteration]
                        alpha = {1: 0.20,  2: 0.20,  3: 0.30 }[iteration]
                        z_off = 0.005

                    m = self._make_marker(
                        stamp, f'r{robot_id}_i{iteration}_own', mid, width
                    )
                    mid += 1
                    for (px, py) in own_pts:
                        pt = Point(); pt.x=px; pt.y=py; pt.z=z_off
                        m.points.append(pt)
                        m.colors.append(self._color(fr, fg, fb, alpha))
                    array.markers.append(m)

                # ── Merged lane segment (on Robot 1's lane) ──────────────
                # These are Robot 2 & 3 iter-3 trails that physically ran on
                # Robot 1's lane. Show as bright WHITE / CYAN so audience
                # can see all 3 colors converging on the winning path.
                if merged_pts and iteration == 3:
                    if robot_id == 2:
                        # WHITE — Robot 2 merged
                        mr, mg, mb = 1.0, 1.0, 1.0
                    else:
                        # CYAN — Robot 3 merged
                        mr, mg, mb = 0.3, 1.0, 1.0

                    # Thinner than Robot 1's gold (shows "joining" not "winning")
                    m = self._make_marker(
                        stamp, f'r{robot_id}_merged', mid, 0.055
                    )
                    mid += 1
                    # Offset z so all 3 colors are visible, not stacked
                    z_off = 0.03 if robot_id == 2 else 0.05
                    for (px, py) in merged_pts:
                        pt = Point(); pt.x=px; pt.y=py; pt.z=z_off
                        m.points.append(pt)
                        m.colors.append(self._color(mr, mg, mb, 0.90))
                    array.markers.append(m)

        return array

    # ================================================================== #
    #  Timer callbacks
    # ================================================================== #

    def publish_live(self):
        if self.sim_done:
            return
        self._evaporate()
        self.pub_live.publish(self._build_live())

    def publish_final(self):
        if self.final_array is not None:
            self.pub_final.publish(self.final_array)


def main(args=None):
    rclpy.init(args=args)
    node = PheromoneMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()