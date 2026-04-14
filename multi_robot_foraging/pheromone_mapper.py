"""
pheromone_mapper.py
===================
FIX: halt_cb now ignores duplicate robot_id messages.
  In the previous version, Robot 1 published /robot_halted at 50 Hz in HALT
  state, flooding the log with hundreds of "R1 halted — 1/3" messages.
  Now unified_agent publishes ONCE per robot, but the mapper also guards
  with a set so even if a duplicate arrives it is silently ignored.

/pheromone_trails  — Live colored LINE_STRIP per robot per iteration.
  Green=R1, Red=R2, Blue=R3. Thickness grows each iteration (thin→thick).
  Cubic alpha fading (stays bright 80% of lifetime, fades near end).

/final_path_trails — Frozen after ALL 3 robots halt.
  Shows ONLY iteration-3 paths in same Green/Red/Blue colors.
  All 3 converge onto Robot 1's lane (x≈-1.5).
  Z-offset: R1=0.01m, R2=0.03m, R3=0.05m so all 3 visible simultaneously.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from collections import deque

COLOR = {
    1: (0.10, 0.95, 0.20),   # Green  (Robot 1 — winner)
    2: (0.95, 0.20, 0.10),   # Red    (Robot 2)
    3: (0.20, 0.45, 1.00),   # Blue   (Robot 3)
}

WIDTH    = {1: 0.025, 2: 0.055, 3: 0.095}   # grows each iteration
LIFETIME = 300.0   # 5 minutes — covers full run


class PheromoneMapper(Node):

    def __init__(self):
        super().__init__('pheromone_mapper')

        for rn in (1, 2, 3):
            self.create_subscription(
                Point, f'/robot{rn}/pheromone_drop', self.drop_cb, 10
            )
        self.create_subscription(Int32, '/robot_halted', self.halt_cb, 10)

        self.pub_live  = self.create_publisher(MarkerArray, '/pheromone_trails',   10)
        self.pub_final = self.create_publisher(MarkerArray, '/final_path_trails',  10)

        # (robot_id, iteration) → deque of (x, y, timestamp)
        self.buckets = {(r, i): deque() for r in (1,2,3) for i in (1,2,3)}

        # FIX: use a set — only count first halt message per robot
        self.halted      = set()
        self.sim_done    = False
        self.final_array = None

        self.create_timer(0.25, self.live_tick)
        self.create_timer(0.50, self.final_tick)

        self.get_logger().info(
            f'PheromoneMapper online | lifetime={LIFETIME}s'
        )

    # ================================================================== #
    #  Callbacks
    # ================================================================== #

    def drop_cb(self, msg: Point):
        code = int(round(msg.z))
        rid  = code // 10
        it   = code  % 10
        if rid not in (1,2,3) or it not in (1,2,3):
            return
        self.buckets[(rid, it)].append((msg.x, msg.y, self.now()))

    def halt_cb(self, msg: Int32):
        rid = msg.data
        # FIX: ignore duplicates silently — only process first halt per robot
        if rid in self.halted:
            return
        self.halted.add(rid)
        self.get_logger().info(
            f'[Mapper] R{rid} halted — {len(self.halted)}/3'
        )
        if len(self.halted) >= 3 and not self.sim_done:
            self.sim_done    = True
            self.final_array = self._build_final()
            self.get_logger().info(
                '\n\033[1;36m'
                '*** ALL ROBOTS COMPLETE — '
                'final convergence path frozen on /final_path_trails ***'
                '\033[0m\n'
            )

    # ================================================================== #
    #  Utilities
    # ================================================================== #

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _evaporate(self):
        if self.sim_done:
            return
        cutoff = self.now() - LIFETIME
        for dq in self.buckets.values():
            while dq and dq[0][2] < cutoff:
                dq.popleft()

    @staticmethod
    def _mk(stamp, ns, mid, width) -> Marker:
        m = Marker()
        m.header.stamp       = stamp
        m.header.frame_id    = 'world'
        m.ns                 = ns
        m.id                 = mid
        m.type               = Marker.LINE_STRIP
        m.action             = Marker.ADD
        m.scale.x            = float(width)
        m.pose.orientation.w = 1.0
        return m

    @staticmethod
    def _col(r, g, b, a) -> ColorRGBA:
        c = ColorRGBA()
        c.r=float(r); c.g=float(g); c.b=float(b); c.a=float(a)
        return c

    @staticmethod
    def _pt(x, y, z=0.01) -> Point:
        p = Point(); p.x=float(x); p.y=float(y); p.z=float(z)
        return p

    # ================================================================== #
    #  Live view — thickness and fading per iteration
    # ================================================================== #

    def _build_live(self) -> MarkerArray:
        now   = self.now()
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid   = 0

        for rid in (1, 2, 3):
            r, g, b = COLOR[rid]
            for it in (1, 2, 3):
                dq = self.buckets[(rid, it)]
                if not dq:
                    continue
                m = self._mk(stamp, f'live_r{rid}_i{it}', mid, WIDTH[it])
                mid += 1
                for (px, py, t) in dq:
                    age   = now - t
                    frac  = min(1.0, age / LIFETIME)
                    # Cubic: stays above 0.6 for first 80% of lifetime
                    alpha = max(0.12, 1.0 - frac ** 3)
                    m.points.append(self._pt(px, py, 0.01 * it))
                    m.colors.append(self._col(r, g, b, alpha))
                array.markers.append(m)

        return array

    # ================================================================== #
    #  Final view — ONLY iteration-3 paths, same colors as live
    # ================================================================== #

    def _build_final(self) -> MarkerArray:
        """
        Shows the 3 converged iteration-3 paths in Green/Red/Blue.
        All three are physically on Robot 1's lane (x≈-1.5) after the merge.
        Z offset separates them visually so all 3 are simultaneously visible.
        """
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid   = 100

        z_offsets = {1: 0.01, 2: 0.03, 3: 0.05}

        for rid in (1, 2, 3):
            r, g, b = COLOR[rid]
            dq      = self.buckets[(rid, 3)]
            if not dq:
                self.get_logger().warn(
                    f'[Mapper] No iter-3 drops found for Robot {rid} in final path.'
                )
                continue

            # Same thickness as iter-3 live (thickest)
            m = self._mk(stamp, f'final_r{rid}', mid, WIDTH[3])
            mid += 1
            for (px, py, _) in dq:
                m.points.append(self._pt(px, py, z_offsets[rid]))
                m.colors.append(self._col(r, g, b, 1.0))
            array.markers.append(m)

        return array

    # ================================================================== #
    #  Timer callbacks
    # ================================================================== #

    def live_tick(self):
        if self.sim_done:
            return
        self._evaporate()
        self.pub_live.publish(self._build_live())

    def final_tick(self):
        if self.final_array is not None:
            self.pub_final.publish(self.final_array)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PheromoneMapper())


if __name__ == '__main__':
    main()