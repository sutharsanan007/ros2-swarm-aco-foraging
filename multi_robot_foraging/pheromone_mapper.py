"""
pheromone_mapper.py
===================
/pheromone_trails  — Live colored trails per robot, per iteration.
                     Green=R1, Red=R2, Blue=R3.
                     Thickness grows each iteration: thin→medium→thick.
                     Age-based alpha fading (cubic curve, stays bright long).

/final_path_trails — Frozen after ALL 3 robots halt.
                     Shows ONLY the iter-3 paths (the converged result).
                     SAME visual style as live trails: Green/Red/Blue.
                     All three are on Robot 1's lane (x≈-1.5).
                     This is the clean ACO convergence story.
                     No individual pre-merge paths shown.

Pheromone encoding: pt.z = robot_id*10 + iteration
  11=R1I1, 12=R1I2, 13=R1I3, 21=R2I1, ..., 33=R3I3
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from collections import deque

# Live colors — same used in final display for consistency
COLOR = {
    1: (0.10, 0.95, 0.20),   # Green  (Robot 1 — winner)
    2: (0.95, 0.20, 0.10),   # Red    (Robot 2)
    3: (0.20, 0.45, 1.00),   # Blue   (Robot 3)
}

# Line width per iteration (grows to show reinforcement)
WIDTH = {1: 0.025, 2: 0.055, 3: 0.095}

# 300 s = 5 minutes — covers entire 3-iteration run without evaporating
LIFETIME = 300.0


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

        # Buckets keyed by (robot_id, iteration)
        self.buckets: dict = {
            (r, i): deque()
            for r in (1, 2, 3) for i in (1, 2, 3)
        }

        self.halted      = set()
        self.sim_done    = False
        self.final_array = None

        self.create_timer(0.25, self.live_tick)   # 4 Hz
        self.create_timer(0.50, self.final_tick)  # 2 Hz

        self.get_logger().info(
            f'PheromoneMapper online | lifetime={LIFETIME}s | '
            f'waiting for 3/3 halts'
        )

    # ================================================================== #
    #  Callbacks
    # ================================================================== #

    def drop_cb(self, msg: Point):
        code = int(round(msg.z))
        rid  = code // 10
        it   = code  % 10
        if rid not in (1, 2, 3) or it not in (1, 2, 3):
            return
        self.buckets[(rid, it)].append((msg.x, msg.y, self.now()))

    def halt_cb(self, msg: Int32):
        self.halted.add(msg.data)
        self.get_logger().info(
            f'[Mapper] R{msg.data} halted — {len(self.halted)}/3'
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
        c.r = float(r); c.g = float(g)
        c.b = float(b); c.a = float(a)
        return c

    @staticmethod
    def _pt(x, y, z=0.01) -> Point:
        p = Point(); p.x = float(x); p.y = float(y); p.z = float(z)
        return p

    # ================================================================== #
    #  Live view — fading trails, thicker each iteration
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
                    # Cubic curve: stays above 0.6 opacity for first 80% of life
                    alpha = max(0.12, 1.0 - frac ** 3)
                    m.points.append(self._pt(px, py, 0.01 * it))
                    m.colors.append(self._col(r, g, b, alpha))
                array.markers.append(m)

        return array

    # ================================================================== #
    #  Final view — ONLY iter-3 paths (all 3 converged to lane 1)
    #
    #  Same color scheme as live (Green/Red/Blue) so it's immediately
    #  readable. Same thickness as iter-3 live (thickest).
    #  Z offsets: R1=0.01, R2=0.03, R3=0.05 so all 3 are visible in RViz.
    #  No pre-merge paths shown — the display is ONLY the convergence result.
    # ================================================================== #

    def _build_final(self) -> MarkerArray:
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid   = 100

        # Only show iteration 3 for all robots
        for rid in (1, 2, 3):
            r, g, b  = COLOR[rid]
            dq       = self.buckets[(rid, 3)]
            if not dq:
                self.get_logger().warn(
                    f'[Mapper] No iter-3 drops for Robot {rid} — '
                    f'check robot completed its 3rd iteration.'
                )
                continue

            # Z offset so three lines are stacked visibly above each other
            z_off = {1: 0.01, 2: 0.03, 3: 0.05}[rid]
            # Final lines: same WIDTH as iter-3 live for visual consistency
            m = self._mk(stamp, f'final_r{rid}', mid, WIDTH[3])
            mid += 1
            for (px, py, _) in dq:
                m.points.append(self._pt(px, py, z_off))
                m.colors.append(self._col(r, g, b, 1.0))   # full opacity, frozen
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