"""
pheromone_mapper.py
===================
/pheromone_trails  — Live fading colored LINE_STRIP per robot, per iteration.
                     Thickness increases each iteration. Age-based alpha fading.
                     Robot 1=Green, Robot 2=Red, Robot 3=Blue.

/final_path_trails — Published only after ALL 3 robots halt.
                     Shows ONLY the convergence story on Robot 1's lane:
                       • One thick GOLD bar  (Robot 1's full path)
                       • One WHITE bar       (Robot 2 merged path, slightly thinner)
                       • One CYAN bar        (Robot 3 merged path, slightly thinner)
                     Spaced 4 cm apart in Z so all 3 are simultaneously visible.
                     Own-lane iterations 1&2 of Robots 2 & 3 are NOT shown —
                     the display is clean and focused on the ACO convergence.

Encoding: pt.z = robot_id * 10 + iteration
  11=R1I1, 12=R1I2, 13=R1I3
  21=R2I1, 22=R2I2, 23=R2I3
  31=R3I1, 32=R3I2, 33=R3I3
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from collections import deque

# ── Per-robot live colors ─────────────────────────────────────────────────
LIVE_COLOR = {
    1: (0.10, 0.95, 0.20),   # Green
    2: (0.95, 0.20, 0.10),   # Red
    3: (0.20, 0.45, 1.00),   # Blue
}

# ── Line widths per iteration (live) ────────────────────────────────────
ITER_W = {1: 0.025, 2: 0.055, 3: 0.095}

# ── How long breadcrumbs live before evaporating ─────────────────────────
# 300 s = 5 minutes, covers the full 3-iteration run
LIFETIME = 300.0

# ── Lane-1 detection for final merge highlight ───────────────────────────
LANE1_X   = -1.5
LANE_TOL  =  0.35    # ±35 cm — generous to catch slight path deviation


class PheromoneMapper(Node):

    def __init__(self):
        super().__init__('pheromone_mapper')

        for rn in (1, 2, 3):
            self.create_subscription(
                Point, f'/robot{rn}/pheromone_drop', self.drop_cb, 10
            )
        self.create_subscription(Int32, '/robot_halted', self.halt_cb, 10)

        self.pub_live  = self.create_publisher(MarkerArray, '/pheromone_trails',  10)
        self.pub_final = self.create_publisher(MarkerArray, '/final_path_trails', 10)

        # Buckets: (robot_id, iteration) → deque of (x, y, timestamp)
        self.buckets: dict = {}
        for r in (1, 2, 3):
            for i in (1, 2, 3):
                self.buckets[(r, i)] = deque()

        self.halted      = set()
        self.sim_done    = False
        self.final_array = None

        self.create_timer(0.25, self.live_tick)    # 4 Hz live
        self.create_timer(0.50, self.final_tick)   # 2 Hz final

        self.get_logger().info(
            'PheromoneMapper online | '
            f'lifetime={LIFETIME}s | waiting for 3/3 halts'
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
        self.halted.add(msg.data)
        self.get_logger().info(
            f'[Mapper] R{msg.data} halted — {len(self.halted)}/3'
        )
        if len(self.halted) >= 3 and not self.sim_done:
            self.sim_done    = True
            self.final_array = self._build_final()
            self.get_logger().info(
                '\n\033[1;36m'
                '*** ALL ROBOTS COMPLETE — final convergence path frozen! '
                'Subscribe to /final_path_trails in RViz ***'
                '\033[0m\n'
            )

    # ================================================================== #
    #  Helpers
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
    #  Live view — fading colored trails, thicker each iteration
    # ================================================================== #

    def _build_live(self) -> MarkerArray:
        now   = self.now()
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid   = 0

        for rid in (1, 2, 3):
            r, g, b = LIVE_COLOR[rid]
            for it in (1, 2, 3):
                dq = self.buckets[(rid, it)]
                if not dq:
                    continue
                m = self._mk(stamp, f'r{rid}i{it}', mid, ITER_W[it])
                mid += 1
                for (px, py, t) in dq:
                    age   = now - t
                    frac  = min(1.0, age / LIFETIME)
                    # Cubic fade: bright for first ~80%, then drops
                    alpha = max(0.12, 1.0 - frac ** 3)
                    m.points.append(self._pt(px, py, 0.01 * it))
                    m.colors.append(self._col(r, g, b, alpha))
                array.markers.append(m)

        return array

    # ================================================================== #
    #  Final view — ONLY the merged convergence on Robot 1's lane
    #
    #  Shows three parallel bars at the same X lane, offset in Z height:
    #    GOLD  z=0.01  — Robot 1 (winner, thickest)
    #    WHITE z=0.03  — Robot 2 iteration-3 merged segment
    #    CYAN  z=0.05  — Robot 3 iteration-3 merged segment
    #
    #  Robots 2 & 3 own-lane segments are intentionally NOT rendered —
    #  the final view is about convergence, not full history.
    # ================================================================== #

    def _build_final(self) -> MarkerArray:
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid   = 200   # offset well away from live namespace

        # ── GOLD: Robot 1 full path (all 3 iterations, thickest) ─────────
        r1_pts = []
        for it in (1, 2, 3):
            r1_pts.extend([(px, py) for (px, py, _) in self.buckets[(1, it)]])

        if r1_pts:
            m = self._mk(stamp, 'final_r1_gold', mid, 0.14)
            mid += 1
            for (px, py) in r1_pts:
                m.points.append(self._pt(px, py, 0.01))
                m.colors.append(self._col(1.0, 0.80, 0.0, 1.0))   # gold
            array.markers.append(m)

        # ── WHITE: Robot 2 iteration-3 trail on Robot 1's lane ───────────
        r2_merge = [
            (px, py)
            for (px, py, _) in self.buckets[(2, 3)]
            if abs(px - LANE1_X) < LANE_TOL
        ]
        # Also include Robot 2's full iter-3 trail (the whole path IS on lane 1
        # after merge, so collect all iter-3 points)
        r2_all_i3 = [(px, py) for (px, py, _) in self.buckets[(2, 3)]]
        r2_show   = r2_all_i3 if r2_all_i3 else r2_merge

        if r2_show:
            m = self._mk(stamp, 'final_r2_white', mid, 0.085)
            mid += 1
            for (px, py) in r2_show:
                m.points.append(self._pt(px, py, 0.03))
                m.colors.append(self._col(1.0, 1.0, 1.0, 0.92))   # white
            array.markers.append(m)

        # ── CYAN: Robot 3 iteration-3 trail on Robot 1's lane ────────────
        r3_all_i3 = [(px, py) for (px, py, _) in self.buckets[(3, 3)]]
        r3_merge  = [
            (px, py)
            for (px, py, _) in self.buckets[(3, 3)]
            if abs(px - LANE1_X) < LANE_TOL
        ]
        r3_show = r3_all_i3 if r3_all_i3 else r3_merge

        if r3_show:
            m = self._mk(stamp, 'final_r3_cyan', mid, 0.085)
            mid += 1
            for (px, py) in r3_show:
                m.points.append(self._pt(px, py, 0.05))
                m.colors.append(self._col(0.2, 1.0, 1.0, 0.92))   # cyan
            array.markers.append(m)

        # ── Legend markers: three horizontal bars as a visual key ─────────
        # Placed at world(1.0, -2.5) so they don't overlap robot paths
        legend = [
            ('GOLD  = Robot 1 (optimal)',  0.01, (1.0, 0.80, 0.0)),
            ('WHITE = Robot 2 (merged)',   0.03, (1.0, 1.00, 1.0)),
            ('CYAN  = Robot 3 (merged)',   0.05, (0.2, 1.00, 1.0)),
        ]
        for i, (_, z_off, (lr, lg, lb)) in enumerate(legend):
            m = self._mk(stamp, f'legend_{i}', mid, 0.06)
            mid += 1
            for lx in (1.0, 1.5, 2.0, 2.5):
                m.points.append(self._pt(lx, -2.5, z_off))
                m.colors.append(self._col(lr, lg, lb, 1.0))
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