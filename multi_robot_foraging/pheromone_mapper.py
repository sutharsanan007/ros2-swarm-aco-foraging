"""
pheromone_mapper.py
===================
Hub-and-Spoke + Ant Trail Convoy RViz visualization.

/pheromone_trails  — Live colored trails per robot per iteration.
  Robot 1 = GREEN  — North arm, all 3 iterations
  Robot 2 = RED    — East arm (iter 1&2), North arm (iter 3)
  Robot 3 = BLUE   — West arm (iter 1&2), North arm (iter 3)
  Line width grows each iteration: thin→medium→thick

/final_path_trails — Frozen after ALL 3 robots halt.
  Tells the complete ACO story in one clean picture:

  LAYER 1 — Original Arms (faded, thin):
    Red East arm   (Robot 2, iters 1&2) — shows where R2 started
    Blue West arm  (Robot 3, iters 1&2) — shows where R3 started
    Green North arm (Robot 1, all iters) — already optimal

  LAYER 2 — Pivot trails (semi-bright):
    Small colored arcs at hub showing R2 & R3 turning to face North

  LAYER 3 — Convoy convergence on North arm (thick, full opacity):
    GREEN (R1) at z=0.01 — thickest (winner/leader)
    RED   (R2) at z=0.03 — medium thick (joined convoy)
    BLUE  (R3) at z=0.05 — medium thick (joined convoy)
    All three stacked on x=0, y=0→1.5 — the ACO convergence moment

  This final picture shows the FULL story:
  "Three robots on three different arms → all converge to the shortest."
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from collections import deque

# Per-robot trail colors
COLOR = {
    1: (0.05, 0.85, 0.20),   # Green  (Robot 1 — winner)
    2: (0.90, 0.15, 0.08),   # Red    (Robot 2)
    3: (0.15, 0.35, 0.90),   # Blue   (Robot 3)
}

# Live trail widths grow each iteration to show pheromone reinforcement
LIVE_WIDTH = {1: 0.030, 2: 0.060, 3: 0.100}

# Final trail widths
FINAL_OWN_ARM_WIDTH    = 0.020   # own arm (iter 1&2) — thin, faded
FINAL_CONVOY_R1_WIDTH  = 0.120   # convoy leader (R1) — thickest
FINAL_CONVOY_R23_WIDTH = 0.080   # convoy followers (R2, R3)

LIFETIME = 400.0   # seconds — covers full 3-iteration run (~5-6 min)

# Robot 1's North arm x-coordinate — used to identify convoy points
NORTH_ARM_X   = 0.0
NORTH_ARM_TOL = 0.15   # ±15cm tolerance


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

        # (robot_id, iteration) → deque of (x, y, timestamp)
        self.buckets = {(r, i): deque() for r in (1, 2, 3) for i in (1, 2, 3)}

        self.halted      = set()
        self.sim_done    = False
        self.final_array = None

        self.create_timer(0.25, self.live_tick)
        self.create_timer(0.50, self.final_tick)

        self.get_logger().info(
            f'PheromoneMapper online | lifetime={LIFETIME}s | '
            f'hub-and-spoke + convoy layout'
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
        rid = msg.data
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
                'ACO convoy convergence frozen on /final_path_trails ***'
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
    def _mk(stamp, ns, mid, width, marker_type=Marker.LINE_STRIP) -> Marker:
        m = Marker()
        m.header.stamp       = stamp
        m.header.frame_id    = 'world'
        m.ns                 = ns
        m.id                 = mid
        m.type               = marker_type
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
    def _pt(x, y, z=0.02) -> Point:
        p = Point()
        p.x = float(x); p.y = float(y); p.z = float(z)
        return p

    def _is_on_north_arm(self, px, py):
        """True if this point is on Robot 1's North arm (x≈0, y>0)."""
        return (abs(px - NORTH_ARM_X) < NORTH_ARM_TOL and py >= -0.1)

    # ================================================================== #
    #  Live view: per-robot per-iteration colored LINE_STRIPs
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
                m = self._mk(stamp, f'live_r{rid}_i{it}', mid, LIVE_WIDTH[it])
                mid += 1
                for (px, py, t) in dq:
                    age   = now - t
                    frac  = min(1.0, age / LIFETIME)
                    # Cubic fade: stays bright ~80% of lifetime
                    alpha = max(0.12, 1.0 - frac ** 3)
                    m.points.append(self._pt(px, py, 0.01 * it))
                    m.colors.append(self._col(r, g, b, alpha))
                array.markers.append(m)

        return array

    # ================================================================== #
    #  Final view: the complete ACO story in one picture
    #
    #  Layer 1 — Original arms (iter 1&2, R2 East + R3 West): faded thin
    #  Layer 2 — North arm (all robots iter 3): thick, full color, stacked
    #
    #  The viewer immediately sees:
    #    "Two robots abandoned their arms and joined the green North arm."
    # ================================================================== #

    def _build_final(self) -> MarkerArray:
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid   = 200   # offset from live marker IDs

        # ── LAYER 1: R2 and R3 original arms (iter 1 & iter 2 only) ─────
        # Shows "where they came from" — thin, 35% opacity
        for rid in (2, 3):
            r, g, b = COLOR[rid]
            for it in (1, 2):
                dq = self.buckets[(rid, it)]
                if not dq:
                    continue
                m = self._mk(stamp, f'arm_r{rid}_i{it}', mid, FINAL_OWN_ARM_WIDTH)
                mid += 1
                for (px, py, _) in dq:
                    m.points.append(self._pt(px, py, 0.005))
                    m.colors.append(self._col(r, g, b, 0.35))
                array.markers.append(m)

        # ── LAYER 2: Robot 1 North arm (all 3 iterations) ─────────────────
        # Green arm visible in full across all iterations
        r, g, b = COLOR[1]
        all_r1_pts = []
        for it in (1, 2, 3):
            all_r1_pts.extend(
                [(px, py) for (px, py, _) in self.buckets[(1, it)]]
            )
        if all_r1_pts:
            m = self._mk(stamp, 'arm_r1_all', mid, FINAL_CONVOY_R1_WIDTH)
            mid += 1
            for (px, py) in all_r1_pts:
                m.points.append(self._pt(px, py, 0.01))
                m.colors.append(self._col(r, g, b, 1.0))
            array.markers.append(m)

        # ── LAYER 3: R2 & R3 convoy segment on North arm (iter 3 only) ───
        # These are the points from R2 and R3's iteration 3 trail that
        # fall on the North arm (x≈0). Thick, full opacity, stacked in Z.
        z_convoy = {2: 0.03, 3: 0.05}

        for rid in (2, 3):
            r, g, b = COLOR[rid]
            dq = self.buckets[(rid, 3)]
            if not dq:
                self.get_logger().warn(
                    f'[Mapper] No iter-3 drops for Robot {rid}.'
                )
                continue

            # Split into own-arm (pre-pivot) and north-arm (post-pivot) segments
            north_pts = [
                (px, py) for (px, py, _) in dq
                if self._is_on_north_arm(px, py)
            ]

            if north_pts:
                m = self._mk(
                    stamp,
                    f'convoy_r{rid}',
                    mid,
                    FINAL_CONVOY_R23_WIDTH
                )
                mid += 1
                for (px, py) in north_pts:
                    m.points.append(self._pt(px, py, z_convoy[rid]))
                    m.colors.append(self._col(r, g, b, 1.0))
                array.markers.append(m)
            else:
                # Fallback: show all iter-3 drops if north-arm filter has no hits
                self.get_logger().warn(
                    f'[Mapper] R{rid} iter-3 has no north-arm points — '
                    f'showing full iter-3 trail as fallback.'
                )
                m = self._mk(
                    stamp,
                    f'convoy_r{rid}_fallback',
                    mid,
                    FINAL_CONVOY_R23_WIDTH
                )
                mid += 1
                for (px, py, _) in dq:
                    m.points.append(self._pt(px, py, z_convoy[rid]))
                    m.colors.append(self._col(r, g, b, 0.8))
                array.markers.append(m)

        # ── HUB MARKER: large sphere at (0,0) to highlight convergence ───
        hub = Marker()
        hub.header.stamp     = stamp
        hub.header.frame_id  = 'world'
        hub.ns               = 'hub'
        hub.id               = mid
        hub.type             = Marker.SPHERE
        hub.action           = Marker.ADD
        hub.pose.position.x  = 0.0
        hub.pose.position.y  = 0.0
        hub.pose.position.z  = 0.04
        hub.pose.orientation.w = 1.0
        hub.scale.x = hub.scale.y = hub.scale.z = 0.18
        hub.color = self._col(1.0, 0.85, 0.0, 1.0)   # gold sphere at hub
        array.markers.append(hub)
        mid += 1

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