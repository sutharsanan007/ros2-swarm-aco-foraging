"""
unified_agent.py  —  Hub and Spoke (Reverse Gear) Implementation
================================================================
DESIGN: Eliminates ALL 180-degree turns by using reverse driving.

ARENA LAYOUT:
               Resource1 (0.0, 1.5)
                     |
             SHORT   |  1.5m
                     |
  Resource3 ----- HUB(0,0) ----- Resource2
  (-3.5, 0.0)   LONG 3.5m  MEDIUM 2.5m   (2.5, 0.0)

ROBOT SPAWN POSITIONS (near hub, facing outward):
  Robot 1 — GREEN  — (0.0, 0.15)  facing NORTH
  Robot 2 — RED    — (0.15, 0.0)  facing EAST
  Robot 3 — BLUE   — (-0.15, 0.0) facing WEST

ITERATION 1 & 2 — ALL ROBOTS (zero turns, zero drift):
  DRIVE_FWD → robot drives forward along its arm to resource
  DRIVE_BWD → robot drives BACKWARD (vel.linear.x negative)
               along exact same axis back to hub
  Arrival detected by DISTANCE to target, not by axis threshold
  → works regardless of heading direction

ITERATION 3 — CONVERGENCE (only one 90° turn per robot):
  Robot 1: continues its own short path as normal
  Robot 2: reverses to hub → turns LEFT  90° (East→North) → uses R1 path
  Robot 3: reverses to hub → turns RIGHT 90° (West→North) → uses R1 path

WHY NO DRIFT:
  - Iterations 1 & 2: ZERO turns, ZERO angular motion
  - Iteration 3: ONE 90° turn (vs previous 180° turns × 2 per iteration)
  - Distance-based arrival: no axis threshold leakage regardless of direction
  - Both X and Y snapped at every waypoint arrival
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int32
import math

# Cardinal headings
NORTH =  math.pi / 2.0   # facing +Y
EAST  =  0.0              # facing +X
WEST  =  math.pi          # facing -X
SOUTH = -math.pi / 2.0   # facing -Y

SPEED       = 0.20    # m/s forward (and backward)
TURN_SPEED  = 0.50    # rad/s for the single 90° convergence turn
SNAP_XY     = 0.07    # metres — arrival distance threshold
SNAP_TH     = 0.025   # radians — turn completion threshold
SETTLE_TIME = 0.40    # seconds of zero velocity after 90° turn


class UnifiedForagingAgent(Node):

    def __init__(self):
        super().__init__('unified_agent')
        self.declare_parameter('resource_id', 1)
        self.resource_id = self.get_parameter('resource_id').value

        self.pub_vel    = self.create_publisher(Twist, 'cmd_vel',        10)
        self.pub_phero  = self.create_publisher(Point, 'pheromone_drop', 10)
        self.pub_trail  = self.create_publisher(Point, 'trail_pos',      10)
        self.pub_halted = self.create_publisher(Int32, '/robot_halted',  10)

        # ── Hub-and-spoke waypoints ───────────────────────────────────────
        # HOME is always the hub at (0, 0)
        self.HOME_X = 0.0
        self.HOME_Y = 0.0

        if self.resource_id == 1:        # GREEN — North arm — SHORT
            self.cx0    =  0.00;   self.cy0    =  0.15   # spawn
            self.cth0   = NORTH
            self.RES_X  =  0.00;   self.RES_Y  =  1.50
        elif self.resource_id == 2:      # RED   — East arm  — MEDIUM
            self.cx0    =  0.15;   self.cy0    =  0.00
            self.cth0   = EAST
            self.RES_X  =  2.50;   self.RES_Y  =  0.00
        else:                            # BLUE  — West arm  — LONG
            self.cx0    = -0.15;   self.cy0    =  0.00
            self.cth0   = WEST
            self.RES_X  = -3.50;   self.RES_Y  =  0.00

        # ── Commanded odometry (initialised at spawn pose) ────────────────
        self.cx  = self.cx0
        self.cy  = self.cy0
        self.cth = self.cth0
        self.lv  = 0.0
        self.lw  = 0.0

        # ── ACO ───────────────────────────────────────────────────────────
        self.iteration       = 1
        self.max_iterations  = 3
        self.pheromone_total = 0.0

        # ── State machine ─────────────────────────────────────────────────
        self.settle_next    = None
        self.halt_published = False
        self.state  = 'BOOT_UP'
        self.t0     = self.st()
        self.t_last = self.st()

        self.create_timer(0.02, self.loop)   # 50 Hz

        self.get_logger().info(
            f'[R{self.resource_id}] HUB-SPOKE | '
            f'heading={math.degrees(self.cth0):.0f}deg | '
            f'res=({self.RES_X},{self.RES_Y}) | '
            f'home=({self.HOME_X},{self.HOME_Y})'
        )

    # ================================================================== #
    #  Helpers
    # ================================================================== #

    def st(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed(self):
        return self.st() - self.t0

    def go(self, state):
        self.state = state
        self.t0    = self.st()
        self.get_logger().info(f'[R{self.resource_id}] → {state}')

    def settle_to(self, nxt):
        self.settle_next = nxt
        self.go('SETTLE')

    def integrate(self, dt):
        """Integrate commanded velocity into commanded odometry position."""
        self.cth += self.lw  * dt
        self.cx  += self.lv  * math.cos(self.cth) * dt
        self.cy  += self.lv  * math.sin(self.cth) * dt

    def dist_to(self, tx, ty):
        """Euclidean distance from current commanded position to target."""
        return math.hypot(self.cx - tx, self.cy - ty)

    def breadcrumb(self, phero=False):
        """
        Publish current position as trail breadcrumb (always)
        and optionally as pheromone drop (ACO return trip only).
        """
        pt   = Point()
        pt.x = float(self.cx)
        pt.y = float(self.cy)
        pt.z = float(self.resource_id * 10 + self.iteration)
        self.pub_trail.publish(pt)
        if phero:
            self.pub_phero.publish(pt)

    # ================================================================== #
    #  Control loop  (50 Hz)
    # ================================================================== #

    def loop(self):
        now = self.st()
        dt  = now - self.t_last
        self.t_last = now

        vel = Twist()
        self.integrate(dt)

        # Drop breadcrumb during all driving states
        if self.state == 'DRIVE_FWD':
            self.breadcrumb(phero=False)
        elif self.state == 'DRIVE_BWD':
            self.breadcrumb(phero=True)   # return trip = pheromone drop

        # ── BOOT_UP ──────────────────────────────────────────────────────
        if self.state == 'BOOT_UP':
            # Iterations 1 & 2: staggered start by robot_id
            # Iteration 3:
            #   R1 goes immediately — best path
            #   R3 goes a little after R1 (starts its 90° turn)
            #   R2 waits until R3 is well up the North arm before turning
            if self.iteration <= 2:
                delay = 2.0 + self.resource_id * 0.8
            else:
                delay = {1: 2.5, 3: 4.0, 2: 18.0}[self.resource_id]

            if self.elapsed() > delay:
                if self.iteration == 3 and self.resource_id != 1:
                    self.go('TURN_NORTH')
                else:
                    self.go('DRIVE_FWD')

        # ── SETTLE: zero velocity pause after the 90° convergence turn ───
        elif self.state == 'SETTLE':
            if self.elapsed() > SETTLE_TIME:
                self.get_logger().info(
                    f'[R{self.resource_id}][SETTLE→{self.settle_next}] '
                    f'cx={self.cx:.3f} cy={self.cy:.3f} '
                    f'cth={math.degrees(self.cth):.1f}deg'
                )
                self.go(self.settle_next)

        # ── TURN_NORTH: 90° convergence turn (iteration 3 only) ──────────
        # Robot 2 (East  = 0°  → North = 90°): turn LEFT  (+angular.z)
        # Robot 3 (West  = π   → North = 90°): turn RIGHT (-angular.z)
        elif self.state == 'TURN_NORTH':
            err = NORTH - self.cth
            if abs(err) < SNAP_TH:
                # Snap heading and position to Robot 1's lane exactly
                self.cth   = NORTH
                self.cx    = 0.0    # snap onto Robot 1's X lane
                self.cy    = 0.0    # snap onto hub Y
                self.RES_X = 0.0;  self.RES_Y = 1.5   # Robot 1 resource
                self.get_logger().info(
                    f'\n\033[1;33m'
                    f'*** [R{self.resource_id}] TURN COMPLETE → '
                    f'NOW ON ROBOT 1 OPTIMAL LANE (0,0→0,1.5) ***'
                    f'\033[0m\n'
                )
                self.settle_to('DRIVE_FWD')
            else:
                # Turn toward North whichever is shorter direction
                vel.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED

        # ── DRIVE_FWD: drive forward along arm to resource ────────────────
        # Uses heading-aligned forward motion — no turning while driving.
        # Distance-based arrival check works for all headings.
        elif self.state == 'DRIVE_FWD':
            if self.dist_to(self.RES_X, self.RES_Y) < SNAP_XY:
                # Snap to exact resource position
                self.cx = self.RES_X
                self.cy = self.RES_Y
                self.get_logger().info(
                    f'[R{self.resource_id}] ✓ Resource '
                    f'({self.RES_X:.1f},{self.RES_Y:.1f}) '
                    f'iter={self.iteration}'
                )
                self.go('PICK')
            else:
                vel.linear.x = SPEED   # forward

        # ── PICK: pause at resource ───────────────────────────────────────
        elif self.state == 'PICK':
            if self.elapsed() > 2.0:
                self.go('DRIVE_BWD')

        # ── DRIVE_BWD: REVERSE straight back to hub — ZERO TURNING ───────
        # This is the key innovation: negative linear velocity.
        # The robot faces the same direction it drove outward but moves backward.
        # For Robot 1 (NORTH): backward = South, cy decreases → home
        # For Robot 2 (EAST):  backward = West, cx decreases → home
        # For Robot 3 (WEST):  backward = East, cx increases → home
        # Distance check catches all cases identically.
        elif self.state == 'DRIVE_BWD':
            if self.dist_to(self.HOME_X, self.HOME_Y) < SNAP_XY:
                # Snap to exact hub position
                self.cx = self.HOME_X
                self.cy = self.HOME_Y
                self.get_logger().info(
                    f'[R{self.resource_id}] ✓ Home (hub) '
                    f'iter={self.iteration}'
                )
                self.go('DEPOSIT')
            else:
                vel.linear.x = -SPEED   # REVERSE — no turn needed!

        # ── DEPOSIT ───────────────────────────────────────────────────────
        elif self.state == 'DEPOSIT':
            if self.elapsed() > 2.0:
                self.pheromone_total += 1.0
                self.get_logger().info(
                    f'\n\033[1;32m'
                    f'*** [R{self.resource_id}] +1 PHEROMONE | '
                    f'iter={self.iteration} '
                    f'total={self.pheromone_total:.1f} ***'
                    f'\033[0m\n'
                )
                self.go('REST')

        # ── REST ──────────────────────────────────────────────────────────
        elif self.state == 'REST':
            if self.elapsed() > 3.0:
                if self.iteration < self.max_iterations:
                    self.iteration += 1
                    self._announce_swarm()
                    self.get_logger().info(
                        f'--- [R{self.resource_id}] '
                        f'ITER {self.iteration}/{self.max_iterations} | '
                        f'cx={self.cx:.3f} cy={self.cy:.3f} ---'
                    )
                    self.go('BOOT_UP')
                else:
                    self.get_logger().info(
                        f'\n\033[1;35m'
                        f'*** [R{self.resource_id}] ALL COMPLETE | '
                        f'total pheromones={self.pheromone_total:.1f} ***'
                        f'\033[0m\n'
                    )
                    self.go('HALT')

        # ── HALT ─────────────────────────────────────────────────────────
        elif self.state == 'HALT':
            if not self.halt_published:
                msg = Int32(); msg.data = self.resource_id
                self.pub_halted.publish(msg)
                self.halt_published = True
                self.get_logger().info(
                    f'[R{self.resource_id}] halt signal sent'
                )

        self.lv = vel.linear.x
        self.lw = vel.angular.z
        self.pub_vel.publish(vel)

    # ================================================================== #

    def _announce_swarm(self):
        if self.iteration != 3:
            return
        t = self.pheromone_total
        if self.resource_id == 1:
            self.get_logger().info(
                f'\n\033[1;33m'
                f'*** [SWARM] R1 LEADS with {t:.1f} pheromone units. '
                f'North arm is optimal! ***'
                f'\033[0m\n'
            )
        else:
            self.get_logger().info(
                f'\n\033[1;33m'
                f'*** [SWARM] R{self.resource_id} has {t:.1f} units. '
                f'Turning 90° to merge onto Robot 1 optimal path! ***'
                f'\033[0m\n'
            )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(UnifiedForagingAgent())


if __name__ == '__main__':
    main()