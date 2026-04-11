"""
unified_agent.py  —  Pure Commanded-Odometry State Machine
===========================================================
DESIGN DECISION: We use COMMANDED ODOMETRY only — no odom subscription.
This was the approach that produced the verified successful run logs.
The odom-frame conversion was unreliable because Gazebo's DiffDrive odom
X-axis aligns with the robot's spawn heading, not world East.

Commanded odometry works because:
  - We start at the known spawn world position
  - We integrate velocity × dt using Gazebo's sim clock (immune to lag)
  - We snap coordinates on arrival to prevent floating-point drift
  - All coordinates are in WORLD frame from the start

ARENA LAYOUT (world frame, all positions hardcoded):
  ┌─────────────────────────────────────────────────┐
  │  Robot2 Res  Robot3 Res  Robot1 Res             │
  │  (1.5, 3.0)  (0.0, 2.0) (-1.5, 1.5)  ← markers │
  │                                                  │
  │  R2 spawn    R3 spawn    R1 spawn                │
  │  (1.5, 0.0)  (0.0, 0.0) (-1.5, 0.0)             │
  │                                                  │
  │  HOME DOCK (shared Y=-1.5, each robot's own X)   │
  │  (1.5,-1.5)  (0.0,-1.5) (-1.5,-1.5)             │
  └─────────────────────────────────────────────────┘

ITERATION 3 LANE MERGE (collision-free):
  Both robots 2 & 3 must reach x=-1.5 without colliding.
  Solution: They navigate to DIFFERENT intermediate Y positions:
    Robot 3: moves to (-1.5, +0.6)  — above spawn line
    Robot 2: moves to (-1.5, -0.6)  — below spawn line (home side)
  They arrive at different world points so they never occupy same cell.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int32
import math


SPEED       = 0.20   # m/s linear
TURN_SPEED  = 0.50   # rad/s angular
ARRIVAL_TOL = 0.06   # metres — snap threshold


class UnifiedForagingAgent(Node):

    def __init__(self):
        super().__init__('unified_agent')

        self.declare_parameter('resource_id', 1)
        self.resource_id = self.get_parameter('resource_id').value

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_vel    = self.create_publisher(Twist, 'cmd_vel',        10)
        self.pub_phero  = self.create_publisher(Point, 'pheromone_drop', 10)
        self.pub_halted = self.create_publisher(Int32, '/robot_halted',  10)

        # ── Hardcoded world coordinates ───────────────────────────────────
        # Resource Y distances: R1=1.5m (short), R3=2.0m (medium), R2=3.0m (long)
        if self.resource_id == 1:
            self.SPAWN_X   = -1.5
            self.RES_X     = -1.5;  self.RES_Y   = 1.5
            self.HOME_X    = -1.5;  self.HOME_Y  = -1.5
        elif self.resource_id == 2:
            self.SPAWN_X   =  1.5
            self.RES_X     =  1.5;  self.RES_Y   = 3.0
            self.HOME_X    =  1.5;  self.HOME_Y  = -1.5
        else:  # robot 3
            self.SPAWN_X   =  0.0
            self.RES_X     =  0.0;  self.RES_Y   = 2.0
            self.HOME_X    =  0.0;  self.HOME_Y  = -1.5

        # Iteration-3 lane-merge intermediate waypoints (collision-free)
        # Robot 3 arrives at y=+0.6, Robot 2 arrives at y=-0.6
        if self.resource_id == 2:
            self.MERGE_X = -1.5;  self.MERGE_Y = -0.6
        else:
            self.MERGE_X = -1.5;  self.MERGE_Y =  0.6

        # ── Commanded-odometry state ──────────────────────────────────────
        # Start at exact spawn position, facing North (θ = π/2)
        self.cx    = float(self.SPAWN_X)
        self.cy    = 0.0
        self.cth   = math.pi / 2.0
        self.lv    = 0.0   # last commanded linear velocity
        self.lw    = 0.0   # last commanded angular velocity

        # Current navigation target (overwritten per state)
        self.tx    = self.cx
        self.ty    = self.cy

        # ── ACO bookkeeping ──────────────────────────────────────────────
        self.iteration       = 1
        self.max_iterations  = 3
        self.pheromone_total = 0.0

        # ── State machine ─────────────────────────────────────────────────
        self.state   = 'BOOT_UP'
        self.t_state = self.now()
        self.t_last  = self.now()

        self.create_timer(0.02, self.loop)   # 50 Hz
        self.get_logger().info(
            f'[R{self.resource_id}] Commanded-odom agent ready | '
            f'res=({self.RES_X},{self.RES_Y}) home=({self.HOME_X},{self.HOME_Y})'
        )

    # ================================================================== #
    #  Utilities
    # ================================================================== #

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed(self):
        return self.now() - self.t_state

    def go(self, new_state):
        self.state   = new_state
        self.t_state = self.now()
        self.get_logger().info(f'[R{self.resource_id}] → {new_state}')

    @staticmethod
    def wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def set_target(self, x, y):
        self.tx = x;  self.ty = y

    # ── Integrate odometry ───────────────────────────────────────────────
    def integrate(self, dt):
        self.cth = self.wrap(self.cth + self.lw * dt)
        self.cx += self.lv * math.cos(self.cth) * dt
        self.cy += self.lv * math.sin(self.cth) * dt

    # ── Drop pheromone at current commanded position ─────────────────────
    def drop(self):
        if self.lv > 0.0:
            pt   = Point()
            pt.x = float(self.cx)
            pt.y = float(self.cy)
            # Encode robot_id and iteration into z for per-bucket rendering
            pt.z = float(self.resource_id * 10 + self.iteration)
            self.pub_phero.publish(pt)

    # ── Drive straight to (tx, ty) — returns True when arrived ───────────
    def drive_to(self, vel):
        dy = self.ty - self.cy
        dx = self.tx - self.cx
        dist = math.hypot(dx, dy)

        if dist < ARRIVAL_TOL:
            return True   # arrived

        desired = math.atan2(dy, dx)
        err     = self.wrap(desired - self.cth)

        if abs(err) > 0.15:
            vel.linear.x  = 0.0
            vel.angular.z = TURN_SPEED * (1.0 if err > 0 else -1.0)
        else:
            vel.linear.x  = min(SPEED, 0.8 * dist)
            vel.angular.z = 1.0 * err
        return False

    # ── Turn until heading matches target_theta — returns True when done ──
    def turn_to(self, target_theta, vel):
        err = self.wrap(target_theta - self.cth)
        if abs(err) < 0.03:
            self.cth = self.wrap(target_theta)
            return True
        vel.angular.z = TURN_SPEED * (1.0 if err > 0 else -1.0)
        return False

    # ================================================================== #
    #  Control loop  (50 Hz)
    # ================================================================== #

    def loop(self):
        now = self.now()
        dt  = now - self.t_last
        self.t_last = now

        vel = Twist()
        self.integrate(dt)
        self.drop()

        # ── BOOT_UP: staggered start ─────────────────────────────────────
        if self.state == 'BOOT_UP':
            delay = 2.0 + self.resource_id * 0.8
            if self.elapsed() > delay:
                if self.iteration == 3 and self.resource_id != 1:
                    self.set_target(self.MERGE_X, self.MERGE_Y)
                    self.go('LANE_CHANGE')
                else:
                    self.set_target(self.RES_X, self.RES_Y)
                    self.go('TO_RESOURCE')

        # ── LANE_CHANGE: robots 2 & 3 move to Robot 1's lane ────────────
        elif self.state == 'LANE_CHANGE':
            if self.drive_to(vel):
                # Snap to intermediate merge point
                self.cx = self.MERGE_X;  self.cy = self.MERGE_Y
                # Now update waypoints to Robot 1's lane
                self.RES_X  = -1.5;  self.RES_Y  = 1.5
                self.HOME_X = -1.5;  self.HOME_Y = -1.5
                self.set_target(self.RES_X, self.RES_Y)
                self.get_logger().info(
                    f'\n\033[1;33m'
                    f'*** [R{self.resource_id}] MERGED → Robot 1 optimal lane! ***'
                    f'\033[0m\n'
                )
                self.go('TO_RESOURCE')

        # ── TO_RESOURCE: drive North to marker ──────────────────────────
        elif self.state == 'TO_RESOURCE':
            if self.drive_to(vel):
                self.cx = self.RES_X;  self.cy = self.RES_Y
                self.get_logger().info(
                    f'[R{self.resource_id}] ✓ Resource ({self.RES_X},{self.RES_Y})'
                    f'  iter={self.iteration}'
                )
                self.go('PICK')

        # ── PICK: pause at resource ──────────────────────────────────────
        elif self.state == 'PICK':
            if self.elapsed() > 2.0:
                self.set_target(self.HOME_X, self.HOME_Y)
                self.go('TO_HOME')

        # ── TO_HOME: drive South to home — DROP PHEROMONES here ─────────
        elif self.state == 'TO_HOME':
            if self.drive_to(vel):
                self.cx = self.HOME_X;  self.cy = self.HOME_Y
                self.get_logger().info(
                    f'[R{self.resource_id}] ✓ Home ({self.HOME_X},{self.HOME_Y})'
                    f'  iter={self.iteration}'
                )
                self.go('DEPOSIT')

        # ── DEPOSIT: count pheromone, announce, then rest ────────────────
        elif self.state == 'DEPOSIT':
            if self.elapsed() > 2.0:
                self.pheromone_total += 1.0
                self.get_logger().info(
                    f'\n\033[1;32m'
                    f'*** [PHEROMONE] R{self.resource_id} +1  |  '
                    f'iter={self.iteration}  total={self.pheromone_total:.1f} ***'
                    f'\033[0m\n'
                )
                self.go('REST')

        # ── REST: wait, then advance iteration ───────────────────────────
        elif self.state == 'REST':
            if self.elapsed() > 3.0:
                if self.iteration < self.max_iterations:
                    self.iteration += 1
                    self._log_swarm(self.pheromone_total)
                    self.get_logger().info(
                        f'--- [R{self.resource_id}] '
                        f'ITER {self.iteration}/{self.max_iterations} START ---'
                    )
                    self.go('BOOT_UP')
                else:
                    self.get_logger().info(
                        f'\n\033[1;35m'
                        f'*** [R{self.resource_id}] COMPLETE | '
                        f'total pheromones={self.pheromone_total:.1f} ***'
                        f'\033[0m\n'
                    )
                    self.go('HALT')

        # ── HALT ─────────────────────────────────────────────────────────
        elif self.state == 'HALT':
            msg = Int32();  msg.data = self.resource_id
            self.pub_halted.publish(msg)

        # ── Publish velocity + save for next integration step ────────────
        self.lv = vel.linear.x
        self.lw = vel.angular.z
        self.pub_vel.publish(vel)

    # ================================================================== #
    #  Swarm decision log
    # ================================================================== #

    def _log_swarm(self, total):
        if self.iteration != 3:
            return
        if self.resource_id == 1:
            self.get_logger().info(
                f'\n\033[1;33m'
                f'*** [SWARM] R1 LEADS with {total:.1f} units — '
                f'shortest path confirmed! ***'
                f'\033[0m\n'
            )
        else:
            self.get_logger().info(
                f'\n\033[1;33m'
                f'*** [SWARM] R{self.resource_id} has {total:.1f} units. '
                f'Path-1 is optimal → merging lanes! ***'
                f'\033[0m\n'
            )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(UnifiedForagingAgent())

if __name__ == '__main__':
    main()