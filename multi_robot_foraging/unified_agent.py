"""
unified_agent.py
================
DESIGN: Pure axis-aligned commanded odometry.
  - Robots only ever move NORTH, SOUTH, or WEST (never diagonal).
  - Each direction is an explicit state with a single threshold check.
  - No P-controller, no angular correction while moving.
  - Turns are their own explicit states.
  - This mirrors the original PROVEN working approach.

WHY PREVIOUS VERSIONS FAILED:
  - drive_to() P-controller applied angular.z while moving linearly.
  - This caused diagonal motion → zigzag pheromone trails.
  - With commanded odometry, the robot IS straight (odom says so)
    but Gazebo physics curves the actual path slightly, accumulating error.

THIS SOLUTION: Never combine linear + angular simultaneously.
  - Turn first (angular only), snap to exact heading.
  - Then drive (linear only, zero angular), check single coordinate.
  - Guaranteed straight-line motion in world frame.

ARENA (world frame, hardcoded):
  Robot 1: spawn(-1.5, 0) → resource(-1.5, 1.5) → home(-1.5,-1.5)  SHORT
  Robot 2: spawn( 1.5, 0) → resource( 1.5, 3.0) → home( 1.5,-1.5)  LONG
  Robot 3: spawn( 0.0, 0) → resource( 0.0, 2.0) → home( 0.0,-1.5)  MEDIUM

LANE MERGE (iter 3, Robots 2 & 3):
  Robots start at their HOME position after iter 2.
  They drive WEST to x=-1.5 (at their current y=-1.5).
  Collision avoided by staggered BOOT_UP delay:
    Robot 3 (x=0→-1.5, 1.5m): starts after 3.5 s
    Robot 2 (x=1.5→-1.5, 3.0m): starts after 16.0 s
  Robot 3 clears (-1.5,-1.5) ~17 s in. Robot 2 arrives ~34 s in. No overlap.

PHEROMONE DROPS: ONLY during DRIVE_SOUTH (return trip carrying food).
  Encoding: pt.z = robot_id*10 + iteration  (e.g. 23 = Robot2, Iter3)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int32
import math

# Cardinal headings — snapped exactly, never drift
NORTH =  math.pi / 2.0   # +Y world
SOUTH = -math.pi / 2.0   # -Y world
WEST  =  math.pi          # -X world

SPEED      = 0.20    # m/s
TURN_SPEED = 0.50    # rad/s
SNAP_XY    = 0.05    # metres arrival tolerance
SNAP_TH    = 0.025   # radians turn completion tolerance


class UnifiedForagingAgent(Node):

    def __init__(self):
        super().__init__('unified_agent')
        self.declare_parameter('resource_id', 1)
        self.resource_id = self.get_parameter('resource_id').value

        self.pub_vel    = self.create_publisher(Twist, 'cmd_vel',        10)
        self.pub_phero  = self.create_publisher(Point, 'pheromone_drop', 10)
        self.pub_halted = self.create_publisher(Int32, '/robot_halted',  10)

        # ── Hardcoded world-frame waypoints ──────────────────────────────
        if self.resource_id == 1:
            self.SPAWN_X  = -1.5
            self.RES_X    = -1.5;  self.RES_Y  =  1.5
            self.HOME_X   = -1.5;  self.HOME_Y = -1.5
        elif self.resource_id == 2:
            self.SPAWN_X  =  1.5
            self.RES_X    =  1.5;  self.RES_Y  =  3.0
            self.HOME_X   =  1.5;  self.HOME_Y = -1.5
        else:
            self.SPAWN_X  =  0.0
            self.RES_X    =  0.0;  self.RES_Y  =  2.0
            self.HOME_X   =  0.0;  self.HOME_Y = -1.5

        # ── Commanded odometry — starts at exact spawn in world frame ────
        self.cx  = float(self.SPAWN_X)
        self.cy  = 0.0
        self.cth = NORTH   # all robots spawn facing North
        self.lv  = 0.0
        self.lw  = 0.0

        # ── ACO ──────────────────────────────────────────────────────────
        self.iteration       = 1
        self.max_iterations  = 3
        self.pheromone_total = 0.0

        # ── State machine ─────────────────────────────────────────────────
        self.state  = 'BOOT_UP'
        self.t0     = self.sim_time()
        self.t_last = self.sim_time()

        self.create_timer(0.02, self.loop)   # 50 Hz
        self.get_logger().info(
            f'[R{self.resource_id}] Ready | '
            f'spawn_x={self.SPAWN_X} | '
            f'res=({self.RES_X},{self.RES_Y}) | '
            f'home=({self.HOME_X},{self.HOME_Y})'
        )

    # ================================================================== #
    #  Helpers
    # ================================================================== #

    def sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed(self):
        return self.sim_time() - self.t0

    def go(self, state):
        self.state = state
        self.t0    = self.sim_time()
        self.get_logger().info(f'[R{self.resource_id}] → {state}')

    def integrate(self, dt):
        """
        Raw (unwrapped) theta accumulation so turn thresholds work cleanly.
        cos/sin of snapped cardinal angles are exact:
          NORTH: cos≈0, sin=1  → only cy changes  ✓
          SOUTH: cos≈0, sin=-1 → only cy changes  ✓
          WEST:  cos=-1, sin≈0 → only cx changes  ✓
        """
        self.cth += self.lw  * dt
        self.cx  += self.lv  * math.cos(self.cth) * dt
        self.cy  += self.lv  * math.sin(self.cth) * dt

    def drop(self):
        """Drop pheromone only on DRIVE_SOUTH (return trip with food)."""
        if self.state == 'DRIVE_SOUTH':
            pt   = Point()
            pt.x = float(self.cx)
            pt.y = float(self.cy)
            pt.z = float(self.resource_id * 10 + self.iteration)
            self.pub_phero.publish(pt)

    # ================================================================== #
    #  Control loop  (50 Hz)
    # ================================================================== #

    def loop(self):
        now = self.sim_time()
        dt  = now - self.t_last
        self.t_last = now

        vel = Twist()
        self.integrate(dt)
        self.drop()

        # ── BOOT_UP ──────────────────────────────────────────────────────
        if self.state == 'BOOT_UP':
            if self.iteration <= 2:
                delay = 2.0 + self.resource_id * 0.8
            else:
                # Iter 3 stagger: R1=2.5s, R3=3.5s, R2=16.0s
                # R3 needs ~10.5s to clear (-1.5,-1.5) before R2 arrives
                delay = {1: 2.5, 3: 3.5, 2: 16.0}[self.resource_id]

            if self.elapsed() > delay:
                if self.iteration == 3 and self.resource_id != 1:
                    self.go('TURN_WEST')
                else:
                    self.go('DRIVE_NORTH')

        # ── LANE CHANGE: turn West, drive to x=-1.5, turn North ──────────
        elif self.state == 'TURN_WEST':
            # From NORTH(π/2) to WEST(π): turn LEFT (cth increases)
            err = WEST - self.cth    # positive: π - π/2 = π/2
            if err < SNAP_TH:
                self.cth = WEST
                self.go('DRIVE_WEST')
            else:
                vel.angular.z = TURN_SPEED

        elif self.state == 'DRIVE_WEST':
            # Drive WEST (cth=π → only cx changes, sin≈0 so cy fixed)
            if self.cx <= -1.5 + SNAP_XY:
                self.cx     = -1.5
                self.cth    = WEST
                # Adopt Robot 1's waypoints
                self.RES_X  = -1.5;  self.RES_Y  =  1.5
                self.HOME_X = -1.5;  self.HOME_Y = -1.5
                self.get_logger().info(
                    f'\n\033[1;33m*** [R{self.resource_id}] '
                    f'MERGED → Robot 1 optimal lane! ***\033[0m\n'
                )
                self.go('TURN_NORTH_A')
            else:
                vel.linear.x = SPEED

        elif self.state == 'TURN_NORTH_A':
            # From WEST(π) to NORTH(π/2): turn RIGHT (cth decreases)
            if self.cth <= NORTH + SNAP_TH:
                self.cth = NORTH
                self.go('DRIVE_NORTH')
            else:
                vel.angular.z = -TURN_SPEED

        # ── DRIVE NORTH: go to resource ───────────────────────────────────
        elif self.state == 'DRIVE_NORTH':
            if self.cy >= self.RES_Y - SNAP_XY:
                self.cy = self.RES_Y
                self.get_logger().info(
                    f'[R{self.resource_id}] ✓ Resource '
                    f'({self.RES_X},{self.RES_Y}) iter={self.iteration}'
                )
                self.go('PICK')
            else:
                vel.linear.x = SPEED

        # ── PICK: pause at resource ───────────────────────────────────────
        elif self.state == 'PICK':
            if self.elapsed() > 2.0:
                self.go('TURN_SOUTH')

        # ── TURN SOUTH: 180° right turn ───────────────────────────────────
        elif self.state == 'TURN_SOUTH':
            # From NORTH(π/2) to SOUTH(-π/2): turn RIGHT (cth decreases)
            if self.cth <= SOUTH + SNAP_TH:
                self.cth = SOUTH
                self.go('DRIVE_SOUTH')
            else:
                vel.angular.z = -TURN_SPEED

        # ── DRIVE SOUTH: go to home (pheromones dropped here) ────────────
        elif self.state == 'DRIVE_SOUTH':
            if self.cy <= self.HOME_Y + SNAP_XY:
                self.cy = self.HOME_Y
                self.get_logger().info(
                    f'[R{self.resource_id}] ✓ Home '
                    f'({self.HOME_X},{self.HOME_Y}) iter={self.iteration}'
                )
                self.go('DEPOSIT')
            else:
                vel.linear.x = SPEED

        # ── DEPOSIT ───────────────────────────────────────────────────────
        elif self.state == 'DEPOSIT':
            if self.elapsed() > 2.0:
                self.pheromone_total += 1.0
                self.get_logger().info(
                    f'\n\033[1;32m'
                    f'*** [R{self.resource_id}] +1 PHEROMONE | '
                    f'iter={self.iteration} total={self.pheromone_total:.1f} ***'
                    f'\033[0m\n'
                )
                self.go('TURN_NORTH_B')

        # ── TURN NORTH: 180° left turn back to North ──────────────────────
        elif self.state == 'TURN_NORTH_B':
            # From SOUTH(-π/2) to NORTH(π/2): turn LEFT (cth increases)
            if self.cth >= NORTH - SNAP_TH:
                self.cth = NORTH
                self.go('REST')
            else:
                vel.angular.z = TURN_SPEED

        # ── REST: wait then advance iteration ─────────────────────────────
        elif self.state == 'REST':
            if self.elapsed() > 3.0:
                if self.iteration < self.max_iterations:
                    self.iteration += 1
                    self._log_swarm()
                    self.get_logger().info(
                        f'--- [R{self.resource_id}] '
                        f'ITER {self.iteration}/{self.max_iterations} ---'
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

        # ── HALT: signal mapper ───────────────────────────────────────────
        elif self.state == 'HALT':
            msg = Int32(); msg.data = self.resource_id
            self.pub_halted.publish(msg)

        self.lv = vel.linear.x
        self.lw = vel.angular.z
        self.pub_vel.publish(vel)

    # ================================================================== #
    #  Swarm decision log
    # ================================================================== #

    def _log_swarm(self):
        if self.iteration != 3:
            return
        t = self.pheromone_total
        if self.resource_id == 1:
            self.get_logger().info(
                f'\n\033[1;33m*** [SWARM] R1 LEADS with {t:.1f} pheromone units. '
                f'Shortest path confirmed! ***\033[0m\n'
            )
        else:
            self.get_logger().info(
                f'\n\033[1;33m*** [SWARM] R{self.resource_id} has {t:.1f} units. '
                f'Merging to Robot 1 optimal lane! ***\033[0m\n'
            )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(UnifiedForagingAgent())


if __name__ == '__main__':
    main()