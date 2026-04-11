"""
unified_agent.py
================
ROOT CAUSE OF "GOING DOWN" BUG:
  Gazebo DiffDrive odom X axis = robot's SPAWN-FACING direction, NOT world East.
  All robots spawn facing North (yaw=π/2). So:
      odom +X  =  world +Y  (North, robot's forward)
      odom +Y  =  world -X  (West,  robot's left)
  
  Previous code used odom target (0, 0.8) thinking odom Y = world North.
  But odom Y = world West! So the robot drifted sideways.

FIX: Convert odom readings to WORLD coordinates using spawn_yaw rotation.
  world_x = spawn_x + odom_x * cos(π/2) - odom_y * sin(π/2)
           = spawn_x + 0 - odom_y
           = spawn_x - odom_y
  world_y = spawn_y + odom_x * sin(π/2) + odom_y * cos(π/2)
           = spawn_y + odom_x + 0
           = spawn_y + odom_x
  world_theta = spawn_yaw + odom_theta

  Then navigate_to() uses world coordinates directly — clean and unambiguous.

COLLISION FIX:
  Robot 2 and Robot 3 both targeted world(-1.5, 0.0) during lane change → crash.
  Now they target slightly offset y positions: Robot 2 → y=+0.3, Robot 3 → y=-0.3.

PHEROMONE ENCODING:
  pt.z = robot_id * 10 + iteration  (e.g. 21 = Robot 2 Iteration 1)
  Allows pheromone_mapper to render per-iteration thickness.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import math

# All robots spawn facing North (world yaw = π/2)
SPAWN_YAW = math.pi / 2.0


class UnifiedForagingAgent(Node):

    def __init__(self):
        super().__init__('unified_agent')

        self.declare_parameter('resource_id', 1)
        self.declare_parameter('spawn_x',     0.0)
        self.declare_parameter('spawn_y',     0.0)

        self.resource_id = self.get_parameter('resource_id').value
        self.spawn_x     = self.get_parameter('spawn_x').value
        self.spawn_y     = self.get_parameter('spawn_y').value

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_vel    = self.create_publisher(Twist, 'cmd_vel',       10)
        self.pub_phero  = self.create_publisher(Point, 'pheromone_drop', 10)
        self.pub_halted = self.create_publisher(Int32, '/robot_halted',  10)

        # ── Odom subscriber ──────────────────────────────────────────────
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self._odom_raw_x  = 0.0
        self._odom_raw_y  = 0.0
        self._odom_raw_th = 0.0
        self.odom_ready   = False

        # ── World-frame position (computed from odom + spawn transform) ──
        self._wx    = self.spawn_x
        self._wy    = self.spawn_y
        self._wth   = SPAWN_YAW

        # ── World-frame waypoints (clear, unambiguous coordinates) ───────
        # Resource locations in world frame:
        #   Robot 1 → (-1.5,  0.8)   Robot 2 → (1.5, 2.8)   Robot 3 → (0.0, 1.8)
        # Home location (shared loading dock in world frame):
        #   All robots → their own lane X, y=-1.8
        if self.resource_id == 1:
            self.w_res_x,  self.w_res_y  = -1.5,  0.8
            self.w_home_x, self.w_home_y = -1.5, -1.8
        elif self.resource_id == 2:
            self.w_res_x,  self.w_res_y  =  1.5,  2.8
            self.w_home_x, self.w_home_y =  1.5, -1.8
        else:
            self.w_res_x,  self.w_res_y  =  0.0,  1.8
            self.w_home_x, self.w_home_y =  0.0, -1.8

        # ── ACO bookkeeping ──────────────────────────────────────────────
        self.iteration       = 1
        self.max_iterations  = 3
        self.pheromone_total = 0.0

        # ── State machine ─────────────────────────────────────────────────
        self.state             = 'WAIT_ODOM'
        self.action_start_time = self.get_sim_time()

        self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.get_logger().info(
            f'[Robot {self.resource_id}] Online | '
            f'spawn=({self.spawn_x:.1f},{self.spawn_y:.1f}) | '
            f'resource world=({self.w_res_x},{self.w_res_y}) | '
            f'home world=({self.w_home_x},{self.w_home_y})'
        )

    # ================================================================== #
    #  Odom → World transform
    # ================================================================== #

    def odom_cb(self, msg):
        """
        Store world-frame position by rotating odom frame by spawn_yaw.

        Gazebo DiffDrive odom:
          - Origin at spawn position (0,0 at spawn, not at world origin)
          - X axis = robot's spawn-facing direction = world North (+Y)
          - Y axis = robot's left at spawn = world West (-X)

        Rotation (spawn_yaw = π/2):
          world_x = spawn_x + odom_x * cos(π/2) - odom_y * sin(π/2)
                  = spawn_x - odom_y
          world_y = spawn_y + odom_x * sin(π/2) + odom_y * cos(π/2)
                  = spawn_y + odom_x
          world_theta = SPAWN_YAW + odom_theta
        """
        ox  = msg.pose.pose.position.x
        oy  = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        oth = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                         1.0 - 2.0*(q.y*q.y + q.z*q.z))

        cy = math.cos(SPAWN_YAW)
        sy = math.sin(SPAWN_YAW)

        self._wx  = self.spawn_x + cy * ox - sy * oy
        self._wy  = self.spawn_y + sy * ox + cy * oy
        self._wth = self.normalize(SPAWN_YAW + oth)
        self.odom_ready = True

    # ================================================================== #
    #  Helpers
    # ================================================================== #

    def get_sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def transition(self, new_state):
        self.state             = new_state
        self.action_start_time = self.get_sim_time()
        self.get_logger().info(f'[Robot {self.resource_id}] → {new_state}')

    @staticmethod
    def normalize(a):
        return math.atan2(math.sin(a), math.cos(a))

    def navigate_to(self, target_wx, target_wy, vel, arrival=0.10):
        """
        P-controller navigation in WORLD coordinates.
        Returns True when within `arrival` metres of target.
        """
        dx   = target_wx - self._wx
        dy   = target_wy - self._wy
        dist = math.hypot(dx, dy)

        if dist < arrival:
            return True

        desired = math.atan2(dy, dx)
        err     = self.normalize(desired - self._wth)

        if abs(err) > 0.25:
            vel.angular.z = 0.45 * (1.0 if err > 0 else -1.0)
            vel.linear.x  = 0.0
        else:
            vel.linear.x  = min(0.20, 0.7 * dist)
            vel.angular.z = 1.2 * err
        return False

    def drop_pheromone(self):
        """
        Publish world-frame breadcrumb.
        z encodes robot_id * 10 + iteration so mapper can do per-iteration rendering.
        e.g. z=23 → Robot 2, Iteration 3
        """
        pt   = Point()
        pt.x = float(self._wx)
        pt.y = float(self._wy)
        pt.z = float(self.resource_id * 10 + self.iteration)
        self.pub_phero.publish(pt)

    # ================================================================== #
    #  Control loop  (20 Hz)
    # ================================================================== #

    def control_loop(self):
        vel     = Twist()
        elapsed = self.get_sim_time() - self.action_start_time

        # ── Wait for valid odom ─────────────────────────────────────────
        if self.state == 'WAIT_ODOM':
            if self.odom_ready and elapsed > 2.0:
                self.transition('BOOT_UP')
            self.pub_vel.publish(vel)
            return

        # ── Staggered start ─────────────────────────────────────────────
        elif self.state == 'BOOT_UP':
            if elapsed > (1.5 + self.resource_id * 0.5):
                if self.iteration == 3 and self.resource_id != 1:
                    self.transition('LANE_CHANGE')
                else:
                    self.transition('DRIVE_TO_RESOURCE')

        # ── Iteration-3 lane merge ──────────────────────────────────────
        elif self.state == 'LANE_CHANGE':
            # COLLISION FIX: stagger y arrival so robots 2 & 3 don't
            # both target identical world point (-1.5, 0.0).
            # Robot 2 → (-1.5, +0.3)  Robot 3 → (-1.5, -0.3)
            stagger_y = 0.3 * (1.0 if self.resource_id == 2 else -1.0)
            if self.navigate_to(-1.5, stagger_y, vel, arrival=0.15):
                # Inherit Robot 1's world waypoints
                self.w_res_x,  self.w_res_y  = -1.5,  0.8
                self.w_home_x, self.w_home_y = -1.5, -1.8
                self.get_logger().info(
                    f'\n\033[1;33m'
                    f'*** [Robot {self.resource_id}] LANE MERGED → '
                    f'now on Robot 1 optimal path! ***'
                    f'\033[0m\n'
                )
                self.transition('DRIVE_TO_RESOURCE')

        # ── Drive to resource ───────────────────────────────────────────
        elif self.state == 'DRIVE_TO_RESOURCE':
            if self.navigate_to(self.w_res_x, self.w_res_y, vel):
                self.get_logger().info(
                    f'[Robot {self.resource_id}] ✓ Resource world='
                    f'({self.w_res_x:.1f},{self.w_res_y:.1f})'
                )
                self.transition('PICK')

        # ── Simulate pick-up ─────────────────────────────────────────────
        elif self.state == 'PICK':
            if elapsed > 2.0:
                self.transition('DRIVE_TO_HOME')

        # ── Return to home — pheromones dropped on the RETURN trip only ──
        elif self.state == 'DRIVE_TO_HOME':
            self.drop_pheromone()
            if self.navigate_to(self.w_home_x, self.w_home_y, vel):
                self.get_logger().info(
                    f'[Robot {self.resource_id}] ✓ Home world='
                    f'({self.w_home_x:.1f},{self.w_home_y:.1f})'
                )
                self.transition('DEPOSIT')

        # ── Deposit ─────────────────────────────────────────────────────
        elif self.state == 'DEPOSIT':
            if elapsed > 2.0:
                self.pheromone_total += 1.0
                self.get_logger().info(
                    f'\n\033[1;32m'
                    f'*** [PHEROMONE] ROBOT {self.resource_id} +1 unit  |  '
                    f'ITERATION {self.iteration} TOTAL: {self.pheromone_total:.1f} ***'
                    f'\033[0m\n'
                )
                self.transition('REST')

        # ── Rest between iterations ──────────────────────────────────────
        elif self.state == 'REST':
            if elapsed > 3.0:
                if self.iteration < self.max_iterations:
                    self.iteration += 1
                    self._announce_swarm_decision()
                    self.get_logger().info(
                        f'--- [Robot {self.resource_id}] '
                        f'ITERATION {self.iteration}/{self.max_iterations} START ---'
                    )
                    self.transition('BOOT_UP')
                else:
                    self.get_logger().info(
                        f'\n\033[1;35m'
                        f'*** [Robot {self.resource_id}] ALL FORAGING COMPLETE '
                        f'| TOTAL PHEROMONES DROPPED: {self.pheromone_total:.1f} ***'
                        f'\033[0m\n'
                    )
                    self.transition('HALT')

        # ── Halt — signal the mapper ─────────────────────────────────────
        elif self.state == 'HALT':
            msg      = Int32()
            msg.data = self.resource_id
            self.pub_halted.publish(msg)

        self.pub_vel.publish(vel)

    # ================================================================== #
    #  Swarm decision log (iteration 3 entry)
    # ================================================================== #

    def _announce_swarm_decision(self):
        if self.iteration != 3:
            return
        total = self.pheromone_total   # actual accumulated units, not 0.0
        if self.resource_id == 1:
            self.get_logger().info(
                f'\n\033[1;33m'
                f'*** [SWARM COMM] ROBOT 1 PATH LEADS WITH '
                f'{total:.1f} PHEROMONE UNITS. LEADING THE SWARM! ***'
                f'\033[0m\n'
            )
        else:
            self.get_logger().info(
                f'\n\033[1;33m'
                f'*** [SWARM COMM] ROBOT {self.resource_id} has {total:.1f} units. '
                f'PATH-1 IS OPTIMAL → REDIRECTING TO ROBOT 1 LANE! ***'
                f'\033[0m\n'
            )


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedForagingAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()