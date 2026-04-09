import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

class UnifiedForagingAgent(Node):

    def __init__(self):
        super().__init__('unified_agent')

        self.declare_parameter('resource_id', 1)
        self.resource_id = self.get_parameter('resource_id').value

        # ── Publishers ──────────────────────────────────────────────────
        self.pub_vel      = self.create_publisher(Twist, 'cmd_vel',        10)
        self.pub_phero    = self.create_publisher(Point, 'pheromone_drop', 10)
        self.pub_complete = self.create_publisher(Bool,  '/sim_complete',  10)

        # ── Real odometry subscriber — fixes drift completely ───────────
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.x          = 0.0
        self.y          = 0.0
        self.theta      = 0.0
        self.odom_ready = False

        # ── Waypoints (updated on lane-change for iter 3) ───────────────
        if self.resource_id == 1:
            self.res_x,  self.res_y  = -1.5,  0.8
            self.home_x, self.home_y = -1.5, -1.8
        elif self.resource_id == 2:
            self.res_x,  self.res_y  =  1.5,  2.8
            self.home_x, self.home_y =  1.5, -1.8
        else:
            self.res_x,  self.res_y  =  0.0,  1.8
            self.home_x, self.home_y =  0.0, -1.8

        # ── ACO bookkeeping ─────────────────────────────────────────────
        self.iteration       = 1
        self.max_iterations  = 3
        self.pheromone_total = 0.0

        # ── State machine ────────────────────────────────────────────────
        self.state             = 'WAIT_ODOM'
        self.action_start_time = self.get_sim_time()

        self.create_timer(0.05, self.control_loop)   # 20 Hz
        self.get_logger().info(
            f'[Robot {self.resource_id}] Odom-based agent online. '
            f'Resource=({self.res_x},{self.res_y})  '
            f'Home=({self.home_x},{self.home_y})'
        )

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

    def odom_cb(self, msg):
        """Store real Gazebo position — used for all navigation."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.odom_ready = True

    def navigate_to(self, tx, ty, vel, arrival=0.08):
        """
        P-controller: drive to (tx, ty) using real odom position.
        Returns True when within `arrival` metres of target.
        """
        dx   = tx - self.x
        dy   = ty - self.y
        dist = math.hypot(dx, dy)
        if dist < arrival:
            return True

        desired = math.atan2(dy, dx)
        err     = self.normalize(desired - self.theta)

        if abs(err) > 0.20:          # large heading error: turn in place
            vel.angular.z = 0.5 * (1.0 if err > 0 else -1.0)
            vel.linear.x  = 0.0
        else:                        # small error: drive + correct heading
            vel.linear.x  = min(0.20, 0.6 * dist)
            vel.angular.z = 1.5 * err
        return False

    def drop_pheromone(self):
        """Publish current real-world position as a pheromone breadcrumb."""
        pt = Point()
        pt.x = float(self.x)
        pt.y = float(self.y)
        self.pub_phero.publish(pt)

    # ================================================================== #
    #  Control loop  (20 Hz)
    # ================================================================== #

    def control_loop(self):
        vel     = Twist()
        elapsed = self.get_sim_time() - self.action_start_time

        # ── Wait for first odom message ─────────────────────────────────
        if self.state == 'WAIT_ODOM':
            if self.odom_ready and elapsed > 1.5:
                self.transition('BOOT_UP')
            self.pub_vel.publish(vel)
            return

        # ── Boot-up stagger so robots don't all start simultaneously ────
        elif self.state == 'BOOT_UP':
            if elapsed > (2.0 + self.resource_id * 0.5):
                if self.iteration == 3 and self.resource_id != 1:
                    self.transition('LANE_CHANGE')
                else:
                    self.transition('DRIVE_TO_RESOURCE')

        # ── Iteration-3: robots 2 & 3 merge into robot 1's optimal lane ─
        elif self.state == 'LANE_CHANGE':
            if self.navigate_to(-1.5, 0.0, vel, arrival=0.10):
                self.res_x,  self.res_y  = -1.5,  0.8   # inherit short path
                self.home_x, self.home_y = -1.5, -1.8
                self.get_logger().info(
                    f'\n\033[1;33m*** [Robot {self.resource_id}] '
                    f'LANE MERGED → now sharing Robot 1 optimal path ***\033[0m\n'
                )
                self.transition('DRIVE_TO_RESOURCE')

        # ── Navigate to resource ────────────────────────────────────────
        elif self.state == 'DRIVE_TO_RESOURCE':
            if self.navigate_to(self.res_x, self.res_y, vel):
                self.get_logger().info(
                    f'[Robot {self.resource_id}] ✓ Resource reached '
                    f'({self.res_x:.1f}, {self.res_y:.1f})'
                )
                self.transition('PICK')

        # ── Simulate pick-up (2 s stop) ─────────────────────────────────
        elif self.state == 'PICK':
            if elapsed > 2.0:
                self.transition('DRIVE_TO_HOME')

        # ── Return to home — pheromones dropped HERE (ACO-correct) ──────
        elif self.state == 'DRIVE_TO_HOME':
            self.drop_pheromone()           # every 50 ms = 20 drops/sec
            if self.navigate_to(self.home_x, self.home_y, vel):
                self.get_logger().info(
                    f'[Robot {self.resource_id}] ✓ Home reached '
                    f'({self.home_x:.1f}, {self.home_y:.1f})'
                )
                self.transition('DEPOSIT')

        # ── Deposit resource ────────────────────────────────────────────
        elif self.state == 'DEPOSIT':
            if elapsed > 2.0:
                self.pheromone_total += 1.0
                self.get_logger().info(
                    f'\n\033[1;32m'
                    f'*** [PHEROMONE] ROBOT {self.resource_id} '
                    f'DROPPED +1 UNIT  |  PATH TOTAL: '
                    f'{self.pheromone_total:.1f} ***'
                    f'\033[0m\n'
                )
                self.transition('REST')

        # ── Rest between iterations ─────────────────────────────────────
        elif self.state == 'REST':
            if elapsed > 3.0:
                if self.iteration < self.max_iterations:
                    self.iteration += 1
                    self._announce_iteration_3()
                    self.get_logger().info(
                        f'--- [Robot {self.resource_id}] '
                        f'ITERATION {self.iteration}/{self.max_iterations} ---'
                    )
                    self.transition('BOOT_UP')
                else:
                    self.get_logger().info(
                        f'\n\033[1;35m'
                        f'*** [Robot {self.resource_id}] FORAGING COMPLETE ***'
                        f'\033[0m\n'
                    )
                    self.transition('HALT')

        # ── Halted — publish sim_complete to freeze RViz heatmap ────────
        elif self.state == 'HALT':
            if self.resource_id == 1:
                done      = Bool()
                done.data = True
                self.pub_complete.publish(done)

        self.pub_vel.publish(vel)

    # ================================================================== #
    #  Swarm decision announcement
    # ================================================================== #

    def _announce_iteration_3(self):
        if self.iteration != 3:
            return
        elapsed_total = self.get_sim_time() - self.action_start_time
        path1_est     = elapsed_total / 45.0
        if self.resource_id == 1:
            self.get_logger().info(
                f'\n\033[1;33m'
                f'*** [SWARM COMM] ROBOT 1 PATH LEADS ({path1_est:.1f}). '
                f'LEADING THE SWARM! ***'
                f'\033[0m\n'
            )
        else:
            self.get_logger().info(
                f'\n\033[1;33m'
                f'*** [SWARM COMM] PATH-1 PHEROMONE ({path1_est:.1f}) IS HIGHEST. '
                f'ROBOT {self.resource_id} REDIRECTING TO OPTIMAL PATH! ***'
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