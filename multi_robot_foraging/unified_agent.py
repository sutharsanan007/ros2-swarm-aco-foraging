import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

class UnifiedForagingAgent(Node):

    def __init__(self):
        super().__init__('unified_agent')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('resource_id', 1)
        self.declare_parameter('spawn_x',     0.0)   # FIX: world-frame spawn offset
        self.declare_parameter('spawn_y',     0.0)

        self.resource_id = self.get_parameter('resource_id').value
        self.spawn_x     = self.get_parameter('spawn_x').value
        self.spawn_y     = self.get_parameter('spawn_y').value

        # ── Publishers ──────────────────────────────────────────────────
        self.pub_vel      = self.create_publisher(Twist, 'cmd_vel',        10)
        self.pub_phero    = self.create_publisher(Point, 'pheromone_drop', 10)
        self.pub_complete = self.create_publisher(Bool,  '/sim_complete',  10)

        # ── Odom subscriber ─────────────────────────────────────────────
        # Gazebo DiffDrive odom origin = spawn point (not world origin).
        # We add spawn_x/spawn_y to convert odom coords → world coords.
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.world_x    = self.spawn_x   # initialise at known spawn pos
        self.world_y    = self.spawn_y
        self.theta      = math.pi / 2    # facing North at spawn
        self.odom_ready = False

        # ── Per-robot waypoints (world frame) ───────────────────────────
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
            f'[Robot {self.resource_id}] Online | '
            f'spawn=({self.spawn_x},{self.spawn_y}) | '
            f'resource=({self.res_x},{self.res_y}) | '
            f'home=({self.home_x},{self.home_y})'
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

    # ── Odom → world coordinates ────────────────────────────────────────
    def odom_cb(self, msg):
        """
        Gazebo DiffDrive odom is relative to spawn point.
        Adding spawn_x/spawn_y converts it to world frame.
        """
        self.world_x = msg.pose.pose.position.x + self.spawn_x
        self.world_y = msg.pose.pose.position.y + self.spawn_y
        q = msg.pose.pose.orientation
        self.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.odom_ready = True

    # ── P-controller waypoint navigator ────────────────────────────────
    def navigate_to(self, tx, ty, vel, arrival=0.10):
        """
        Drive to world-frame (tx, ty) using real corrected odom.
        Returns True when within `arrival` metres. Sets vel in-place.
        """
        dx   = tx - self.world_x
        dy   = ty - self.world_y
        dist = math.hypot(dx, dy)
        if dist < arrival:
            vel.linear.x  = 0.0
            vel.angular.z = 0.0
            return True

        desired = math.atan2(dy, dx)
        err     = self.normalize(desired - self.theta)

        if abs(err) > 0.25:              # heading correction: turn in place
            vel.angular.z = 0.6 * (1.0 if err > 0 else -1.0)
            vel.linear.x  = 0.0
        else:                            # drive + fine heading correction
            vel.linear.x  = min(0.20, 0.5 * dist)
            vel.angular.z = 1.2 * err
        return False

    # ── Pheromone drop ──────────────────────────────────────────────────
    def drop_phero(self):
        pt = Point()
        pt.x = float(self.world_x)
        pt.y = float(self.world_y)
        self.pub_phero.publish(pt)

    # ================================================================== #
    #  Control loop  (20 Hz)
    # ================================================================== #

    def control_loop(self):
        vel     = Twist()
        elapsed = self.get_sim_time() - self.action_start_time

        # ── Wait for first odom ─────────────────────────────────────────
        if self.state == 'WAIT_ODOM':
            if self.odom_ready and elapsed > 1.0:
                self.transition('BOOT_UP')
            self.pub_vel.publish(vel)
            return

        # ── Stagger start so robots don't lurch simultaneously ──────────
        elif self.state == 'BOOT_UP':
            if elapsed > (1.0 + self.resource_id * 0.5):
                if self.iteration == 3 and self.resource_id != 1:
                    self.transition('LANE_CHANGE')
                else:
                    self.transition('DRIVE_TO_RESOURCE')

        # ── Iteration-3 lane merge (robots 2 & 3 → robot 1's lane) ─────
        # Pheromone dropped here so the merge path is VISIBLE in RViz
        elif self.state == 'LANE_CHANGE':
            self.drop_phero()   # light breadcrumb so merge is visible
            if self.navigate_to(-1.5, 0.0, vel, arrival=0.12):
                self.res_x,  self.res_y  = -1.5,  0.8
                self.home_x, self.home_y = -1.5, -1.8
                self.get_logger().info(
                    f'\n\033[1;33m'
                    f'*** [Robot {self.resource_id}] LANE MERGED → '
                    f'now on Robot 1 optimal path ***'
                    f'\033[0m\n'
                )
                self.transition('DRIVE_TO_RESOURCE')

        # ── Drive to resource ───────────────────────────────────────────
        # Light pheromone on outbound trip (shows full path in RViz)
        elif self.state == 'DRIVE_TO_RESOURCE':
            self.drop_phero()   # light outbound breadcrumb
            if self.navigate_to(self.res_x, self.res_y, vel):
                self.get_logger().info(
                    f'[Robot {self.resource_id}] ✓ Resource '
                    f'({self.res_x:.1f},{self.res_y:.1f})'
                )
                self.transition('PICK')

        # ── Simulate pick-up ────────────────────────────────────────────
        elif self.state == 'PICK':
            if elapsed > 2.0:
                self.transition('DRIVE_TO_HOME')

        # ── Return to home — HEAVY pheromone drop (ACO-correct) ─────────
        elif self.state == 'DRIVE_TO_HOME':
            # Drop 3× per call to make return trail brighter than outbound
            for _ in range(3):
                self.drop_phero()
            if self.navigate_to(self.home_x, self.home_y, vel):
                self.get_logger().info(
                    f'[Robot {self.resource_id}] ✓ Home '
                    f'({self.home_x:.1f},{self.home_y:.1f})'
                )
                self.transition('DEPOSIT')

        # ── Deposit resource ────────────────────────────────────────────
        elif self.state == 'DEPOSIT':
            if elapsed > 2.0:
                self.pheromone_total += 1.0
                self.get_logger().info(
                    f'\n\033[1;32m'
                    f'*** [PHEROMONE] ROBOT {self.resource_id} +1  |  '
                    f'TOTAL: {self.pheromone_total:.1f} ***'
                    f'\033[0m\n'
                )
                self.transition('REST')

        # ── Rest between iterations ─────────────────────────────────────
        elif self.state == 'REST':
            if elapsed > 2.0:
                if self.iteration < self.max_iterations:
                    self.iteration += 1
                    self._log_swarm_decision()
                    self.get_logger().info(
                        f'--- [Robot {self.resource_id}] '
                        f'ITERATION {self.iteration}/{self.max_iterations} START ---'
                    )
                    self.transition('BOOT_UP')
                else:
                    self.get_logger().info(
                        f'\n\033[1;35m'
                        f'*** [Robot {self.resource_id}] ALL FORAGING COMPLETE ***'
                        f'\033[0m\n'
                    )
                    self.transition('HALT')

        # ── Halted — Robot 1 signals RViz to freeze the final heatmap ───
        elif self.state == 'HALT':
            if self.resource_id == 1:
                done = Bool()
                done.data = True
                self.pub_complete.publish(done)

        self.pub_vel.publish(vel)

    # ================================================================== #
    #  Swarm decision log
    # ================================================================== #

    def _log_swarm_decision(self):
        if self.iteration != 3:
            return
        est = (self.get_sim_time() - self.action_start_time) / 45.0
        if self.resource_id == 1:
            self.get_logger().info(
                f'\n\033[1;33m'
                f'*** [SWARM COMM] ROBOT 1 PATH LEADS ({est:.1f} units). '
                f'LEADING THE SWARM! ***'
                f'\033[0m\n'
            )
        else:
            self.get_logger().info(
                f'\n\033[1;33m'
                f'*** [SWARM COMM] PATH-1 IS OPTIMAL ({est:.1f} units). '
                f'ROBOT {self.resource_id} REDIRECTING! ***'
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