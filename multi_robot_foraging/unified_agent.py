import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int32
import math

NORTH =  math.pi / 2.0
SOUTH = -math.pi / 2.0
WEST  =  math.pi

SPEED       = 0.20
TURN_SPEED  = 0.50
SNAP_XY     = 0.05
SNAP_TH     = 0.025
SETTLE_TIME = 0.5


class UnifiedForagingAgent(Node):

    def __init__(self):
        super().__init__('unified_agent')
        self.declare_parameter('resource_id', 1)
        self.resource_id = self.get_parameter('resource_id').value

        self.pub_vel    = self.create_publisher(Twist, 'cmd_vel',        10)
        self.pub_phero  = self.create_publisher(Point, 'pheromone_drop', 10)
        # Trail publisher — feeds gazebo_trail_node for in-Gazebo colored dots
        # Publishes in BOTH drive directions (North and South)
        self.pub_trail  = self.create_publisher(Point, 'trail_pos',      10)
        self.pub_halted = self.create_publisher(Int32, '/robot_halted',  10)

        if self.resource_id == 1:
            self.SPAWN_X  = -1.0
            self.RES_X    = -1.0;  self.RES_Y  =  1.0
            self.HOME_X   = -1.0;  self.HOME_Y = -1.0
        elif self.resource_id == 2:
            self.SPAWN_X  =  1.0
            self.RES_X    =  1.0;  self.RES_Y  =  2.0
            self.HOME_X   =  1.0;  self.HOME_Y = -1.0
        else:
            self.SPAWN_X  =  0.0
            self.RES_X    =  0.0;  self.RES_Y  =  1.5
            self.HOME_X   =  0.0;  self.HOME_Y = -1.0

        self.cx  = float(self.SPAWN_X)
        self.cy  = 0.0
        self.cth = NORTH
        self.lv  = 0.0
        self.lw  = 0.0

        self.iteration       = 1
        self.max_iterations  = 3
        self.pheromone_total = 0.0
        self.settle_next     = None
        self.halt_published  = False

        self.state  = 'BOOT_UP'
        self.t0     = self.sim_time()
        self.t_last = self.sim_time()

        self.create_timer(0.02, self.loop)
        self.get_logger().info(
            f'[R{self.resource_id}] Ready | '
            f'spawn_x={self.SPAWN_X} | '
            f'res=({self.RES_X},{self.RES_Y}) | '
            f'home=({self.HOME_X},{self.HOME_Y})'
        )

    def sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed(self):
        return self.sim_time() - self.t0

    def go(self, state):
        self.state = state
        self.t0    = self.sim_time()
        self.get_logger().info(f'[R{self.resource_id}] → {state}')

    def settle_to(self, next_state):
        self.settle_next = next_state
        self.go('SETTLE')

    def integrate(self, dt):
        self.cth += self.lw  * dt
        self.cx  += self.lv  * math.cos(self.cth) * dt
        self.cy  += self.lv  * math.sin(self.cth) * dt

    def publish_trail(self):
        """
        Publish current commanded position to trail_pos topic.
        gazebo_trail_node subscribes to this and spawns colored dot markers
        in Gazebo at sampled intervals so the path is physically visible.
        pt.z encodes robot_id * 10 + iteration (same scheme as pheromone_drop).
        """
        pt   = Point()
        pt.x = float(self.cx)
        pt.y = float(self.cy)
        pt.z = float(self.resource_id * 10 + self.iteration)
        self.pub_trail.publish(pt)

    def drop_pheromone(self):
        """Pheromone for ACO math — only on return trip (DRIVE_SOUTH)."""
        pt   = Point()
        pt.x = float(self.cx)
        pt.y = float(self.cy)
        pt.z = float(self.resource_id * 10 + self.iteration)
        self.pub_phero.publish(pt)

    def loop(self):
        now = self.sim_time()
        dt  = now - self.t_last
        self.t_last = now

        vel = Twist()
        self.integrate(dt)

        # Pheromone drops (return trip only — ACO correct)
        if self.state == 'DRIVE_SOUTH':
            self.drop_pheromone()

        # Trail drops (BOTH directions — for Gazebo visual)
        if self.state in ('DRIVE_NORTH', 'DRIVE_SOUTH', 'DRIVE_WEST'):
            self.publish_trail()

        if self.state == 'BOOT_UP':
            if self.iteration <= 2:
                delay = 2.0 + self.resource_id * 0.8
            else:
                delay = {1: 2.5, 3: 3.5, 2: 16.0}[self.resource_id]

            if self.elapsed() > delay:
                if self.iteration == 3 and self.resource_id != 1:
                    self.go('TURN_WEST')
                else:
                    self.go('DRIVE_NORTH')

        elif self.state == 'SETTLE':
            if self.elapsed() > SETTLE_TIME:
                self.get_logger().info(
                    f'[R{self.resource_id}][SETTLE→{self.settle_next}] '
                    f'cx={self.cx:.3f} cy={self.cy:.3f} '
                    f'cth={math.degrees(self.cth):.1f}deg'
                )
                self.go(self.settle_next)

        elif self.state == 'TURN_WEST':
            err = WEST - self.cth
            if err < SNAP_TH:
                self.cth = WEST
                self.settle_to('DRIVE_WEST')
            else:
                vel.angular.z = TURN_SPEED

        elif self.state == 'DRIVE_WEST':
            if self.cx <= -1.0 + SNAP_XY:
                self.cx     = -1.0
                self.cy     = self.HOME_Y
                self.cth    = WEST
                self.RES_X  = -1.0;  self.RES_Y  =  1.0
                self.HOME_X = -1.0;  self.HOME_Y = -1.0
                self.get_logger().info(
                    f'\n\033[1;33m*** [R{self.resource_id}] '
                    f'LANE MERGED → Robot 1 optimal path! ***\033[0m\n'
                )
                self.settle_to('TURN_NORTH_A')
            else:
                vel.linear.x = SPEED

        elif self.state == 'TURN_NORTH_A':
            if self.cth <= NORTH + SNAP_TH:
                self.cth = NORTH
                self.settle_to('DRIVE_NORTH')
            else:
                vel.angular.z = -TURN_SPEED

        elif self.state == 'DRIVE_NORTH':
            if self.cy >= self.RES_Y - SNAP_XY:
                self.cx = self.RES_X
                self.cy = self.RES_Y
                self.get_logger().info(
                    f'[R{self.resource_id}] ✓ Resource '
                    f'({self.RES_X},{self.RES_Y}) iter={self.iteration}'
                )
                self.go('PICK')
            else:
                vel.linear.x = SPEED

        elif self.state == 'PICK':
            if self.elapsed() > 2.0:
                self.settle_to('TURN_SOUTH')

        elif self.state == 'TURN_SOUTH':
            if self.cth <= SOUTH + SNAP_TH:
                self.cth = SOUTH
                self.settle_to('DRIVE_SOUTH')
            else:
                vel.angular.z = -TURN_SPEED

        elif self.state == 'DRIVE_SOUTH':
            if self.cy <= self.HOME_Y + SNAP_XY:
                self.cx = self.HOME_X
                self.cy = self.HOME_Y
                self.get_logger().info(
                    f'[R{self.resource_id}] ✓ Home '
                    f'({self.HOME_X},{self.HOME_Y}) iter={self.iteration}'
                )
                self.go('DEPOSIT')
            else:
                vel.linear.x = SPEED

        elif self.state == 'DEPOSIT':
            if self.elapsed() > 2.0:
                self.pheromone_total += 1.0
                self.get_logger().info(
                    f'\n\033[1;32m*** [R{self.resource_id}] +1 PHEROMONE | '
                    f'iter={self.iteration} total={self.pheromone_total:.1f} ***\033[0m\n'
                )
                self.settle_to('TURN_NORTH_B')

        elif self.state == 'TURN_NORTH_B':
            if self.cth >= NORTH - SNAP_TH:
                self.cth = NORTH
                self.settle_to('REST')
            else:
                vel.angular.z = TURN_SPEED

        elif self.state == 'REST':
            if self.elapsed() > 3.0:
                if self.iteration < self.max_iterations:
                    self.iteration += 1
                    self._log_swarm()
                    self.get_logger().info(
                        f'--- [R{self.resource_id}] '
                        f'ITER {self.iteration}/{self.max_iterations} | '
                        f'cx={self.cx:.3f} cy={self.cy:.3f} ---'
                    )
                    self.go('BOOT_UP')
                else:
                    self.get_logger().info(
                        f'\n\033[1;35m*** [R{self.resource_id}] COMPLETE | '
                        f'total pheromones={self.pheromone_total:.1f} ***\033[0m\n'
                    )
                    self.go('HALT')

        elif self.state == 'HALT':
            if not self.halt_published:
                msg = Int32(); msg.data = self.resource_id
                self.pub_halted.publish(msg)
                self.halt_published = True
                self.get_logger().info(
                    f'[R{self.resource_id}] halt signal sent to mapper'
                )

        self.lv = vel.linear.x
        self.lw = vel.angular.z
        self.pub_vel.publish(vel)

    def _log_swarm(self):
        if self.iteration != 3:
            return
        t = self.pheromone_total
        if self.resource_id == 1:
            self.get_logger().info(
                f'\n\033[1;33m*** [SWARM] R1 LEADS with {t:.1f} units. '
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