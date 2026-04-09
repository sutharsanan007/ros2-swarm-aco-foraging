import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import time

class ForagingController(Node):
    def __init__(self):
        super().__init__('foraging_controller')
        
        # ROS Parameters assigned by the launch file
        self.declare_parameter('resource_id', 1) 
        self.declare_parameter('home_id', 0)
        self.declare_parameter('safe_distance', 0.5)
        
        self.resource_id = self.get_parameter('resource_id').value
        self.home_id = self.get_parameter('home_id').value
        self.safe_distance = self.get_parameter('safe_distance').value
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_sub = self.create_subscription(Float32MultiArray, 'aruco_poses', self.pose_callback, 10)
        
        # Initial State
        self.current_state = 'SEARCH_RESOURCE'
        self.get_logger().info(f'Mission Started! Target: ID {self.resource_id} | Home: ID {self.home_id}')
        
        # Control Loop Timer (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop) 
        self.current_poses = []
        
        # Proportional (P) Controller Tuning
        self.kp_linear = 0.4   # Forward speed aggressiveness
        self.kp_angular = 1.5  # Turning aggressiveness
        
        # Memory to handle lost targets
        self.last_seen_time = time.time()

    def pose_callback(self, msg):
        # Update the latest pose array: [ID, X, Y, Z, ID, X, Y, Z...]
        self.current_poses = msg.data

    def get_target_pose(self, target_id):
        # Scan the array for the specific target ID
        for i in range(0, len(self.current_poses), 4):
            if int(self.current_poses[i]) == target_id:
                self.last_seen_time = time.time() # Reset the lost-target timer
                return {'x': self.current_poses[i+1], 'z': self.current_poses[i+3]}
        return None

    def control_loop(self):
        vel_msg = Twist()
        target = None
        
        # ==========================================
        # FAILSAFE: Target Lost Recovery
        # ==========================================
        if self.current_state in ['APPROACH_RESOURCE', 'APPROACH_HOME']:
            if time.time() - self.last_seen_time > 1.5: # If unseen for 1.5 seconds
                self.get_logger().warn('Target lost! Reverting to Search mode.')
                self.current_state = 'SEARCH_RESOURCE' if self.current_state == 'APPROACH_RESOURCE' else 'SEARCH_HOME'

        # ==========================================
        # STATE 1: SEARCH FOR RESOURCE
        # ==========================================
        if self.current_state == 'SEARCH_RESOURCE':
            vel_msg.angular.z = 0.3 # Spin slowly in place to scan the room
            if self.get_target_pose(self.resource_id):
                self.current_state = 'APPROACH_RESOURCE'
                self.get_logger().info('Resource found! Approaching...')

        # ==========================================
        # STATE 2: APPROACH RESOURCE (Visual Servoing)
        # ==========================================
        elif self.current_state == 'APPROACH_RESOURCE':
            target = self.get_target_pose(self.resource_id)
            if target:
                error_z = target['z'] - self.safe_distance
                error_x = target['x'] # Center of camera is 0
                
                if error_z > 0.05: # Keep driving if further than 5cm from safe line
                    # Calculate speeds and cap them to prevent jerky movements
                    vel_msg.linear.x = max(min(self.kp_linear * error_z, 0.2), -0.2)
                    vel_msg.angular.z = max(min(-self.kp_angular * error_x, 0.5), -0.5)
                else:
                    self.current_state = 'PICK'

        # ==========================================
        # STATE 3: PICK UP RESOURCE
        # ==========================================
        elif self.current_state == 'PICK':
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.cmd_pub.publish(vel_msg) # Force stop
            self.get_logger().info('Picking up resource...')
            time.sleep(2.0) # Simulate robotic arm delay
            self.current_state = 'SEARCH_HOME'

        # ==========================================
        # STATE 4: SEARCH FOR HOME
        # ==========================================
        elif self.current_state == 'SEARCH_HOME':
            vel_msg.angular.z = 0.3 # Spin to find the Home marker
            if self.get_target_pose(self.home_id):
                self.current_state = 'APPROACH_HOME'
                self.get_logger().info('Home base found! Returning...')

        # ==========================================
        # STATE 5: APPROACH HOME
        # ==========================================
        elif self.current_state == 'APPROACH_HOME':
            target = self.get_target_pose(self.home_id)
            if target:
                error_z = target['z'] - self.safe_distance
                error_x = target['x']
                
                if error_z > 0.05:
                    vel_msg.linear.x = max(min(self.kp_linear * error_z, 0.2), -0.2)
                    vel_msg.angular.z = max(min(-self.kp_angular * error_x, 0.5), -0.5)
                else:
                    self.current_state = 'DEPOSIT'

        # ==========================================
        # STATE 6: DEPOSIT & END MISSION
        # ==========================================
        elif self.current_state == 'DEPOSIT':
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.cmd_pub.publish(vel_msg)
            self.get_logger().info('MISSION COMPLETE: Resource successfully deposited at Home!')
            self.current_state = 'IDLE' # Put robot to sleep
            
        elif self.current_state == 'IDLE':
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

        # Publish the calculated velocities to the wheels
        self.cmd_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ForagingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()