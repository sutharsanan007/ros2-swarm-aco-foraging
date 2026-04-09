import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header
import numpy as np

class PheromoneManager(Node):
    def __init__(self):
        super().__init__('pheromone_manager')
        
        # Grid settings: 10m x 10m area, 0.1m resolution (100x100 grid)
        self.resolution = 0.1
        self.width = 100
        self.height = 100
        self.origin_x = -5.0
        self.origin_y = -5.0
        
        # Internal float grid to handle smooth evaporation
        self.grid = np.zeros((self.height, self.width), dtype=float)
        
        # Subscriptions to all 3 robots' odometry
        self.create_subscription(Odometry, '/robot1/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/robot2/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/robot3/odom', self.odom_callback, 10)
        
        # Publisher for the OccupancyGrid (so we can visualize it in RViz!)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/pheromone_grid', 10)
        
        # Timer for evaporation and publishing (1 Hz)
        self.create_timer(1.0, self.update_grid)
        self.get_logger().info('Virtual Pheromone Grid Online.')

    def odom_callback(self, msg):
        # 1. Get real-world X, Y coordinates
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 2. Convert to grid indices
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        
        # 3. Add pheromone (Drop a "breadcrumb")
        # Ensure it is within bounds
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            self.grid[grid_y, grid_x] += 20.0 # Increase scent strength
            
            # Cap maximum scent at 100
            if self.grid[grid_y, grid_x] > 100.0:
                self.grid[grid_y, grid_x] = 100.0

    def update_grid(self):
        # 1. Evaporate pheromones slowly over time (decay by 2% every second)
        self.grid *= 0.98 
        
        # 2. Package data for ROS OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'world'
        
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        
        # Convert float grid to int8 (0-100) for standard ROS mapping
        int_grid = np.clip(self.grid, 0, 100).astype(np.int8)
        grid_msg.data = int_grid.flatten().tolist()
        
        self.grid_pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PheromoneManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()