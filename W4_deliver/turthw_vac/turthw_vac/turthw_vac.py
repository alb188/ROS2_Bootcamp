import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class TurtleSpiral(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10)

        self.initial_position = None
        self.previous_position = None
        
        # Flag to check if initial data is captured
        self.initial_data_captured = False
        
        # Timer setup
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Spiral parameters
        self.a = 0.1  # Initial radius
        self.b = 0.1  # Rate of expansion
        self.theta = 0.0  # Initial angle
        self.linear_velocity = 0.15  # Constant linear velocity
        
        # To track total distance traveled
        self.distance_traveled = 0.0
        
    def timer_callback(self):
        # Calculate angular velocity v_theta
        v_theta = self.linear_velocity / (self.a + self.b * self.theta)
        
        # Create and publish Twist message
        msgt = Twist()
        msgt.angular.z = v_theta
        msgt.linear.x = self.linear_velocity  # Constant linear velocity
        
        self.publisher.publish(msgt)
        self.get_logger().info(f'linear.x={msgt.linear.x}, angular.z={msgt.angular.z}, theta={self.theta}')
        
        # Update theta based on distance traveled
        self.theta = self.distance_traveled

    def pose_callback(self, msg):
        current_position = msg.pose.pose.position
        
        if not self.initial_data_captured:
            # Capture initial position
            self.initial_position = current_position
            self.previous_position = current_position
            self.initial_data_captured = True
            return
        
        # Calculate the incremental distance traveled since the last pose
        distance_x = current_position.x - self.initial_position.x
        distance_y = current_position.y - self.initial_position.y
        diff_x = current_position.x - self.previous_position.x
        diff_y = current_position.y - self.previous_position.y
        incremental_distance = math.sqrt(diff_x**2 + diff_y**2)
        distance = math.sqrt(distance_x**2 + distance_y**2)
        
        # Accumulate total distance traveled
        self.distance_traveled += incremental_distance
        
        # Update the previous position to current position
        self.previous_position = current_position

        # Check if the robot has gone out of bounds (e.g., distance exceeds 8.0)
        if distance > 1.0:
            stop_msg = Twist()
            stop_msg.angular.z = 0.0
            stop_msg.linear.x = 0.0
            self.publisher.publish(stop_msg)
            self.get_logger().info(f"Out of bounds, stopping...")
            
            # Shutdown node
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    turtle_spiral = TurtleSpiral()
    rclpy.spin(turtle_spiral)
    
if __name__ == '__main__':
    main()
