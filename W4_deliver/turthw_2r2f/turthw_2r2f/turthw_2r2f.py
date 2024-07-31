import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
import math 

class TurtlePreGoal(Node):
    def __init__(self): 
        super().__init__('turtle_pre_goal')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, 
            '/odom', 
            self.pose_callback, 
            10)

        self.initial_position = None
        self.initial_orientation = None

        # Flag to check if initial data is captured
        self.initial_data_captured = False
        # x_pose_i = msg.pose.pose.position.x
        # y_pose_i = msg.pose.pose.position.y
        # self.get_logger().info(f'x-coordinate = {x_pos_i}, y-coordinate = {y_pose_i}')
    
    def pose_callback(self, msg):
        # Process the odometry message here
        # Check if the initial data is already captured
        if not self.initial_data_captured:
            # Capture the initial position and orientation
            self.initial_position = msg.pose.pose.position
            self.initial_orientation = msg.pose.pose.orientation
            self.initial_data_captured = True

            # Log the initial position and orientation
            self.get_logger().info(f"Initial Position: x={self.initial_position.x}, y={self.initial_position.y}, z={self.initial_position.z}")
            self.get_logger().info(f"Initial Orientation: x={self.initial_orientation.x}, y={self.initial_orientation.y}, z={self.initial_orientation.z}, w={self.initial_orientation.w}")

        current_position = msg.pose.pose.position
        sum_of_positions = self.initial_position.x + current_position.x

        # Log current position and sum for debugging
        self.get_logger().info(f"Current Position: x={current_position.x}")
        self.get_logger().info(f"Sum of Initial and Current x Position: {sum_of_positions}")
        
        if sum_of_positions > 5.0:
            twist = Twist()
            twist.linear.x = 0.0  # Stop the robot
            twist.angular.z = 0.0  # Stop rotation
            self.publisher.publish(twist)

            # Log that the robot has stopped
            self.get_logger().info("Stopping the robot as the sum of x positions exceeded 5.0.")
        else:
            # Example: Publishing a Twist message to move the robot
            twist = Twist()
            twist.linear.x = 0.5  # Move forward
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtle_pregoal = TurtlePreGoal()
    rclpy.spin(turtle_pregoal)
    turtle_pre_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()

        
            
        

