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
        self.initial_yaw = None

        # Flags to check if initial data is captured and if the robot has stopped
        self.initial_data_captured = False
        self.has_stopped = False
        self.is_rotating = False
    
    def pose_callback(self, msg):
        # Process the odometry message here
        # Check if the initial data is already captured
        if not self.initial_data_captured:
            # Capture the initial position and yaw
            self.initial_position = msg.pose.pose.position
            self.initial_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
            self.initial_data_captured = True

            # Log the initial position and orientation
            self.get_logger().info(f"Initial Position: x={self.initial_position.x}, y={self.initial_position.y}, z={self.initial_position.z}")
            self.get_logger().info(f"Initial Yaw: {self.initial_yaw}")

        current_position = msg.pose.pose.position
        current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        diff_of_positions = abs(self.initial_position.x - current_position.x)

        # Log current position and sum for debugging
        self.get_logger().info(f"Current Position: x={current_position.x}")
        self.get_logger().info(f"Difference of Initial and Current x Position: {diff_of_positions}")
        
        if diff_of_positions > 1.0:
            if not self.has_stopped:
                # Stop the robot
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher.publish(twist)

                # Log that the robot has stopped
                self.get_logger().info("Stopping the robot as the difference of x positions exceeded 1.0.")
                self.has_stopped = True

                # Start rotating
                self.is_rotating = True

            if self.is_rotating:
                # Calculate the yaw difference
                yaw_difference = self.calculate_yaw_difference(self.initial_yaw, current_yaw)

                # Log current yaw and yaw difference
                self.get_logger().info(f"Current Yaw: {current_yaw}")
                self.get_logger().info(f"Yaw Difference: {yaw_difference}")

                if abs(yaw_difference) >= math.pi / 2:
                    # Stop rotation
                    twist = Twist()
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)

                    # Log that the robot has completed the rotation
                    self.get_logger().info("Completed 90-degree rotation.")
                    self.is_rotating = False
                else:
                    # Continue rotating
                    twist = Twist()
                    twist.angular.z = -0.5  # Negative for clockwise rotation
                    self.publisher.publish(twist)

        else:
            # Move forward
            twist = Twist()
            twist.linear.x = 0.2
            self.publisher.publish(twist)

    def get_yaw_from_quaternion(self, quaternion):
        """
        Convert quaternion to yaw angle (rotation around the z-axis).
        """
        # Extract quaternion values
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Convert to yaw
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def calculate_yaw_difference(self, initial_yaw, current_yaw):
        """
        Calculate the difference between initial and current yaw.
        """
        yaw_difference = current_yaw - initial_yaw

        # Normalize yaw_difference to be within [-pi, pi]
        yaw_difference = (yaw_difference + math.pi) % (2 * math.pi) - math.pi

        return yaw_difference

def main(args=None):
    rclpy.init(args=args)
    turtle_pregoal = TurtlePreGoal()
    rclpy.spin(turtle_pregoal)
    turtle_pregoal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
