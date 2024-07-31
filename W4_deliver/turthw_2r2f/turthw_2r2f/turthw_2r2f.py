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

        # States to manage the sequence of movements
        self.state = "move_forward_1"  # <-- Added
        self.distance_moved = 0.0  # <-- Added

    def pose_callback(self, msg):
        # Process the odometry message here
        if self.initial_position is None:
            # Capture the initial position and yaw
            self.initial_position = msg.pose.pose.position
            self.initial_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

            # Log the initial position and orientation
            self.get_logger().info(f"Initial Position: x={self.initial_position.x}, y={self.initial_position.y}, z={self.initial_position.z}")
            self.get_logger().info(f"Initial Yaw: {self.initial_yaw}")

        current_position = msg.pose.pose.position
        current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        
        if self.state == "move_forward_1":  # <-- Added
            self.distance_moved = self.calculate_distance(self.initial_position, current_position)  # <-- Added

            if self.distance_moved >= 1.0:  # <-- Added
                # Stop moving forward
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher.publish(twist)

                # Log the transition to rotation
                self.get_logger().info("Reached target distance. Transitioning to rotation.")

                # Transition to rotating
                self.state = "rotate"  # <-- Added

            else:
                # Move forward
                twist = Twist()
                twist.linear.x = 0.2
                self.publisher.publish(twist)

        elif self.state == "rotate":  # <-- Added
            yaw_difference = self.calculate_yaw_difference(self.initial_yaw, current_yaw)

            if abs(yaw_difference) >= math.pi / 2:
                # Stop rotation
                twist = Twist()
                twist.angular.z = 0.0
                self.publisher.publish(twist)

                # Log the transition to the second forward movement
                self.get_logger().info("Completed 90-degree rotation. Transitioning to second forward movement.")

                # Capture the new initial position for the second move
                self.initial_position = current_position  # <-- Added

                # Transition to moving forward again
                self.state = "move_forward_2"  # <-- Added

            else:
                # Continue rotating
                twist = Twist()
                twist.angular.z = -0.5  # Negative for clockwise rotation
                self.publisher.publish(twist)

        elif self.state == "move_forward_2":  # <-- Added
            self.distance_moved = self.calculate_distance(self.initial_position, current_position)  # <-- Added

            if self.distance_moved >= 1.0:  # <-- Added
                # Stop moving forward
                twist = Twist()
                twist.linear.x = 0.0
                self.publisher.publish(twist)

                # Log that the sequence is complete
                self.get_logger().info("Reached second target distance. Sequence complete.")  # <-- Added

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

    def calculate_distance(self, initial_position, current_position):  # <-- Added
        """
        Calculate the Euclidean distance moved from the initial position.  # <-- Added
        """
        dx = current_position.x - initial_position.x  # <-- Added
        dy = current_position.y - initial_position.y  # <-- Added
        distance = math.sqrt(dx * dx + dy * dy)  # <-- Added

        return distance  # <-- Added

def main(args=None):
    rclpy.init(args=args)
    turtle_pregoal = TurtlePreGoal()
    rclpy.spin(turtle_pregoal)
    turtle_pregoal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
