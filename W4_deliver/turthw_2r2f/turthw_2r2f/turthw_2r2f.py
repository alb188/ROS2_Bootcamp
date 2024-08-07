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


	def pose_callback(self, msg):
    	pi = math.pi
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
    	current_orientation = msg.pose.pose.orientation
    	diff_of_xpositions = abs(self.initial_position.x - current_position.x)
    	diff_of_ypositions = abs(self.initial_position.y - current_position.y)
    	diff_of_orientations = abs(abs(self.initial_orientation.z) - abs(current_orientation.z))

    	# Log current position and sum for debugging
    	self.get_logger().info(f"Current Position: x={current_position.x}")
    	self.get_logger().info(f"Diff of Initial and Current x Position: {diff_of_xpositions}")
    	self.get_logger().info(f"Diff of Initial and Current y Position: {diff_of_ypositions}")
    	self.get_logger().info(f"Diff of Initial and Current z Orientation: {diff_of_orientations}")

    	if diff_of_xpositions < 1.8 and diff_of_ypositions < 2.4 and diff_of_orientations < 0.6:
        	twist = Twist()
        	twist.linear.x = 0.2  # Stop the robot
        	twist.angular.z = 0.0  # Stop rotation
        	self.publisher.publish(twist)

    	elif diff_of_xpositions >= 1.8 and diff_of_orientations < 0.6:
        	twist = Twist()
        	twist.linear.x = 0.0  # Stop the robot
        	twist.angular.z = -1.0  # Stop rotation
        	self.publisher.publish(twist)

    	elif diff_of_ypositions < 2.4: #and diff_of_orientations >= 0.6:
        	twist = Twist()
        	twist.linear.x = 0.2  # Stop the robot
        	twist.angular.z = 0.0  # Stop rotation
        	self.publisher.publish(twist)

    	else:
        	# Example: Publishing a Twist message to move the robot
        	twist = Twist()
        	twist.linear.x = 0.0  # Move forward
        	twist.angular.z = 0.0  # Stop rotation
        	self.publisher.publish(twist)
        	self.get_logger().info("Stopping the robot as the sum of x positions exceeded 5.0.")

def main(args=None):
	rclpy.init(args=args)
	turtle_pregoal = TurtlePreGoal()
	rclpy.spin(turtle_pregoal)
	turtle_pre_goal.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
