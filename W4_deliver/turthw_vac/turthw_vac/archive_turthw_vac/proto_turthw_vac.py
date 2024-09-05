# turt_vac_spiral.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

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
		
		# Flag to check if initial data is captured
		self.initial_data_captured = False
		
		self.timer_period = 0.1  # seconds
		self.timer = self.create_timer(self.timer_period, self.timer_callback)
		
		self.a = 0.21  # Initial radius, can be zero
		self.b = 0.05  # Rate of expansion
		self.theta = 0.0  # Initial angle
		self.linear_velocity = 0.15  # Constant linear velocity
		
	def timer_callback(self):
		# Calculate the angular velocity needed for a constant linear velocity in an Archimedean spiral
		v_theta = self.linear_velocity / (self.a + self.b * self.theta)
		
		# Create the Twist message
		msgt = Twist()
		msgt.angular.z = v_theta
		msgt.linear.x = self.linear_velocity  # Constant linear velocity
		
		# Publish the message
		self.publisher.publish(msgt)
		self.get_logger().info(f'Publishing: linear.x={msgt.linear.x}, angular.z={msgt.angular.z}, theta={self.theta}')
		
		# Update the angle
		self.theta += v_theta * self.timer_period
		
	def pose_callback(self, msg):
		if not self.initial_data_captured:
			# Capture the initial position and orientation
			self.initial_position = msg.pose.pose.position
			self.initial_data_captured = True
			
		current_position = msg.pose.pose.position
		diff_of_xpositions = abs(self.initial_position.x - current_position.x)
		diff_of_ypositions = abs(self.initial_position.y - current_position.y)
		
		self.get_logger().info(f"curr_x={current_position.x}, curr_y={current_position.y}, diff_x={diff_of_xpositions}, diff_y={diff_of_ypositions}")
		
		if diff_of_xpositions < 8.0 and diff_of_ypositions < 8.0 and (diff_of_xpositions > 1.0 or diff_of_ypositions > 1.0):
			stop_msg = Twist()
			stop_msg.angular.z = 0.0
			stop_msg.linear.x = 0.0
			self.publisher.publish(stop_msg)
			self.get_logger().info(f"diff_x={diff_of_xpositions}, diff_y={diff_of_ypositions}. Pose out of bounds, stopping...")
			
			# Ensure stop command is sent before shutting down
			self.get_logger().info("Waiting for stop command to take effect...")
			#time.sleep(1.0)  # Wait a short time to ensure the command is processed
			
			# Shutdown node
			self.destroy_node()
			rclpy.shutdown()

def main(args=None):
	rclpy.init(args=args)
	turtle_spiral = TurtleSpiral()
	rclpy.spin(turtle_spiral)
	
if __name__ == '__main__':
	main()
