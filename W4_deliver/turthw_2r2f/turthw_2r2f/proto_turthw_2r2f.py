import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf_transformations

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
		
		self.line = 1
		self.turn = 0 
		self.correct = 0

		self.i = 0


	# def correction(self, diff_of_yaw, twist):
	# 	if diff_of_yaw > 0.05 and diff_of_yaw < 0.25:
	# 		twist.angular.z = -0.125
	# 	elif diff_of_yaw < -0.05 and diff_of_yaw > -0.25:
	# 		twist.angular.z = 0.125
	# 	elif diff_of_yaw > 0.25 and diff_of_yaw < 0.5:
	# 		twist.angular.z = -0.15
	# 	elif diff_of_yaw < -0.25 and diff_of_yaw > -0.5:
	# 		twist.angular.z = 0.15
	# 	elif diff_of_yaw > 0.5:
	# 		twist.angular.z = -0.3
	# 	elif diff_of_yaw < -0.5:
	# 		twist.angular.z = 0.3
	# 	elif abs(diff_of_yaw) < 0.05:
	# 		twist.angular.z = 0.0

	def line_movement(self, current_position, yaw):
		twist = Twist()
		twist.linear.x = 0.2
		twist.angular.z = 0.0
		diff_of_xpositions = abs(self.initial_position.x - current_position.x)
		diff_of_ypositions = abs(self.initial_position.y - current_position.y)
		distance = math.sqrt((diff_of_xpositions**2) + (diff_of_ypositions**2))
		diff_of_yaw = yaw - self.initial_yaw 

		# self.correction(diff_of_yaw, twist)

		if distance > 1.0:
			twist.linear.x = 0.0
			twist.angular.z = 0.0
			self.publisher.publish(twist)
			self.line = 0
			self.turn = 1
			self.initial_position = None
			self.initial_yaw = None
			self.i += 1
		
		self.publisher.publish(twist)
		self.get_logger().info(f"dist: {distance}, angle: {diff_of_yaw}, angvel: {twist.angular.z}, init_yaw: {self.initial_yaw}, curr_yaw: {yaw}")
	


	def rotate(self, yaw):
		twist = Twist()
		twist.angular.z = -1.0
		self.publisher.publish(twist)
		diff_of_yaw = abs((self.initial_yaw) - (yaw))
		if diff_of_yaw > 270: 
			diff_of_yaw -= 270
		self.get_logger().info(f"angle: {diff_of_yaw}, init_yaw: {self.initial_yaw}, curr_yaw: {yaw}")

		if diff_of_yaw > 85:
			twist.angular.z = 0.0
			self.publisher.publish(twist)
			self.turn = 0
			self.line = 1
			self.initial_yaw = None



	def pose_callback(self, msg):
		# Process the odometry message here
		# Check if the initial data is already captured

		current_position = msg.pose.pose.position
		orientation = msg.pose.pose.orientation 

		quaternion = (
			orientation.x,
			orientation.y,
			orientation.z,
			orientation.w,
		)

		(roll, pitch, yaw) = tf_transformations.euler_from_quaternion(quaternion)
		
		while yaw > math.pi: 
			yaw -= 2*math.pi 
		while yaw < -math.pi: 
			yaw += 2*math.pi 
		
		yaw = math.degrees(yaw)
		
		
		if self.line == 1:
			if self.initial_position == None:
				self.initial_position = current_position
			if self.initial_yaw == None:
				self.initial_yaw = yaw
			
			self.line_movement(current_position, yaw)
			self.get_logger().info(f"{self.i}")

		if self.i == 2:
			self.line = 0 
			self.turn = 0 

		if self.turn == 1:
			if self.initial_yaw == None:
				self.initial_yaw = yaw

			self.rotate(yaw)



def main(args=None):
	rclpy.init(args=args)
	turtle_pregoal = TurtlePreGoal()
	rclpy.spin(turtle_pregoal)
	turtle_pre_goal.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
