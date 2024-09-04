import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import random
import math
import time
import tf_transformations  # For quaternion to euler conversion

class TurtleGazeboMover(Node):
    def __init__(self):
        super().__init__('turtle_gazebo_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)   # Publisher for controlling the robot's velocity
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)   # Subscriber to get the current pose of the robot
        self.movement_timer = self.create_timer(0.1, self.move_to_target) # Timer for continuous movement
        self.current_pose = None # Initialize current pose 
        self.set_new_target() # Initialize target position

        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Start time for current target
        self.distance_threshold = 0.1   # Movement threshold
        self.constant_linear_speed = 0.5  # Set desired constant linear speed
        self.max_angular_speed = 1.0  # Maximum angular speed

    def odom_callback(self, msg):
        position = msg.pose.pose.position   # Update the current pose of the robot
        orientation = msg.pose.pose.orientation # Update the current pose of the robot

        # Convert quaternion to Euler angles
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        self.current_pose = (position.x, position.y, yaw)

    def set_new_target(self):
        self.target_x = random.uniform(-3, 3)   # Set a new random target position
        self.target_y = random.uniform(-3, 3)   # Set a new random target position
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info(f'New target: ({self.target_x:.2f}, {self.target_y:.2f})')

    def normalize_angle(self, angle):
        """Normalize angle to be between -pi and pi."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def move_to_target(self):
        if self.current_pose is None:
            return  # Wait for the first pose message

        # Calculate the distance and angle to the target
        current_x, current_y, current_theta = self.current_pose
        dx = self.target_x - current_x
        dy = self.target_y - current_y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        target_angle = math.atan2(dy, dx)

        
        angular_error = self.normalize_angle(target_angle - current_theta)  # Calculate angular and linear velocity
        angular_speed = max(min(angular_error * 2.0, self.max_angular_speed), -self.max_angular_speed)  # Limit angular speed to the maximum angular speed
        linear_speed = self.constant_linear_speed if distance > self.distance_threshold else 0.0    # Set the linear speed to the constant speed

        # Create Twist message to send velocities
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed

        self.publisher.publish(cmd)

        # Check if the robot has reached the target
        if distance < self.distance_threshold:
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            elapsed_time = current_time - self.start_time

            if elapsed_time >= 10.0:
                self.get_logger().info('Target reached after 10 seconds. Waiting for 2 seconds before setting a new target.')
                self.wait_and_set_new_target(2.0)
            else:
                self.get_logger().info(f'Target reached in {elapsed_time:.2f} seconds. Waiting...')
                self.wait_and_set_new_target(10.0 - elapsed_time)

    def wait_and_set_new_target(self, wait_time):
        self.get_logger().info(f'Waiting for {wait_time:.2f} seconds before setting a new target.')
        
        # Stop the robot during the waiting period
        stop_cmd = Twist()
        self.publisher.publish(stop_cmd)

        # Sleep for the specified wait time
        time.sleep(wait_time)

        # Set a new target after waiting
        self.set_new_target()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleGazeboMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
