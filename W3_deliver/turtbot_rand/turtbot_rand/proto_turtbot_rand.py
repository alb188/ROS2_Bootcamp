import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import math

class TurtleRandomMover(Node):
    def __init__(self):
        super().__init__('turtle_random_mover')
        
        # Publisher for controlling turtle's velocity
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber to get the current pose of the turtle
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timer for checking target every 10 seconds
        self.timer = self.create_timer(10.0, self.check_target)

        # Initialize current pose and target position
        self.current_pose = Pose()
        self.target_x = random.uniform(0, 11)
        self.target_y = random.uniform(0, 11)

        # Movement threshold
        self.distance_threshold = 0.1

    def pose_callback(self, msg):
        # Update the current pose of the turtle
        self.current_pose = msg

    def set_new_target(self):
        # Set a new random target position
        self.target_x = random.uniform(0, 11)
        self.target_y = random.uniform(0, 11)
        self.get_logger().info(f'New target: ({self.target_x:.2f}, {self.target_y:.2f})')

    def move_to_target(self):
        # Calculate the distance and angle to the target
        dx = self.target_x - self.current_pose.x
        dy = self.target_y - self.current_pose.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        target_angle = math.atan2(dy, dx)

        # Calculate angular and linear velocity
        angular_error = target_angle - self.current_pose.theta
        angular_speed = 2.0 * angular_error

        linear_speed = 1.5 * distance

        # Create Twist message to send velocities
        cmd = Twist()
        cmd.linear.x = linear_speed if distance > self.distance_threshold else 0.0
        cmd.angular.z = angular_speed if distance > self.distance_threshold else 0.0

        self.publisher.publish(cmd)

    def check_target(self):
        # Check if the turtle has reached the target
        dx = self.target_x - self.current_pose.x
        dy = self.target_y - self.current_pose.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance < self.distance_threshold:
            self.get_logger().info('Target reached.')
            self.set_new_target()

        # Move the turtle towards the target
        self.move_to_target()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleRandomMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()