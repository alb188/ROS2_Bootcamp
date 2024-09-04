# turtle_spiral_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleSpiral(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.a = 0.4  # Initial radius, can be zero
        self.b = 0.15  # Rate of expansion
        self.theta = 0.0  # Initial angle
        self.linear_velocity = 2.0  # Constant linear velocity

    def timer_callback(self):
        # Calculate the angular velocity needed for a constant linear velocity in an Archimedean spiral
        v_theta = self.linear_velocity / (self.a + self.b * self.theta)

        # Create the Twist message
        msgt = Twist()
        msgt.linear.x = self.linear_velocity  # Constant linear velocity
        msgt.angular.z = v_theta

        # Publish the message
        self.publisher_.publish(msgt)
        self.get_logger().info(f'Publishing: linear.x={msgt.linear.x}, angular.z={msgt.angular.z}')

        # Update the angle
        self.theta += v_theta * self.timer_period

def main(args=None):
    rclpy.init(args=args)
    turtle_spiral = TurtleSpiral()
    rclpy.spin(turtle_spiral)
    turtle_spiral.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
