# turt_vac_spiral.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TurtleSpiral(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.a = 0.4  # Initial radius, can be zero
        self.b = 0.15  # Rate of expansion
        self.theta = 0.0 
        self.linear_velocity = 3.0  

    def timer_callback(self):
        v_theta = self.linear_velocity / (self.a + self.b * self.theta)

        msgt = Twist()
        msgt.linear.x = self.linear_velocity
        msgt.angular.z = v_theta

        self.publisher.publish(msgt)
        self.get_logger().info(f'Publishing: linear.x={msgt.linear.x}, angular.z={msgt.angular.z}')

        self.theta += v_theta * self.timer_period

    def pose_callback(self, msg):
        x_pose = msg.pose.pose.position.x
        y_pose = msg.pose.pose.position.y
        if x_pose < -4.0 or x_pose > 4.0 or y_pose < -4.0 or y_pose > 4.0:
            self.get_logger().info('Pose out of bounds, shutting down...')
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    turtle_spiral = TurtleSpiral()
    rclpy.spin(turtle_spiral)

if __name__ == '__main__':
    main()
