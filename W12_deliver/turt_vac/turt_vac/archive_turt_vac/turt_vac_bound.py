import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist                                #

class TurtlePosePublisher(Node):
    def __init__(self):
        super().__init__('turtle_pose_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/turtle1/xy', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)

        # Publisher for /turtle1/cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)                   #

    def pose_callback(self, msg):
        xy_msg = Float32MultiArray()
        xy_msg.data = [msg.x, msg.y]
        self.publisher_.publish(xy_msg)
        self.get_logger().info(f'Published: x={msg.x}, y={msg.y}')

        if msg.x < 0.3 or msg.x > 10.7 or msg.y < 0.3 or msg.y > 10.7:                  #
            self.stop_turtle()                                                       #

    def stop_turtle(self):                              #
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info('Published stop command to /turtle1/cmd_vel')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
