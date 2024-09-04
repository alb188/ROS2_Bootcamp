import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class TurtlePoseListener(Node):
    def __init__(self):
        super().__init__('turtle_pose_listener')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def pose_callback(self, msg):
        x = msg.x
        y = msg.y
        self.get_logger().info(f'Turtle pose received: x={x}, y={y}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
