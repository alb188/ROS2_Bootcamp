# turtle_pose_listener_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

class TurtlePoseListener(Node):
    def __init__(self):
        super().__init__('turtle_pose_listener')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.publisher_terminate = self.create_publisher(Bool, '/turtle_spiral/terminate', 10)

    def pose_callback(self, msg):
        x = msg.x
        y = msg.y
        self.get_logger().info(f'Turtle pose received: x={x}, y={y}')

        if x < 0.3 or x > 10.7 or y < 0.3 or y > 10.7:
            self.get_logger().info(f'Turtle detected out of bounds: x={x}, y={y}. Terminating TurtleSpiral.')
            terminate_msg = Bool()
            terminate_msg.data = True
            self.publisher_terminate.publish(terminate_msg)
            self.turtle_spiral_node.terminate()

def main(args=None):
    rclpy.init(args=args)
    turtle_pose_listener = TurtlePoseListener()

    try:
        rclpy.spin(turtle_pose_listener)
    except KeyboardInterrupt:
        pass

    turtle_spiral.destroy_node()
    turtle_pose_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
