import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'dim_state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = 0

    def listener_callback(self, msg):
        formatted_data = [f'{x:.10f}' for x in msg.data]
        self.get_logger().info(f'I heard: {formatted_data[:-1]}: {self.i}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()