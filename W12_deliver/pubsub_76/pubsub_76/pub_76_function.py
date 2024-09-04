import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
import random


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, 'dim_state', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [random.uniform(0.0, 10.0) for _ in range(7)]
        self.publisher.publish(msg)
        formatted_data = [f'{x:.10f}' for x in msg.data]
        self.get_logger().info(f'Publishing: {formatted_data}: {self.i}')
        self.i += 1
        


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()