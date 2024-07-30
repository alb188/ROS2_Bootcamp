import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
import random


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [float(random.randint(0,9)), float(random.randint(0,9)), float(random.randint(0,9)), float(random.randint(0,9)), float(random.randint(0,9)), float(random.randint(0,9)), float(random.randint(0,9))]
        msg.data.append(float(self.i))
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data[:-1]}: {self.i}')
        self.i += 1
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()