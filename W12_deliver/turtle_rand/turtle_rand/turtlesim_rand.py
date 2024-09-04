import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute 
import random
import numpy as np

class TurtleRandom(Node):
    def __init__(self):
        super().__init__('turtle_rand__node')
        self.client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute') 

        while not self.client.wait_for_service(timeout_sec=1.0):                        
            self.get_logger().info('Service not available, waiting again...')           

        self.timer = self.create_timer(10.0, self.call_service)

    def call_service(self):                                                           
        self.request = TeleportAbsolute.Request()                                       
        self.request.x = random.uniform(1,10)                                                           
        self.request.y = random.uniform(1,10)                                                        
        self.request.theta = random.uniform(0, 2 * 3.14)                                         

        self.future = self.client.call_async(self.request)                            
        self.future.add_done_callback(self.service_response_callback)                  

    def service_response_callback(self, future):                                       
        try:                                                                             
            response = future.result()                                                           
            self.get_logger().info(f'Service call successful. Current Location: x={self.request.x:.8f}, y={self.request.y:.8f}, theta={(self.request.theta*(180/np.pi)):.8f}')                           
        except Exception as e:                                                           
            self.get_logger().error(f'Service call failed: {e}')                        

def main(args=None):
    rclpy.init(args=args)
    node = TurtleRandom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
