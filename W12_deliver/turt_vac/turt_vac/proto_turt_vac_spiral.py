# turt_vac_spiral.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleSpiral(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.a = 0.4  # Initial radius, can be zero
        self.b = 0.15  # Rate of expansion
        self.theta = 0.0  # Initial angle
        self.linear_velocity = 5.0  # Constant linear velocity

    def timer_callback(self):
        if not rclpy.ok():
            return

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

    def pose_callback(self, msg):
        if msg.x < 0.3 or msg.x > 10.7 or msg.y < 0.3 or msg.y > 10.7:
            self.get_logger().info('Pose out of bounds, shutting down...')
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    turtle_spiral = TurtleSpiral()
    while rclpy.ok():
        rclpy.spin_once(turtle_spiral, timeout_sec=0.01)
    turtle_spiral.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# turt_vac_spiral.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleSpiral(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.a = 0.4  # Initial radius, can be zero
        self.b = 0.15  # Rate of expansion
        self.theta = 0.0  # Initial angle
        self.linear_velocity = 5.0  # Constant linear velocity

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

    def pose_callback(self, msg):
        if msg.x < 0.3 or msg.x > 10.7 or msg.y < 0.3 or msg.y > 10.7:
            self.get_logger().info('Pose out of bounds, shutting down...')
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    turtle_spiral = TurtleSpiral()
    rclpy.spin(turtle_spiral)

if __name__ == '__main__':
    main()

# turtle_spiral_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleSpiral(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.a = 0.4  # Initial radius, can be zero
        self.b = 0.15  # Rate of expansion
        self.theta = 0.0  # Initial angle
        self.linear_velocity = 4.0  # Constant linear velocity

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

    def pose_callback(self, msg):
        if msg.x < 0.3 or msg.x > 10.7 or msg.y < 0.3 or msg.y > 10.7:
            self.get_logger().info('Pose out of bounds, shutting down...')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    turtle_spiral = TurtleSpiral()
    # Instead of spinning, just use a while loop
    while rclpy.ok():
        rclpy.spin_once(turtle_spiral)
    turtle_spiral.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# # turtle_spiral_node.py
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from turtlesim.msg import Pose

# class TurtleSpiral(Node):
#     def __init__(self):
#         super().__init__('cmd_vel_publisher')
#         self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
#         self.subscription = self.create_subscription(
#             Pose,
#             '/turtle1/pose',
#             self.pose_callback,
#             10)
#         self.timer_period = 0.1  # seconds
#         self.timer = self.create_timer(self.timer_period, self.timer_callback)

#         self.a = 0.4  # Initial radius, can be zero
#         self.b = 0.15  # Rate of expansion
#         self.theta = 0.0  # Initial angle
#         self.linear_velocity = 2.0  # Constant linear velocity

#     def timer_callback(self):
#         # Calculate the angular velocity needed for a constant linear velocity in an Archimedean spiral
#         v_theta = self.linear_velocity / (self.a + self.b * self.theta)

#         # Create the Twist message
#         msgt = Twist()
#         msgt.linear.x = self.linear_velocity  # Constant linear velocity
#         msgt.angular.z = v_theta

#         # Publish the message
#         self.publisher_.publish(msgt)
#         self.get_logger().info(f'Publishing: linear.x={msgt.linear.x}, angular.z={msgt.angular.z}')

#         # Update the angle
#         self.theta += v_theta * self.timer_period

#     def pose_callback(self, msg):
#         if msg.x < 0.3 or msg.x > 10.7 or msg.y < 0.3 or msg.y > 10.7:
#             rclpy.shutdown()
#             self.get_logger().info('Pose out of bounds, shutting down...')
            

# def main(args=None):
#     rclpy.init(args=args)
#     turtle_spiral = TurtleSpiral()
#     rclpy.spin(turtle_spiral)
#     turtle_spiral.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# # turtle_spiral_node.py
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from turtlesim.msg import Pose                                #

# class TurtleSpiral(Node):
#     def __init__(self):
#         super().__init__('cmd_vel_publisher')
#         self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
#         self.timer_period = 0.1  # seconds
#         self.timer = self.create_timer(self.timer_period, self.timer_callback)

#         self.a = 0.4  # Initial radius, can be zero
#         self.b = 0.15  # Rate of expansion
#         self.theta = 0.0  # Initial angle
#         self.linear_velocity = 2.0  # Constant linear velocity

#         # Service to handle termination
#         self.srv = self.create_service(Trigger, 'terminate_spiral', self.terminate_callback)            #


#     def timer_callback(self):
#         # Calculate the angular velocity needed for a constant linear velocity in an Archimedean spiral
#         v_theta = self.linear_velocity / (self.a + self.b * self.theta)

#         # Create the Twist message
#         msgt = Twist()
#         msgt.linear.x = self.linear_velocity  # Constant linear velocity
#         msgt.angular.z = v_theta

#         # Publish the message
#         self.publisher_.publish(msgt)
#         self.get_logger().info(f'Publishing: linear.x={msgt.linear.x}, angular.z={msgt.angular.z}')

#         # Update the angle
#         self.theta += v_theta * self.timer_period

#     def terminate_callback(self, request, response):                    #
#         self.get_logger().info('Termination requested')
#         response.success = True
#         response.message = 'Terminating TurtleSpiral node'
#         rclpy.shutdown()
#         return response

# def main(args=None):
#     rclpy.init(args=args)
#     turtle_spiral = TurtleSpiral()
#     rclpy.spin(turtle_spiral)
#     turtle_spiral.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()