import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool


class TurtlePoseListener(Node):
    def __init__(self, turtle_spiral_node):
        super().__init__('turtle_pose_listener')
        self.turtle_spiral_node = turtle_spiral_node
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.publisher_terminate = self.create_publisher(Bool, '/turtle_spiral/terminate', 10)
        self.subscription  # Prevent unused variable warning

    def pose_callback(self, msg):
        x = msg.x
        y = msg.y

        if x < 0.3 or x > 10.7 or y < 0.3 or y > 10.7:
            self.get_logger().info(f'Turtle detected out of bounds: x={x}, y={y}. Terminating TurtleSpiral.')
            terminate_msg = Bool()
            terminate_msg.data = True
            self.publisher_terminate.publish(terminate_msg)
            self.turtle_spiral_node.terminate()


class TurtleSpiral(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.a = 0.4  # Initial radius, can be zero
        self.b = 0.15  # Rate of expansion
        self.theta = 0.0  # Initial angle
        self.t = 0.0  # Initial time
        self.linear_velocity = 2.0  # Constant linear velocity

        self.terminate_flag = False

        self.subscription_terminate = self.create_subscription(
            Bool,
            '/turtle_spiral/terminate',
            self.terminate_callback,
            10
        )
        self.subscription_terminate  # Prevent unused variable warning

    def terminate_callback(self, msg):
        self.terminate_flag = msg.data

    def terminate(self):
        self.timer.cancel()
        self.get_logger().info('Terminating TurtleSpiral...')
        self.destroy_node()
        rclpy.shutdown()

    def timer_callback(self):
        if self.terminate_flag:
            return

        # Calculate the angular velocity needed for a constant linear velocity in an Archimedean spiral
        v_theta = self.linear_velocity / (self.a + self.b * self.theta)

        # Create the Twist message
        msgt = Twist()
        msgt.linear.x = self.linear_velocity  # Constant linear velocity
        msgt.linear.y = 0.0
        msgt.linear.z = 0.0
        msgt.angular.x = 0.0
        msgt.angular.y = 0.0
        msgt.angular.z = v_theta

        # Publish the message
        self.publisher_.publish(msgt)
        self.get_logger().info(f'Publishing: linear.x={msgt.linear.x}, angular.z={msgt.angular.z}')

        # Update the angle
        self.theta += v_theta * self.timer_period
        self.t += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    turtle_spiral = TurtleSpiral()
    turtle_pose_listener = TurtlePoseListener(turtle_spiral)
    
    try:
        while rclpy.ok():
            rclpy.spin_once(turtle_spiral, timeout_sec=0.1)
            rclpy.spin_once(turtle_pose_listener, timeout_sec=0.1)

            # Add additional conditions to break the loop if necessary
            if turtle_spiral.terminate_flag:
                break

    except KeyboardInterrupt:
        pass

    turtle_spiral.destroy_node()
    turtle_pose_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

