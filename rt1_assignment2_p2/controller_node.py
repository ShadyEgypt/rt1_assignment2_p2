import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Publisher for /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for /odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Timer for publishing at regular intervals
        self.timer = self.create_timer(0.5, self.publish_velocity_command)
        
        # Init some variables
        self.my_vel = Twist()
        self.current_x = 0.0
        
        self.get_logger().info('Integrated Publisher and Subscriber Node Started')

    def odom_callback(self, msg):
        """Handle incoming Odometry messages to get x position."""
        self.current_x = msg.pose.pose.position.x
        self.get_logger().info(f'Received x position: {self.current_x:.2f}')

    def publish_velocity_command(self):
        """Adjust velocity commands based on the current x position."""
        if self.current_x > 7.0:
            self.my_vel.linear.x = 1.0
            self.my_vel.angular.z = 1.0
            self.get_logger().info('Turning Left: Linear=1.0, Angular=1.0')
        elif self.current_x < -7.0:
            self.my_vel.linear.x = 1.0
            self.my_vel.angular.z = -1.0
            self.get_logger().info('Turning Right: Linear=1.0, Angular=-1.0')
        else:
            self.my_vel.linear.x = 1.0
            self.my_vel.angular.z = 0.0
            self.get_logger().info('Moving Straight: Linear=1.0, Angular=0.0')
        
        self.publisher_.publish(self.my_vel)


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
