import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_velocity_command)
        self.get_logger().info('Robot Controller Node Started')

    def publish_velocity_command(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing velocity command: Linear=0.5, Angular=0.0')

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
