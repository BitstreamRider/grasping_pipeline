import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('userinput_publisher')
        self.publisher_ = self.create_publisher(String, 'user_input', 10)
        timer_period = 0.5  # seconds
        self.user_input = ""
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        self.user_input = input("Enter a message to publish: ")
        msg.data = self.user_input
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    userinput_publisher = MinimalPublisher()

    rclpy.spin(userinput_publisher)

    userinput_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()