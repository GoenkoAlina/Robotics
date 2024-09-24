import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'cmd_text', 10)
        self.start_publisher()
    
    def publisher_callback(self):
        msg = String()
        msg.data = input()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

    def start_publisher(self):
        while(1):
            self.publisher_callback()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
