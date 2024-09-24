import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import pi


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 5)

    def listener_callback(self, msg):
        move = Twist()
        if msg.data == 'turn_right':
            move.angular.z = -(pi/2)
            self.cmd_vel.publish(move)
        elif msg.data == 'turn_left':
            move.angular.z = pi/2
            self.cmd_vel.publish(move)
        elif msg.data == 'move_forward':
            move.linear.x = 1.0
            self.cmd_vel.publish(move)
        elif msg.data == 'move_backward':
            move.linear.x = -1.0
            self.cmd_vel.publish(move)
        self.get_logger().info(f'I heard: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
