import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
import math


class Turtle2Listener(Node):

    def __init__(self):
        super().__init__('turtle2_tf_listener')

        self.declare_parameter('target_frame', 'carrot1')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.on_timer)

        self.target_frame = self.get_parameter('target_frame').value

    def on_timer(self):
        from_frame = 'turtle2'
        to_frame = self.target_frame

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(from_frame, to_frame, now)
        except tf2_ros.LookupException:
            self.get_logger().info('Transform not found')
            return
        except tf2_ros.ConnectivityException:
            self.get_logger().info('Connectivity exception')
            return
        except tf2_ros.ExtrapolationException:
            self.get_logger().info('Extrapolation exception')
            return

        msg = Twist()

        msg.angular.z = 4.0 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        self.cmd_vel_pub.publish(msg)

def main():
    rclpy.init()
    node = Turtle2Listener()
    rclpy.spin(node)
    rclpy.shutdown()
