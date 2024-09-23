# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


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
        self.move = Twist()

    def listener_callback(self, msg):
        if msg.data == 'turn_right':
            self.move.angular.z = -1.0
            self.move.linear.x = 0.0
            self.cmd_vel.publish(self.move)
        elif msg.data == 'turn_left':
            self.move.angular.z = 1.0
            self.move.linear.x = 0.0
            self.cmd_vel.publish(self.move)
        elif msg.data == 'move_forward':
            self.move.linear.x = 1.0
            self.move.angular.z = 0.0
            self.cmd_vel.publish(self.move)
        elif msg.data == 'move_backward':
            self.move.linear.x = -1.0
            self.move.angular.z = 0.0
            self.cmd_vel.publish(self.move)
        self.get_logger().info(f'I heard: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
