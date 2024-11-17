from threading import Timer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import sin, pi

class MyMovement(Node):

    def __init__(self):
        super().__init__('my_movement')
        self.publisher_cmd_vel = self.create_publisher(
            Twist, 
            '/robot/cmd_vel', 
            10)
        self.timer = self.create_timer(0.5, self.callback)
        self.counter = 0.

    def callback(self):
        move = Twist()
       
        move.angular.z = sin(self.counter)
        move.linear.x = 5.0
        self.publisher_cmd_vel.publish(move)
        if self.counter >= 2 * 3.14:
            self.counter = 0.
        else:
            self.counter += 0.05


def main():
    rclpy.init()

    my_movement = MyMovement()

    rclpy.spin(my_movement)
    
    my_movement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
