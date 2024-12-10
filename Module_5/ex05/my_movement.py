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
        self.command = self.declare_parameter(
            'command', 'square').get_parameter_value().string_value
        if self.command == 'square':
            self.timer = self.create_timer(0.5, self.square_callback)
        elif self.command == 'triangle':
            self.timer = self.create_timer(0.5, self.triangle_callback)
        elif self.command == 'snake':
            self.timer = self.create_timer(0.5, self.snake_callback)
        self.counter = 0.
        self.direction = 1.
    
    def square_callback(self):
        move = Twist()
        if self.counter < 20:
            move.linear.x = 1.0
        # elif self.counter < 27:
        elif self.counter < 32:
            move.angular.z = 0.8
            move.linear.x = 0.2
        else:
            self.counter = 0
        self.publisher_cmd_vel.publish(move)
        self.counter += 1

    def triangle_callback(self):
        move = Twist()
        if self.counter < 20:
            move.linear.x = 1.0
        elif self.counter < 38:
            move.angular.z = 0.8
            move.linear.x = 0.2
        else:
            self.counter = 0
        self.publisher_cmd_vel.publish(move)
        self.counter += 1


    def snake_callback(self):
        move = Twist()
        if self.counter < 20:
            move.linear.x = 1.0
        elif self.counter < 44:
            move.angular.z = self.direction * 0.8
            move.linear.x = 0.8
        else:
            self.counter = 0
            self.direction *= -1
        self.publisher_cmd_vel.publish(move)
        self.counter += 1

def main():
    rclpy.init()

    my_movement = MyMovement()

    rclpy.spin(my_movement)
    
    my_movement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
