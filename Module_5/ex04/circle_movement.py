from threading import Timer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMovement(Node):

    def __init__(self):
        super().__init__('circle_movement')
        self.publisher_cmd_vel = self.create_publisher(
            Twist, 
            '/robot/cmd_vel', 
            10)
        self.timer = self.create_timer(0.5, self.callback)

    def callback(self):
        move = Twist()
       
        move.angular.z = 0.5
        move.linear.x = 5.
        self.publisher_cmd_vel.publish(move)


def main():
    rclpy.init()

    circle_movement = CircleMovement()

    rclpy.spin(circle_movement)
    
    circle_movement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
