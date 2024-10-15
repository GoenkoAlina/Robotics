import sys
from math import pi, sqrt, atan, fabs
from threading import Timer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.executors import MultiThreadedExecutor

class MoveToGoal(Node):

    def __init__(self, x, y, theta):
        super().__init__('minimal_publisher')
        self.publisher_cmd_vel = self.create_publisher(
            Twist, 
            '/turtle1/cmd_vel', 
            10)
        self.subscription_pose = self.create_subscription(
            Pose, 
            '/turtle1/pose', 
            self.position_callback, 
            10)
        self._position = Pose()
        self._x = x
        self._y = y
        self._theta = theta
        self._eps = 0.1
        Timer(1, self.start_publisher).start()

    def position_callback(self, msg):
        self._position = msg

    def find_delta(self):
        self._delta_x = self._x - self._position.x
        self._delta_y = self._y - self._position.y

    def start_publisher(self):
        rate = self.create_rate(10)

        move = Twist()
        
        self._theta = self._theta * pi / 180

        if self._theta > 0:
            while self._theta > pi:
                self._theta -= 2 * pi
        if self._theta < 0:
            while self._theta < - pi:
                self._theta += 2 * pi

        self.find_delta()

        alpha = atan(self._delta_y / self._delta_x)
        if self._delta_x < 0:
            if self._delta_y > 0:
                alpha += pi
            elif self._delta_y < 0:
                alpha += pi
        else:
            if self._delta_y < 0:
                alpha += 2 * pi
        
        alpha -= self._position.theta

        move.linear.x = alpha * 0.5
        move.angular.z = alpha

        if alpha > pi:
            alpha -= 2 * pi
        elif alpha < - pi:
            alpha += 2 * pi
    
        self.publisher_cmd_vel.publish(move)

        while fabs(self._position.theta - alpha) > self._eps:
            self.get_logger().info(f'x : {self._position.x} y : {self._position.y} theta : {self._position.theta}')


        self.find_delta()

        move.angular.z = 0.
        move.linear.x = sqrt(self._delta_x**2 + self._delta_y**2)
        self.publisher_cmd_vel.publish(move)

        while (fabs(self._position.x - self._x) > self._eps) and (fabs(self._position.y - self._y) > self._eps):
            self.get_logger().info(f'x : {self._position.x} y : {self._position.y} theta : {self._position.theta}')

        move.linear.x = 0.
        move.angular.z = self._theta - self._position.theta
        self.publisher_cmd_vel.publish(move)

        while fabs(self._position.theta - self._theta) > self._eps:
            self.get_logger().info(f'x : {self._position.x} y : {self._position.y} theta : {self._position.theta}')

        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()

    move_publisher = MoveToGoal(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))

    executor = MultiThreadedExecutor()
    executor.add_node(move_publisher)

    executor.spin()


if __name__ == '__main__':
    main()
