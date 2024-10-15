from math import pi, sqrt, fabs

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from message_turtle_commands.action import MessageTurtleCommands

class ActionTurtleServer(Node):

    def __init__(self):
        super().__init__('action_turtle_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'message_turtle_commands',
            self.execute_callback
        )
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

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        command = goal_handle.request.command
        s = goal_handle.request.s
        angle = goal_handle.request.angle
        
        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0.0

        move = Twist()

        rate = self.create_rate(10)

        if command == 'forward':
            self.get_logger().info(f'Starting to move forward {s} meters!')
            move.linear.x = 1.

            remained = self._position

            while feedback_msg.odom < s:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('The goal has been canceled!')
                    goal_handle.canceled()
                    move.linear.x = 0.
                    self.publisher_cmd_vel.publish(move)
                    return MessageTurtleCommands.Result(result=False)
                
                feedback_msg.odom = sqrt((remained.x - self._position.x)**2 + (remained.y - self._position.y)**2)

                self.publisher_cmd_vel.publish(move)

                goal_handle.publish_feedback(feedback_msg)

                self.get_logger().info(f'Feedback: current odom {feedback_msg.odom} meters!')

                self.get_logger().info(f'position : {self._position.x}  {self._position.y}, {self._position.theta}')

                rate.sleep()

        elif command in ['turn_right', 'turn_left']:
            if command == 'turn_right':
                self.get_logger().info(f'Starting to turn right {angle} degrees!')
                necessary_angle = - (float(angle) * pi / 180.0)
                move.angular.z = - 0.5
            else:
                self.get_logger().info(f'Starting to turn left {angle} degrees!')
                necessary_angle = (float(angle) * pi / 180.0)
                move.angular.z = 0.5

            remained_angular = self._position.theta

            theta = 0.

            while feedback_msg.odom < fabs(necessary_angle):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('The goal has been canceled!')
                    goal_handle.canceled()
                    move.linear.x = 0.
                    self.publisher_cmd_vel.publish(move)
                    return MessageTurtleCommands.Result(result=False)
                
                theta = remained_angular - self._position.theta
                while theta > pi:
                    theta -= 2 * pi
                while theta < -pi:
                    theta += 2 * pi
                
                feedback_msg.odom = fabs(theta)

                self.publisher_cmd_vel.publish(move)

                goal_handle.publish_feedback(feedback_msg)

                self.get_logger().info(f'Feedback: current odom {feedback_msg.odom} degrees!')

                rate.sleep()
            move.angular.z = (theta)
            self.publisher_cmd_vel.publish(move)
            
        else:
            self.get_logger().info('There is no such command.!')
            goal_handle.aborted()
            result = MessageTurtleCommands.Result()
            result.result = False
            return result
        
        move.linear.x = 0.
        move.angular.z = 0.
        self.publisher_cmd_vel.publish(move)

        goal_handle.succeed()
        result = MessageTurtleCommands.Result()
        result.result = True
        self.get_logger().info('Goal succeeded.')
        return result
    
    def position_callback(self, msg):
        self._position = msg


    
def main(args=None):
    rclpy.init(args=args)

    action_server = ActionTurtleServer()

    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    executor.spin()


if __name__ == '__main__':
    main()
