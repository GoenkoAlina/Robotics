import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from message_turtle_commands.action import MessageTurtleCommands


class ActionTurtleClient(Node):

    def __init__(self, commands):
        super().__init__('action_turtle_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'message_turtle_commands')
        self._commands = commands

        self.send_goal()


    def send_goal(self):
        if not self._commands:
            self.get_logger().info('All commands have been processed!')
            self.destroy_node()
            rclpy.shutdown()
            return
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.command = self._commands[0]['command']
        goal_msg.s = self._commands[0]['s']
        goal_msg.angle = self._commands[0]['angle']

        self.get_logger().info('Start')

        self._action_client.wait_for_server()

        self.get_logger().info('Server is free')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self.get_logger().info('Get future')

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.get_logger().info('Get result')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            del self._commands[0]
            self.send_goal()
            return
        
        self.get_logger().info('Goal accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info(f'Goal succeeded!')
        else:
            self.get_logger().info(f'Goal has status: {status}!')
        self.get_logger().info(f'Result: {result.result}')
        del self._commands[0]
        self.send_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.odom}')


def main(args=None):
    commands = [{'command' : 'forward', 's' : 2., 'angle' : 0.},
                {'command' : 'turn_right', 's' : 0., 'angle' : 90.},
                {'command' : 'forward', 's' : 1., 'angle' : 0.}]

    rclpy.init(args=args)

    action_client = ActionTurtleClient(commands)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()