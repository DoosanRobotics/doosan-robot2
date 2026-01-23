#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dsr_msgs2.action import MovelH2r
import sys

class MovelH2rActionClient(Node):

    def __init__(self):
        super().__init__('movel_h2r_action_client')
        self._action_client = ActionClient(self, MovelH2r, 'dsr01/motion/movel_h2r')
        self._goal_future = None
        self._goal_handle = None

    def send_goal(self, target_pos, target_vel, target_acc):
        goal_msg = MovelH2r.Goal()
        goal_msg.target_pos = target_pos
        goal_msg.target_vel = target_vel
        goal_msg.target_acc = target_acc

        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal request...')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f'Result: {result.success}')
        except Exception as e:
            self.get_logger().warning('Goal execution failed or cancelled')
        # rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.pos}')

    def cancel_goal(self):
        if self._goal_handle is not None and self._goal_handle.accepted:
            self.get_logger().info('Canceling goal...')
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            self.get_logger().info('Goal cancel request sent.')
        else:
            self.get_logger().info('No active goal to cancel.')


def main(args=None):
    rclpy.init(args=args)

    action_client = MovelH2rActionClient()

    # Define goal parameters
    # MoveL target is usually a task space pose: [x, y, z, rx, ry, rz]
    # Units: mm and degrees
    target_pos = [400.0, 0.0, 500.0, 0.0, 180.0, 0.0] 
    
    # Velocity and Acceleration for MoveL usually have 2 components: [linear, angular] probably?
    # Or based on the action definition float64[2]
    target_vel = [30.0, 30.0] 
    target_acc = [30.0, 30.0]

    action_client.send_goal(target_pos, target_vel, target_acc)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Keyboard Interrupt (SIGINT)')
        action_client.cancel_goal()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    
if __name__ == '__main__':
    main()