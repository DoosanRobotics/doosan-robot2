#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dsr_msgs2.action import MovejH2r
import sys

class MovejH2rActionClient(Node):

    def __init__(self):
        super().__init__('movej_h2r_action_client')
        self._action_client = ActionClient(self, MovejH2r, 'dsr01/motion/movej_h2r')
        self._goal_future = None
        self._goal_handle = None

    def send_goal(self, target_pos, target_vel, target_acc):
        goal_msg = MovejH2r.Goal()
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
        # rclpy.shutdown() # 메인에서 제어하도록 주석 처리

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.pos}')

    def cancel_goal(self):
        if self._goal_handle is not None and self._goal_handle.accepted:
            self.get_logger().info('Canceling goal...')
            future = self._goal_handle.cancel_goal_async()
            # 취소 요청이 완료될 때까지 동기적으로 기다리거나, 
            # 단순히 비동기 요청을 보내고 종료할 수도 있음.
            # 여기서는 비동기 요청 후 잠시 대기
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            self.get_logger().info('Goal cancel request sent.')
        else:
            self.get_logger().info('No active goal to cancel.')


def main(args=None):
    rclpy.init(args=args)

    action_client = MovejH2rActionClient()

    target_pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0] 
    target_vel = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0] 
    target_acc = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]

    action_client.send_goal(target_pos, target_vel, target_acc)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Keyboard Interrupt (SIGINT)')
        # [수정됨] 취소 로직 호출
        action_client.cancel_goal()
    finally:
        # 종료 처리
        if rclpy.ok():
            rclpy.shutdown()
    
if __name__ == '__main__':
    main()