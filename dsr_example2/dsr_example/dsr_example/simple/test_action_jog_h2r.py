#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unique_identifier_msgs.msg import UUID
from dsr_msgs2.action import JogH2r
import sys
import uuid

# The server keeps a hold-to-run goal moving only while this goal id keeps arriving on motion/h2r_keepalive. If this client stops publishing (crash, hang, network loss), the robot stops.
KEEPALIVE_HZ = 20.0

class JogH2rActionClient(Node):

    def __init__(self):
        super().__init__('jog_h2r_action_client')
        self._action_client = ActionClient(self, JogH2r, 'dsr01/motion/jog_h2r')
        self._goal_future = None
        self._goal_handle = None
        self._keepalive_pub = self.create_publisher(
            UUID, 'dsr01/motion/h2r_keepalive', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._goal_uuid = None
        self._keepalive_timer = None

    def send_goal(self, jog_axis, move_reference, velocity):
        goal_msg = JogH2r.Goal()
        goal_msg.jog_axis = jog_axis
        goal_msg.move_reference = move_reference
        goal_msg.velocity = velocity

        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal request...')

        self._goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        self._keepalive_timer = self.create_timer(1.0 / KEEPALIVE_HZ, self.publish_keepalive)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
            goal_uuid=self._goal_uuid
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.stop_keepalive()
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def publish_keepalive(self):
        self._keepalive_pub.publish(self._goal_uuid)

    def stop_keepalive(self):
        if self._keepalive_timer is not None:
            self._keepalive_timer.cancel()
            self._keepalive_timer = None

    def get_result_callback(self, future):
        self.stop_keepalive()
        try:
            result = future.result().result
            self.get_logger().info(f'Result: {result.success}')
        except Exception as e:
            self.get_logger().warning('Goal execution failed or cancelled')

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

    action_client = JogH2rActionClient()

    JOG_AXIS_JOINT_1 = 0
    MOVE_REFERENCE_BASE = 0
    VELOCITY = -10.0

    action_client.send_goal(JOG_AXIS_JOINT_1, MOVE_REFERENCE_BASE, VELOCITY)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Keyboard Interrupt (SIGINT)')
        action_client.cancel_goal()
    finally:
        action_client.stop_keepalive()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()