#!/usr/bin/env python3
import sys
import argparse
import rclpy
from rclpy.node import Node
from dsr_msgs2.msg import ServojRtStream

class ServoRtCLI(Node):
    def __init__(self, node_name='test_servoj_rt'):
        super().__init__(node_name)
        # replace 'dsr01' with your actual robot namespace if different
        topic = 'dsr01/servoj_rt_stream'
        self._pub = self.create_publisher(ServojRtStream, topic, 10)
        self.get_logger().info(f'Publishing to {topic}')

    def send_servoj_rt(self, pos, vel, acc, tm):
        msg = ServojRtStream()
        msg.pos = pos
        msg.vel = vel if vel is not None else [0.0]*6
        msg.acc = acc if acc is not None else [0.0]*6
        msg.time = tm
        self._pub.publish(msg)
        self.get_logger().info(f'→ servoj_rt pos={pos} time={tm}s')

def parse_args():
    p = argparse.ArgumentParser(
        description='Publish one ServojRtStream message to the robot.')
    p.add_argument(
        '--pos', '-p', nargs=6, type=float, required=True,
        help='6 joint positions in degrees')
    p.add_argument(
        '--vel', '-v', nargs=6, type=float,
        help='6 joint velocities in deg/s (default: auto-calc)')
    p.add_argument(
        '--acc', '-a', nargs=6, type=float,
        help='6 joint accelerations in deg/s² (default: auto-calc)')
    p.add_argument(
        '--time', '-t', type=float, default=0.1,
        help='move duration in seconds (default: 0.1)')
    return p.parse_args()

def main():
    args = parse_args()
    rclpy.init()
    node = ServoRtCLI()

    # wait briefly for subscriber to connect
    rclpy.spin_once(node, timeout_sec=0.5)

    node.send_servoj_rt(
        pos=args.pos,
        vel=args.vel,
        acc=args.acc,
        tm=args.time
    )

    # give ROS a moment to send the message
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()