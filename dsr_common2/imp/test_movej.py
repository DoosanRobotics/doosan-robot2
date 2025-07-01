#!/usr/bin/env python3
import sys
import argparse
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint

class MoveJCLI(Node):
    def __init__(self, ns='dsr01'):
        super().__init__('test_movej')
        srv_name = f'{ns}/motion/move_joint'
        self.cli = self.create_client(MoveJoint, srv_name)
        self.get_logger().info(f'Waiting for service: {srv_name}')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service not available: {srv_name}')
            rclpy.shutdown()
            sys.exit(1)

    def call_movej(self, pos, vel, acc, tm, radius, mode, blend, sync):
        req = MoveJoint.Request()
        req.pos         = pos
        req.vel         = vel
        req.acc         = acc
        req.time        = tm
        req.radius      = radius
        req.mode        = mode
        # updated field names:
        req.blend_type  = blend     # was blendType
        req.sync_type   = sync      # was syncType

        future = self.cli.call_async(req)


        self.get_logger().info(f'→ movej pos={pos} vel={vel} acc={acc} time={tm} radius={radius}')
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            res = future.result()
            self.get_logger().info(f'Result: success={res.success}, message=\"{res.message}\"')
        else:
            self.get_logger().error('Service call failed')

def parse_args():
    p = argparse.ArgumentParser(description='Call the /motion/move_joint service')
    p.add_argument('--ns',    '-n', type=str, default='dsr01',
                   help='robot namespace (default: dsr01)')
    p.add_argument('--pos',   '-p', nargs=6, type=float, required=True,
                   help='6 joint targets [deg]')
    p.add_argument('--vel',   '-v', type=float, default=20.0,
                   help='joint velocity [deg/s] (default: 20)')
    p.add_argument('--acc',   '-a', type=float, default=20.0,
                   help='joint acceleration [deg/s²] (default: 20)')
    p.add_argument('--time',  '-t', type=float, default=1.0,
                   help='reach time [s] (default: 1.0) — overrides vel/acc if >0.0)')
    p.add_argument('--radius','-r', type=float, default=0.0,
                   help='blend radius [deg] (default: 0.0)')
    p.add_argument('--mode',  '-m', type=int, default=0,
                   help='MOVE_MODE: 0=ABS, 1=REL (default: 0)')
    p.add_argument('--blend', '-b', type=int, default=0,
                   help='BLENDING_SPEED_TYPE: 0=Dup, 1=Override (default: 0)')
    p.add_argument('--sync',  '-s', type=int, default=0,
                   help='SYNC_TYPE: 0=Sync, 1=Async (default: 0)')
    return p.parse_args()

def main():
    args = parse_args()
    rclpy.init()
    node = MoveJCLI(ns=args.ns)

    node.call_movej(
        pos    = args.pos,
        vel    = args.vel,
        acc    = args.acc,
        tm     = args.time,
        radius = args.radius,
        mode   = args.mode,
        blend  = args.blend,
        sync   = args.sync,
    )

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
