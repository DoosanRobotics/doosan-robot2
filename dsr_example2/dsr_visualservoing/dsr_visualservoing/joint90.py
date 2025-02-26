# 
#  joint90.py
#  Author: Chemin Ahn (chemx3937@gmail.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint

class Joint90(Node):

    def __init__(self):
        super().__init__('joint90')
        self.cli = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveJoint.Request()

    def send_request(self):
        self.req.pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self.req.vel = 100.0
        self.req.acc = 100.0
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    chem_joint_non_singularity = Joint90()
    chem_joint_non_singularity.send_request()

    while rclpy.ok():
        rclpy.spin_once(chem_joint_non_singularity)
        if chem_joint_non_singularity.future.done():
            try:
                response = chem_joint_non_singularity.future.result()
            except Exception as e:
                chem_joint_non_singularity.get_logger().info(f'Service call failed {e}')
            else:
                chem_joint_non_singularity.get_logger().info(f'Result: {response}')
            break

    chem_joint_non_singularity.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
