# 
#  joint90.py
#  Author: Chemin Ahn (chemx3937@gmail.com)
#  
#  Copyright (c) 2025 Doosan Robotics
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
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
