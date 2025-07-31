# 
#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
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
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import re

class GazeboConnection(Node):
    def __init__(self):
        super().__init__('gazebo_connection')
        self.declare_parameter("model", "m1013")  # set defual model value
        self.command_msg = Float64MultiArray()
        self.model = self.get_parameter("model").value

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(
            JointState,
            'gz/joint_states',
            10)
        
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            'gz/dsr_position_controller/commands',
            10
        )
        self.previous_command = Float64MultiArray()  # Save previous command
        self.current_pos= []

    def listener_callback(self, msg):
        sorted_msg = self.sort_joint_states(msg)
        self.publisher.publish(sorted_msg)
        if self.previous_command.data == [] or list(self.previous_command.data) != list(self.command_msg.data):
            self.command_publisher.publish(self.command_msg)
            self.previous_command.data = list(self.command_msg.data)  # Save the current command as the previous command

    def sort_joint_states(self, msg):
        """ JointState 메시지를 숫자 순서대로 정렬하고 NaN 값을 0.0으로 변경 """
        def extract_joint_number(joint_name):
            match = re.search(r'(\d+)', joint_name)  # find numbers using regular expressions
            return int(match.group(1)) if match else float('inf')  # Return the number if found, otherwise return a large value

        sorted_indices = sorted(range(len(msg.name)), key=lambda i: extract_joint_number(msg.name[i]))

        sorted_msg = JointState()
        sorted_msg.header = msg.header  # Keep original header
        sorted_msg.name = [msg.name[i] for i in sorted_indices]
        sorted_msg.position = [msg.position[i] for i in sorted_indices]
        sorted_msg.velocity = [msg.velocity[i] for i in sorted_indices]
        sorted_msg.effort = [0.0 if np.isnan(msg.effort[i]) else msg.effort[i] for i in sorted_indices]
        
        joint_4_index = next((i for i in sorted_indices if "joint_4" in msg.name[i]), None)

        if self.model == "p3020":
            filtered_indices = [i for i in sorted_indices if (i != joint_4_index)]
            self.current_pos = [msg.position[i] for i in filtered_indices]
        else:
            self.current_pos = [msg.position[i] for i in sorted_indices]
        self.command_msg.data = self.current_pos
        
        return sorted_msg
    
    
def main(args=None):
    rclpy.init(args=args)
    node = GazeboConnection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()