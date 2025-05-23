import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import re
from controller_manager_msgs.srv import SwitchController, LoadController
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters

class MuJoCoBridge(Node):
    def __init__(self):
        super().__init__('mujoco_bridge')
        self.get_logger().info("MuJoCo Bridge Node Initialized")
        self.declare_parameter("model", "m1013")
        self.model = self.get_parameter("model").value

        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states', # 수정 필요
            self.listener_callback,
            10)
        
        self.mujoco_cmd_pub = self.create_publisher(
            Float64MultiArray,
            'mj/dsr_position_controller/commands',
            10
        )
        
        self.previous_positions = []
        self.first_callback = True
        
        self.get_logger().info(f"Publishing joint commands to: /dsr_position_controller/commands")

    def listener_callback(self, msg):
        sorted_indices = self.sort_joint_states(msg)
        
        positions = [msg.position[i] for i in sorted_indices]
        
        if self.first_callback or positions != self.previous_positions:
            cmd_msg = Float64MultiArray()
            cmd_msg.data = positions
            self.mujoco_cmd_pub.publish(cmd_msg)
            self.previous_positions = positions
            self.first_callback = False
            self.get_logger().debug(f"Published positions to MuJoCo: {positions}")

    def sort_joint_states(self, msg):
        """ JointState 메시지를 숫자 순서대로 정렬하고 NaN 값을 0.0으로 변경 """
        def extract_joint_number(joint_name):
            match = re.search(r'joint_(\d+)', joint_name)
            return int(match.group(1)) if match else float('inf')
            
        sorted_indices = sorted(range(len(msg.name)), 
                               key=lambda i: extract_joint_number(msg.name[i]))
        
        sorted_msg = JointState()
        sorted_msg.header = msg.header  # 원래의 헤더 유지
        sorted_msg.name = [msg.name[i] for i in sorted_indices]
        sorted_msg.position = [msg.position[i] for i in sorted_indices]
        sorted_msg.velocity = [msg.velocity[i] for i in sorted_indices]
        sorted_msg.effort = [0.0 if np.isnan(msg.effort[i]) else msg.effort[i] for i in sorted_indices]
        
        joint_4_index = next((i for i in sorted_indices if "joint_4" in msg.name[i]), None)


        if self.model == "p3020":
            joint_4_index = next((i for i in sorted_indices if "joint_4" in msg.name[i]), None)
            if joint_4_index is not None:
                sorted_indices = [i for i in sorted_indices if i != joint_4_index]
                
        return sorted_indices
    
        

def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()