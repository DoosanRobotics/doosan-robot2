import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import re

class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(
            JointState,
            'gz/joint_states',
            10)

    def listener_callback(self, msg):
        sorted_msg = self.sort_joint_states(msg)
        self.publisher.publish(sorted_msg)

    def sort_joint_states(self, msg):
        """ JointState 메시지를 숫자 순서대로 정렬하고 NaN 값을 0.0으로 변경 """
        

        def extract_joint_number(joint_name):
            match = re.search(r'(\d+)', joint_name)  # 정규표현식으로 숫자 찾기
            return int(match.group(1)) if match else float('inf')  # 숫자 반환, 없으면 큰 값
        

        sorted_indices = sorted(range(len(msg.name)), key=lambda i: extract_joint_number(msg.name[i]))

        sorted_msg = JointState()
        sorted_msg.header = msg.header  # 원래의 헤더 유지
        sorted_msg.name = [msg.name[i] for i in sorted_indices]
        sorted_msg.position = [msg.position[i] for i in sorted_indices]
        sorted_msg.velocity = [msg.velocity[i] for i in sorted_indices]
        sorted_msg.effort = [0.0 if np.isnan(msg.effort[i]) else msg.effort[i] for i in sorted_indices]
        
        return sorted_msg
    
    
def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()