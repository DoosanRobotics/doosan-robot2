import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import yaml
class GazeboConnection(Node):

    def __init__(self):
        super().__init__('gazebo_connection')        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'gz/dsr_position_controller/commands',
            10
        )
        self.buffer = 0

    def listener_callback(self, msg):
        current_pos = [msg.position[i] for i in range(6)]
        command_msg = Float64MultiArray()
        command_msg.data = current_pos
        self.buffer += 1
        if self.buffer == 2:
            self.publisher.publish(command_msg)
            self.buffer = 0

def main(args=None):
    rclpy.init(args=args)
    gazebo_connection = GazeboConnection()
    rclpy.spin(gazebo_connection)
    gazebo_connection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()