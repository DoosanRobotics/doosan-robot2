import os
import yaml
import signal

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node


class ConnectionNode(Node):
    def __init__(self):
        super().__init__('set_config_node')
        
        # 파라미터 선언
        self.declare_parameter('name', 'dsr01')
        self.declare_parameter('rate', 100)
        self.declare_parameter('standby', 5000)
        self.declare_parameter('command', True)
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 12345)
        self.declare_parameter('mode', 'virtual')
        self.declare_parameter('model', 'm1013')
        self.declare_parameter('gripper', 'none')
        self.declare_parameter('mobile', 'none')
        self.declare_parameter('rt_host', '192.168.137.50')

        parameters = {}
        parameters['name'] = self.get_parameter('name').get_parameter_value().string_value
        parameters['rate'] = self.get_parameter('rate').get_parameter_value().integer_value
        parameters['standby'] = self.get_parameter('standby').get_parameter_value().integer_value
        parameters['command'] = self.get_parameter('command').get_parameter_value().bool_value
        parameters['host'] = self.get_parameter('host').get_parameter_value().string_value
        parameters['port'] = self.get_parameter('port').get_parameter_value().integer_value
        parameters['mode'] = self.get_parameter('mode').get_parameter_value().string_value
        parameters['model'] = self.get_parameter('model').get_parameter_value().string_value
        parameters['gripper'] = self.get_parameter('gripper').get_parameter_value().string_value
        parameters['mobile'] = self.get_parameter('mobile').get_parameter_value().string_value
        parameters['rt_host'] = self.get_parameter('rt_host').get_parameter_value().string_value

        current_file_path = os.path.join(
            get_package_share_directory("dsr_hardware2"), "config"
        )
        os.makedirs(current_file_path, exist_ok=True)
        param_name = self.get_namespace()[1:] +'_parameters.yaml'
        with open(os.path.join(current_file_path, param_name), 'w') as file:
            yaml.dump(parameters, file)
        os.system("sync")

def main(args=None):
    rclpy.init(args=args)
    ConnectionNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
