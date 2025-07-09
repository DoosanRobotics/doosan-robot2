import os
import yaml
import signal

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

class VirtualDRCF(Node):
    def __init__(self):
        super().__init__('virtual_node')
        
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

        

        mode, port, model, name = parameters['mode'], parameters['port'], parameters['model'], parameters['name']
        self.emulator_name = "emulator"
        if name:
            self.emulator_name = name + "_" + "emulator"
        if mode == "virtual":
            self.run_drcf(port, model, name)

        # signal.signal(signal.SIGINT, self.terminate_drcf)
        # signal.signal(signal.SIGTERM, self.terminate_drcf)
    
    ## TODO(Improve) : The context seems to can't handle signal perfectly.
    # needed to fix after https://github.com/ros2/rclpy/issues/1287
    def __del__(self):
        self.terminate_drcf()

    def run_drcf(self, port, model, name):
        run_script_path = os.path.join(
            get_package_share_directory("dsr_common2"), "bin"
        )
        start_cmd = "{}/run_drcf.sh ".format(run_script_path) +" "+ str(port)+" "+ model +" " +name
        os.system(start_cmd)

    def terminate_drcf(self):
        stop_cmd = "docker ps -a --filter name={} -q | xargs -r docker rm -f".\
            format(self.emulator_name)
        print("stop_cmd : ",stop_cmd)
        os.system(stop_cmd)
        rclpy.shutdown()
        exit(1)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualDRCF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
