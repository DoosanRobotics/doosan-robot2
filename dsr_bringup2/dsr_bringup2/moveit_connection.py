# pose_subscriber.py
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene
from dsr_msgs2.srv import MoveJoint
from sensor_msgs.msg import JointState
import os, yaml, math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class Moveit2Follower(Node):
    def __init__(self):
        super().__init__('dsr_moveit2')
        self.subscription = self.create_subscription(
            PlanningScene,
            '/monitored_planning_scene',
            self.listener_callback,
            10)
        
        current_file_path = os.path.join(
            get_package_share_directory("dsr_hardware2"), "config"
        )
        os.makedirs(current_file_path, exist_ok=True)
        param_name = self.get_namespace()[1:] +'_parameters.yaml'
        with open(os.path.join(current_file_path, param_name), 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
        name_value = data.get('name')
        print(f"The 'name' parameter value is: {name_value}")
        self.subscription  # prevent unused variable warning
        self.follow_planning_clients = self.create_client(MoveJoint, name_value+'/motion/move_joint')
        
        self.subscription1 = self.create_subscription(
            JointState,
            name_value+'/joint_states',
            self.joint_state_callback,
            10
        )
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        self.previous_positions = None
        self.subscription  # prevent unused variable warning

    def round_positions(self, positions):
        # 소수점 5자리로 반올림
        return [round(pos, 5) for pos in positions]

    def joint_state_callback(self, msg):
        # 관절 위치 값을 소수점 5자리로 반올림
        current_positions = self.round_positions(msg.position)
        # print(current_positions)

        # # 이전 위치 값이 없거나 이전 값과 현재 값이 다를 경우에만 퍼블리시
        # if self.previous_positions is None or current_positions != self.previous_positions:
        #     # 새로운 메시지를 생성하여 퍼블리시
        #     new_msg = JointState()
        #     # new_msg.header = msg.header
        #     new_msg.name = msg.name
        #     new_msg.position = current_positions
        #     new_msg.velocity = msg.velocity
        #     new_msg.effort = msg.effort
        #     # print("new joint_states")

        #     self.publisher.publish(new_msg)
        #     self.previous_positions = current_positions
    
    def rad_to_deg(self, radians):
        return radians * (180 / math.pi)

    def listener_callback(self, msg):
        command = msg.robot_state.joint_state.position
        # print(command)
        req_set = [self.rad_to_deg(command[i]) for i in range(len(command))]
        print(req_set)

        request = MoveJoint.Request()
        request.pos = req_set
        # request.vel = 60.0
        # request.acc = 60.0
        request.time = 0.7
        request.sync_type = 1

        self.future = self.follow_planning_clients.call_async(request)
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    moveit_follower = Moveit2Follower()
    rclpy.spin(moveit_follower)
    moveit_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()