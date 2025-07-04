import rclpy
import os
import sys
import time
from rclpy.logging import get_logger

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m1013"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

logger = get_logger('single_robot_simple_logger')

def main(args=None):
        rclpy.init(args=args)
        node = rclpy.create_node('single_robot_simple_py', namespace=ROBOT_ID)

        DR_init.__dsr__node = node

        try:
                from DSR_ROBOT2 import (print_ext_result, movej, movejx, movesj, movesx, movel, movec, move_periodic, move_spiral, moveb, set_velx, set_accx, set_robot_mode, servoj, servoj_rt, set_safety_mode, connect_rt_control, disconnect_rt_control, 
                                        get_rt_control_output_version_list, get_rt_control_input_version_list, set_rt_control_output, set_velj_rt, set_accx_rt,
                                        get_rt_control_input_data_list, get_rt_control_output_data_list, start_rt_control, stop_rt_control, read_data_rt, write_data_rt, get_robot_mode )
                from DSR_ROBOT2 import posj, posx, posb, set_velj, set_accj
                from DSR_ROBOT2 import DR_LINE, DR_CIRCLE, DR_BASE, DR_TOOL, DR_AXIS_X, DR_AXIS_Z, DR_MV_MOD_ABS, ROBOT_MODE_AUTONOMOUS, SAFETY_MODE_MANUAL, SAFETY_MODE_EVENT_MOVE
                # print_result("Import DSR_ROBOT2 Success!")
        except ImportError as e:
                print(f"Error importing DSR_ROBOT2 : {e}")
                return
        
        r = get_robot_mode()
        print(f"Current Robot Mode: {r}")

        # set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        print(f"Connecting to RT control...")
        r = connect_rt_control()
        print(f"Connecting RT response : {r}")
        time.sleep(1.5)   

        # r = start_rt_control()
        # print(f"Start RT response : {r}")

        version_list = get_rt_control_output_version_list()
        if version_list:
                print(f"Supported RT Control Output Versions: {version_list}")
        else:
                print("Failed to get version list.")


        version_list = get_rt_control_input_version_list()
        if version_list:
                print(f"Supported RT Control Input Versions: {version_list}")
        else:
                print("Failed to get version list.")    
                r = disconnect_rt_control()

        input_versions = get_rt_control_input_version_list()
        if input_versions:
                latest_input_version = input_versions.split(',')[-1]
                print(f"Latest Input Version: {latest_input_version}")

        input_data_list = get_rt_control_input_data_list(latest_input_version)
        if input_data_list:
                print(f"Supported Input Data: {input_data_list}")

        output_versions = get_rt_control_output_version_list()
        if output_versions:
                latest_output_version = output_versions.split(',')[-1]
                print(f"Latest Output Version: {latest_output_version}")

        output_data_list = get_rt_control_output_data_list(latest_output_version)
        if output_data_list:
                print(f"Supported Output Data: {output_data_list}")

        if output_versions:
                latest_version = output_versions.split(',')[-1]
                print(f"Using latest output version: {latest_version}")

                # 2. RT 제어 출력 설정 (주기 8ms, 데이터 손실 허용 10)
                if set_rt_control_output(latest_version, 0.008, 10) == 1:
                        print("RT control output configured successfully.")

        # set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        # Example of reading data
        rt_data = read_data_rt()
        if rt_data is not None:
                print(f"Current Robot Mode_rt: {rt_data.robot_mode}")
                print(f"Actual TCP Position: {rt_data.actual_tcp_position}")
                print(f"Actual Joint Torques: {rt_data.actual_joint_torque}")

        r = start_rt_control()
        print(f"Start RT response : {r}")

        # # Set joint velocities for subsequent RT moves
        # if set_velj_rt([100.0, 100.0, 100.0, 150.0, 150.0, 150.0]) == 1:
        #         print("RT joint velocities set.")

        # # Set task space accelerations for subsequent RT moves
        # if set_accx_rt(200.0, 180.0) == 1:
        #         print("RT task accelerations set.")

        r = get_robot_mode()
        print(f"Current Robot Mode: {r}")

        # r = stop_rt_control()
        # print(f"Stop RT response : {r}")


        # servoj_rt(posj(0,0,0,0,90,0),vel=[100.0]*6, acc=[100.0]*6,time=-10000)
        # time.sleep(1.0)
        # servoj_rt(posj(0,0,0,0,0,0),vel=[100.0]*6, acc=[100.0]*6,time=-10000)
        # time.sleep(1.0)

        servoj_rt(posj(0,0,0,0,90,0),time=2.0)
        time.sleep(2.0)
        servoj_rt(posj(0,0,0,0,0,0),time=2.0)
        time.sleep(2.0)
        servoj_rt(posj(0,0,0,0,90,0),vel=[0,0,0,0,50,50], acc=[0,0,0,0,0,0],time=2.0)
        time.sleep(2.0)
        servoj_rt(posj(0,0,0,0,0,0),vel=[0,0,0,0,50,50], acc=[0,0,0,0,0,0],time=2.0)
        time.sleep(2.0)

        i = 1
        while i < 1:
                # movej(p2, vel=100, acc=100)

                # movejx(x1, vel=30, acc=60, sol=0)

                # movel(x2, velx, accx)

                # movec(c1, c2, velx, accx)

                # movesj(qlist, vel=100, acc=100)

                # movesx(xlist, vel=100, acc=100)

                # move_spiral(rev=9.5,rmax=20.0,lmax=50.0,time=20.0,axis=DR_AXIS_Z,ref=DR_TOOL)
                
                # move_periodic(amp =[10,0,0,0,30,0], period=1.0, atime=0.2, repeat=5, ref=DR_TOOL)
                
                # moveb(b_list1, vel=150, acc=250, ref=DR_BASE, mod=DR_MV_MOD_ABS)

                for i in range(120):
                        servoj_rt(posj(0,0,0,0,i,0),vel=[0,0,0,0,50,50], acc=[0,0,0,0,50,50],time=0.1)
                        time.sleep(0.05)
                time.sleep(0.5)
                # movej(p1, vel=100, acc=100)
                time.sleep(1.5)
                for i in range(120):
                        servoj_rt(posj(0,0,0,0,120-i,0),vel=[0,0,0,0,50,50], acc=[0,0,0,0,50,50],time=0.1)
                        time.sleep(0.05)
                time.sleep(1.0)                

                i += 1

        r = stop_rt_control()
        print(f"Stop RT response : {r}")

        r = disconnect_rt_control()
        print(f"Disconnecting RT response : {r}")
        time.sleep(1.5) 

        print('good bye!')
        rclpy.shutdown()

if __name__ == "__main__":
        main()

