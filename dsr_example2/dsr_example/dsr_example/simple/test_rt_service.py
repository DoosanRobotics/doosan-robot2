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
    node = rclpy.create_node('rt_services_test_py', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            # RT Service Functions
            connect_rt_control, disconnect_rt_control,
            get_rt_control_output_version_list, get_rt_control_input_version_list,
            get_rt_control_input_data_list, get_rt_control_output_data_list,
            start_rt_control, stop_rt_control,
            set_rt_control_input, set_rt_control_output,
            set_velj_rt, set_accj_rt, set_velx_rt, set_accx_rt,
            read_data_rt, write_data_rt, servol_rt, posx,
            # Other necessary functions
            servoj_rt, posj, get_robot_mode, set_robot_mode, ROBOT_MODE_MANUAL
        )
    except ImportError as e:
        logger.error(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    r = get_robot_mode()
    print(f"Current Robot Mode: {r}")

    try:
        # 0. Set robot to manual mode for safety
        # if get_robot_mode() != ROBOT_MODE_MANUAL:
        #     logger.info("Setting robot to MANUAL mode for RT control.")
        #     set_robot_mode(ROBOT_MODE_MANUAL)
        #     time.sleep(1)

        # 1. Connect to RT control
        logger.info("Connecting to RT control...")
        if connect_rt_control() == 1:
            logger.info("Successfully connected to RT control.")
        else:
            logger.error("Failed to connect to RT control.")
            return
        time.sleep(0.5)

        # 2. Get version and data lists
        output_versions = get_rt_control_output_version_list()
        input_versions = get_rt_control_input_version_list()
        logger.info(f"Supported Output Versions: {output_versions}")
        logger.info(f"Supported Input Versions: {input_versions}")

        if not output_versions or not input_versions:
            logger.error("Failed to get version lists. Aborting.")
            disconnect_rt_control()
            return

        latest_output_version = output_versions.split(',')[-1]
        latest_input_version = input_versions.split(',')[-1]
        logger.info(f"Latest Output Version: {latest_output_version}")
        logger.info(f"Latest Input Version: {latest_input_version}")

        output_data_list = get_rt_control_output_data_list(latest_output_version)
        input_data_list = get_rt_control_input_data_list(latest_input_version)
        logger.info(f"Supported Output Data for {latest_output_version}: {output_data_list}")
        logger.info(f"Supported Input Data for {latest_input_version}: {input_data_list}")

        # 3. Configure RT control I/O
        logger.info("Configuring RT control output...")
        if set_rt_control_output(latest_output_version, 0.008, 10) == 1:
            logger.info("RT control output configured successfully.")
        else:
            logger.error("Failed to configure RT control output.")

        # logger.info("Configuring RT control input...")
        # if set_rt_control_input(latest_input_version, 0.008, 10) == 1:
        #     logger.info("RT control input configured successfully.")
        # else:
        #     logger.error("Failed to configure RT control input.")

        # 4. Start RT control
        logger.info("Starting RT control...")
        if start_rt_control() == 1:
            logger.info("RT control started successfully.")
        else:
            logger.error("Failed to start RT control.")
            disconnect_rt_control()
            return
        time.sleep(0.5)

        # 5. Test runtime services
        logger.info("--- Testing Runtime Services ---")

        # Read Data
        rt_data = read_data_rt()
        if rt_data:
            logger.info(f"Read Data RT successful. Robot Mode: {rt_data.robot_mode}, TCP Pos: {rt_data.actual_tcp_position}")
        else:
            logger.error("Failed to read data from RT.")

        # Write Data
        logger.info("Attempting to write data to RT...")
        if write_data_rt([0.0]*6, 0, 0, [0.0]*6, [0.0]*6) == 1:
             logger.info("Write Data RT successful.")
        else:
             logger.error("Failed to write data to RT.")

        # Set Velocities and Accelerations
        logger.info("Setting RT velocities and accelerations...")
        if set_velj_rt([5.0]*6) == 1: logger.info("Set Velj RT successful.")
        else: logger.error("Failed to set Velj RT.")

        if set_accj_rt([5.0]*6) == 1: logger.info("Set Accj RT successful.")
        else: logger.error("Failed to set Accj RT.")

        if set_velx_rt(20.0, 20.0) == 1: logger.info("Set Velx RT successful.")
        else: logger.error("Failed to set Velx RT.")

        if set_accx_rt(20.0, 20.0) == 1: logger.info("Set Accx RT successful.")
        else: logger.error("Failed to set Accx RT.")

        logger.info("--- Runtime Services Test Complete ---")
        time.sleep(1)

        # 6. Perform a simple motion test
        logger.info("Performing servoj_rt motion test...")
        servoj_rt(posj(0,0,0,0,90,0),vel=[0,0,0,0,50,0], acc=[0,0,0,0,50,0], time=2.0)
        time.sleep(2.5)
        servoj_rt(posj(0,0,0,0,0,0), vel=[0,0,0,0,50,0], acc=[0,0,0,0,50,0], time=2.0)
        time.sleep(2.5)
        servoj_rt(posj(0,0,0,0,90,0),vel=[0,0,0,0,50,0], acc=[0,0,0,0,50,0], time=2.0)
        time.sleep(2.5)
        servoj_rt(posj(0,0,0,0,0,0), vel=[0,0,0,0,50,0], acc=[0,0,0,0,50,0], time=2.0)
        time.sleep(2.5)
        # servoj_rt(posj(0,0,90,0,90,0), time=2.0)
        # time.sleep(2.5)

        # servol_rt(posx(370, 0, 420, 0, 180, 0), time=1.0)
        # time.sleep(2.0)
        # servol_rt(posx(370, 0, 450, 0, 180, 0), time=1.0)
        # time.sleep(2.0)
        # servol_rt(posx(370, 0, 420, 0, 180, 0), time=1.0)
        # time.sleep(2.0)

        logger.info("Motion test complete.")
        servoj_rt(posj(0,0,0,0,0,0), time=3.5)
        time.sleep(4.5)

    except Exception as e:
        logger.error(f"An exception occurred during the test: {e}")
    finally:
        # 7. Stop and disconnect
        logger.info("Stopping RT control...")
        if stop_rt_control() == 1:
            logger.info("RT control stopped successfully.")
        else:
            logger.error("Failed to stop RT control.")

        logger.info("Disconnecting from RT control...")
        if disconnect_rt_control() == 1:
            logger.info("Successfully disconnected from RT control.")
        else:
            logger.error("Failed to disconnect from RT control.")
        time.sleep(0.5)

        logger.info('RT services test finished.')
        rclpy.shutdown()

if __name__ == "__main__":
    main()