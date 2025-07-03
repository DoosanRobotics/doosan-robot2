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
        from DSR_ROBOT2 import (
            connect_rt_control, disconnect_rt_control, get_rt_control_output_version_list,
            set_rt_control_output, start_rt_control, stop_rt_control,
            servoj_rt, speedj_rt, posj, get_robot_mode, ROBOT_MODE_MANUAL, set_robot_mode
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    # # It's recommended to be in manual mode for RT control for safety.
    # if get_robot_mode() != ROBOT_MODE_MANUAL:
    #     set_robot_mode(ROBOT_MODE_MANUAL)
    #     time.sleep(1)

    # --- RT Control Setup ---
    try:
        print("Connecting to RT control...")
        connect_rt_control()
        time.sleep(0.5)

        output_versions = get_rt_control_output_version_list()
        if not output_versions:
            logger.error("Failed to get RT control output version list.")
            disconnect_rt_control()
            return

        latest_version = output_versions.split(',')[-1]
        print(f"Using latest output version: {latest_version}")

        # Set RT control output: 8ms period, 10 lost packets allowed
        control_period = 0.008
        if set_rt_control_output(latest_version, control_period, 10) != 1:
            logger.error("Failed to set RT control output.")
            disconnect_rt_control()
            return
        print("RT control output configured successfully.")

        # --- Main Control Logic ---
        start_rt_control()
        print("RT control started.")

        # Move to a starting position using servoj_rt
        start_pos = posj(0, 0, 0, 0, 0, 0)
        servoj_rt(start_pos, time=3.0)
        time.sleep(3.5) # Wait for the move to complete

        print("Starting speedj_rt test loop...")
        
        # Loop to gradually increase speed on the 5th joint
        print("Increasing speed on J5...")
        for i in range(21): # 0 to 20
            vel = [0.0, 0.0, 0.0, 0.0, i * 2.0, 0.0]  # Target velocity up to 40 deg/s
            # acc = [30.0] * 6
            acc = [0.0, 0.0, 0.0, 0.0, 50.0, 0.0]

            speedj_rt(vel, acc, time=0.1)
            time.sleep(0.1)
        
        time.sleep(1.0)

        # Loop to gradually decrease speed on the 5th joint (negative velocity)
        print("Decreasing speed on J5...")
        for i in range(21): # 0 to 20
            vel = [0.0, 0.0, 0.0, 0.0, (20-i) * -2.0, 0.0] # Target velocity from -40 to 0 deg/s
            # acc = [30.0] * 6
            acc = [0.0, 0.0, 0.0, 0.0, 50.0, 0.0]            
            speedj_rt(vel, acc, time=0.1)
            time.sleep(0.1)

        # Ensure the robot is stopped
        print("Sending final stop command.")
        speedj_rt([0.0]*6, [50.0]*6, time=0.5)
        time.sleep(1.0)

    except Exception as e:
        logger.error(f"An error occurred during the test: {e}")

    finally:
        # --- Cleanup ---
        print("Stopping RT control...")
        stop_rt_control()
        print("Disconnecting from RT control...")
        disconnect_rt_control()
        time.sleep(0.5)

    print('Speedj_rt test finished.')
    rclpy.shutdown()

if __name__ == "__main__":
    main()