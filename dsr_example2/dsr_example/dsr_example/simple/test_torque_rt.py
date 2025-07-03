import rclpy
import os
import sys
import time
import math
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
                    servoj_rt, torque_rt, read_data_rt, posj, get_robot_mode, ROBOT_MODE_MANUAL, set_robot_mode
                )
        except ImportError as e:
                print(f"Error importing DSR_ROBOT2 : {e}")
                return

        # For torque mode, it's recommended to be in manual mode.
        # This is a safety measure.
        # if get_robot_mode() != ROBOT_MODE_MANUAL:
        #     set_robot_mode(ROBOT_MODE_MANUAL)
        #     time.sleep(1) # Give time for the mode to switch

        # --- RT Control Setup ---
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
        try:
            start_rt_control()
            print("RT control started.")

            # Move to a starting position
            start_pos = posj(0, 30, 60, 0, 90, 0)
            servoj_rt(start_pos, time=3.0)
            time.sleep(3.5) # Wait for the move to complete

            print("Starting torque control loop for 10 seconds...")
            start_time = time.time()
            loop_duration = 30.0 # seconds

            while (time.time() - start_time) < loop_duration:
                loop_start_time = time.time()

                # 1. Read current robot state data
                rt_data = read_data_rt()
                if not rt_data:
                    time.sleep(control_period)
                    continue

                # 2. Get gravity compensation torque from the data
                gravity_torque = rt_data.gravity_torque

                # 3. Calculate additional control torque (e.g., oscillation)
                # Let's apply a sine wave torque to the 3rd joint
                elapsed_time = time.time() - start_time
                amplitude = 20.0  # Nm
                frequency = 0.5  # Hz
                additional_torque = [0.0] * 6
                additional_torque[2] = amplitude * math.sin(2 * math.pi * frequency * elapsed_time)

                # 4. Calculate final torque: Gravity Compensation + Control Torque
                final_torque = [g + a for g, a in zip(gravity_torque, additional_torque)]

                # 5. Send the command
                torque_rt(final_torque, time=0) # time=0 for immediate execution in the loop

                # 6. Wait for the next control cycle
                # This ensures the loop runs at the specified control_period
                loop_end_time = time.time()
                sleep_time = control_period - (loop_end_time - loop_start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            logger.error(f"An error occurred during torque control: {e}")

        finally:
            # --- Cleanup ---
            print("Stopping RT control...")
            stop_rt_control()
            print("Disconnecting from RT control...")
            disconnect_rt_control()
            time.sleep(0.5)

        print('Torque RT test finished.')
        rclpy.shutdown()

if __name__ == "__main__":
    main()