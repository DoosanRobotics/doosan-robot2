
import rclpy
import os
import sys
import time
import math
import threading
from rclpy.logging import get_logger

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m1013"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

logger = get_logger('single_robot_simple_logger')

# This function will run in a separate thread to alter the motion
def alter_motion_thread_func(stop_event):
    logger.info("Alter motion thread started.")
    from DSR_ROBOT2 import alter_motion_stream, posx
    
    # Wait a moment for the main move to start
    time.sleep(1.0) 
    
    if not stop_event.is_set():
        logger.info("Altering motion UP...")
        for i in range(40):
            if stop_event.is_set():
                logger.info("Alter motion thread stopped by event.")
                return
            # Gradually increase the position in the Z direction
            alter_motion_stream(posx(0, 0, i*0.2, 0, 0, 0))
            time.sleep(0.005)
        for i in range(40, 0, -1):
            if stop_event.is_set():
                logger.info("Alter motion thread stopped by event.")
                return
            # Gradually decrease the position in the Z direction
            alter_motion_stream(posx(0, 0, i*0.2, 0, 0, 0))
            time.sleep(0.005)
        for i in range(40):
            if stop_event.is_set():
                logger.info("Alter motion thread stopped by event.")
                return
            # Gradually increase the position in the Z direction
            alter_motion_stream(posx(0, 0, -i*0.2, 0, 0, 0))
            time.sleep(0.005)
        for i in range(40, 0, -1):
            if stop_event.is_set():
                logger.info("Alter motion thread stopped by event.")
                return
            # Gradually decrease the position in the Z direction
            alter_motion_stream(posx(0, 0, -i*0.2, 0, 0, 0))
            time.sleep(0.005)
        
    time.sleep(1.0) 
    
    logger.info("Alter motion thread finished its commands.")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('single_robot_simple_py', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej, movel, enable_alter_motion, disable_alter_motion, check_motion,
            set_velx, set_accx, amovel, posj, posx, DR_DPOS, DR_WORLD, DR_STATE_IDLE
        )
    except ImportError as e:
        logger.error(f"Error importing DSR_ROBOT2 : {e}")
        return

    # Initial setup
    set_velx(100, 100)
    set_accx(100, 100)
    
    p_home = posj(0,0,0,0,0,0)
    p_ready = posj(0, 0, 90, 0, 90, 0)

    # Move to a ready position
    movej(p_home, vel=100, acc=100)
    movej(p_ready, vel=100, acc=100)
    movel(posx(370, 0, 400, 0, 180, 0), vel=[50,50], acc=[50,50])

    try:
        # 1. Enable Alter Motion
        r = enable_alter_motion(n=1, mode=DR_DPOS, ref=DR_WORLD, limit_dPOS=[100.0, 100.0], limit_dPOS_per=[100.0, 100.0])
        print(f"Enable alter motion response: {r}")

        # 2. Create and start the background thread
        stop_thread_event = threading.Event()
        alter_thread = threading.Thread(target=alter_motion_thread_func, args=(stop_thread_event,))
        alter_thread.start()

        # 3. Execute the main move asynchronously
        logger.info("Starting main movel command (async)...")
        amovel(posx(370, 250, 400, 0, 180, 0), time = 5.0)

        # 4. Wait for the asynchronous move to complete by polling
        while True:
            motion_status = check_motion()
            if motion_status == DR_STATE_IDLE:
                logger.info("Motion completed.")
                break
            time.sleep(0.1)

        logger.info("Main movel command finished.")

        # 5. Signal the thread to stop and wait for it to exit
        stop_thread_event.set()
        alter_thread.join()

    finally:
        # 6. Always ensure alter_motion is disabled and the robot returns home
        disable_alter_motion()
        logger.info("Alter motion disabled.")
        movej(p_ready, vel=100, acc=100)
        movej(p_home, vel=100, acc=100)

    print('Example finished successfully!')
    rclpy.shutdown()

if __name__ == "__main__":
    main()
