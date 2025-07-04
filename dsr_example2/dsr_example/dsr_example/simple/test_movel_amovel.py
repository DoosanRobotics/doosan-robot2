
import rclpy
import os
import sys
import time
import math
import threading
from rclpy.logging import get_logger
from std_msgs.msg import String,Int32,Int32MultiArray,Float32,Float64,Float32MultiArray,Float64MultiArray,MultiArrayLayout,MultiArrayDimension


from DRFC import *
from DR_common2 import *

from dsr_msgs2.msg import *
from dsr_msgs2.srv import *
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
            movej, movel, enable_alter_motion, disable_alter_motion, check_motion, get_current_posx,
            set_velx, set_accx, amovel, get_robot_state, posj, posx, DR_DPOS, DR_WORLD, DR_STATE_IDLE
        )
    except ImportError as e:
        logger.error(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_velx(100, 100)
    set_accx(100, 100)
    
    p_home = posj(0,0,0,0,0,0)
    p_ready = posj(0, 0, 90, 0, 90, 0)

    movej(p_home, vel=100, acc=100)
    movej(p_ready, vel=100, acc=100)

    amovel(posx(370, 200, 400, 0, 180, 0), time = 2.0)
    print(f"Current Position: {get_current_posx()}")

    time.sleep(2.0)
    print('Example finished successfully!')
    rclpy.shutdown()

if __name__ == "__main__":
    main()
