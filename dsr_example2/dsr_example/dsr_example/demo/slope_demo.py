# Demo for MuJoCo with 2f85 gripper on slope scene.
# The moveset was base on m1013, using others might cause issues.

import rclpy
import os
import sys
import time 

from rclpy.logging import get_logger
from std_msgs.msg import Float64MultiArray

ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m1013"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
        rclpy.init(args=args)

        node = rclpy.create_node('slope_demo', namespace=ROBOT_ID)

        DR_init.__dsr__node = node

        # Publisher for the gripper controller
        knuckle_pub = node.create_publisher(
                Float64MultiArray,
                f'/{ROBOT_ID}/mj/left_knuckle_position_controller/commands',
                10
        )
        try:
                from DSR_ROBOT2 import print_ext_result, movej, movejx, movesj, movesx, movel, movec, move_periodic, move_spiral, moveb, set_velx, set_accx, set_robot_mode
                from DSR_ROBOT2 import posj, posx, posb
                from DSR_ROBOT2 import DR_LINE, DR_CIRCLE, DR_BASE, DR_TOOL, DR_AXIS_X, DR_AXIS_Z, DR_MV_MOD_ABS, ROBOT_MODE_AUTONOMOUS
                # print_result("Import DSR_ROBOT2 Success!")
        except ImportError as e:
                print(f"Error importing DSR_ROBOT2 : {e}")
                return

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        set_velx(300, 80)
        set_accx(100, 40)

        # Default pose
        p1= posj(0,0,0,0,0,0)
        # Home pose
        p2= posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
        
        # Gripping pose
        x1= posx(503, -353, 190.0, 0.0, 180.0, 0.0)
        
        # Going to release pose
        x2= posx(503, -353, 290.0, 0.0, 180.0, 0.0)
        x3 = posx(503, 210, 400.0, 0.0, 180.0, 0.0)
        x4 = posx(503, 550, 420, 0, 180, 0)
        xlist = [x2, x3, x4]

        # Start to move
        movej(p1, vel=100, acc=30)
        movej(p2, vel=100, acc=30)

        # Open gripper
        msg = Float64MultiArray()
        msg.data = [-0.1]
        knuckle_pub.publish(msg)

        while rclpy.ok():

                movel(x2)
                
                movel(x1)

                # Close gripper
                time.sleep(1.0)
                msg.data = [0.8]
                knuckle_pub.publish(msg)
                time.sleep(1.0)

                movesx(xlist, vel=500, acc=200)
                time.sleep(0.2)

                # Open gripper
                msg.data = [-0.1]
                knuckle_pub.publish(msg)
                time.sleep(2.0)
                
                movej(p2, vel=80, acc=50)

        rclpy.shutdown()

if __name__ == "__main__":
        main()

