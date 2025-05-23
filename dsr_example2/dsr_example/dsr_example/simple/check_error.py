import rclpy
import os
import sys

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
                from DSR_ROBOT2 import print_ext_result, movej, movejx, movesj, movesx, movel, movec, move_periodic, move_spiral, moveb, set_velx, set_accx, set_robot_mode
                from DSR_ROBOT2 import posj, posx, posb
                from DSR_ROBOT2 import DR_LINE, DR_CIRCLE, DR_BASE, DR_TOOL, DR_AXIS_X, DR_AXIS_Z, DR_MV_MOD_ABS, ROBOT_MODE_AUTONOMOUS
                # print_result("Import DSR_ROBOT2 Success!")
        except ImportError as e:
                print(f"Error importing DSR_ROBOT2 : {e}")
                return
        
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        set_velx(30, 20)    # set global task speed : 30(mm/sec), 20(deg/sec)
        set_accx(60, 40)    # set global task speed : 60(mm/sec2), 40(deg/sec2)

        velx = [50, 50]
        accx = [100, 100]

        # Default pose
        p1= posj(0,0,0,0,0,0)
        # Home pose
        p2= posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
        
        # Gripping pose
        x1= posx(400, 200, 185.0, 0.0, 180.0, 0.0)
        
        # Going to release pose
        x2= posx(400, 210, 185.0, 0.0, 180.0, 0.0)
        x3= posx(400, 190, 185.0, 0.0, 180.0, 0.0)

        x4= posx(400, -200, 185.0, 0.0, 180.0, 0.0)
        x5= posx(600, -200, 185.0, 0.0, 180.0, 0.0)

        # Start to move
        movej(p1, vel=100, acc=30)
        movej(p2, vel=100, acc=30)

        while rclpy.ok():
                # movej(p1, vel=100, acc=30mo
                # movejx(x1, vel=30, acc=60, sol=0)

                movel(x1, velx, accx)
                movel(x2, velx, accx)
                movel(x3, velx, accx)

                movel(x4, velx, accx)
                movel(x5, velx, accx)
                # movec(c1, c2, velx, accx)

                # movesj(qlist, vel=100, acc=100)

                # movesx(xlist, vel=100, acc=100)

                # move_spiral(rev=9.5,rmax=20.0,lmax=50.0,time=20.0,axis=DR_AXIS_Z,ref=DR_TOOL)
                
                # move_periodic(amp =[10,0,0,0,30,0], period=1.0, atime=0.2, repeat=5, ref=DR_TOOL)
                
                # moveb(b_list1, vel=150, acc=250, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        print('good bye!')
        rclpy.shutdown()

if __name__ == "__main__":
        main()

