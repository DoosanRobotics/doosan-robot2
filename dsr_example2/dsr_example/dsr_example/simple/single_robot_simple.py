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

        p1= posj(0,0,0,0,0,0)                    #joint
        print(p1)
        p2= posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0) #joint

        x1= posx(400, 500, 800.0, 0.0, 180.0, 0.0) #task
        x2= posx(400, 500, 500.0, 0.0, 180.0, 0.0) #task

        c1 = posx(559,434.5,651.5,0,180,0)
        c2 = posx(559,434.5,251.5,0,180,0)


        q0 = posj(0,0,0,0,0,0)
        q1 = posj(10, -10, 20, -30, 10, 20)
        q2 = posj(25, 0, 10, -50, 20, 40) 
        q3 = posj(50, 50, 50, 50, 50, 50) 
        q4 = posj(30, 10, 30, -20, 10, 60)
        q5 = posj(20, 20, 40, 20, 0, 90)
        qlist = [q0, q1, q2, q3, q4, q5]

        x1 = posx(600, 600, 600, 0, 175, 0)
        x2 = posx(600, 750, 600, 0, 175, 0)
        x3 = posx(150, 600, 450, 0, 175, 0)
        x4 = posx(-300, 300, 300, 0, 175, 0)
        x5 = posx(-200, 700, 500, 0, 175, 0)
        x6 = posx(600, 600, 400, 0, 175, 0)
        xlist = [x1, x2, x3, x4, x5, x6]


        X1 =  posx(370, 670, 650, 0, 180, 0)
        X1a = posx(370, 670, 400, 0, 180, 0)
        X1a2= posx(370, 545, 400, 0, 180, 0)
        X1b = posx(370, 595, 400, 0, 180, 0)
        X1b2= posx(370, 670, 400, 0, 180, 0)
        X1c = posx(370, 420, 150, 0, 180, 0)
        X1c2= posx(370, 545, 150, 0, 180, 0)
        X1d = posx(370, 670, 275, 0, 180, 0)
        X1d2= posx(370, 795, 150, 0, 180, 0)


        seg11 = posb(DR_LINE, X1, radius=20)
        seg12 = posb(DR_CIRCLE, X1a, X1a2, radius=21)
        seg14 = posb(DR_LINE, X1b2, radius=20)
        seg15 = posb(DR_CIRCLE, X1c, X1c2, radius=22)
        seg16 = posb(DR_CIRCLE, X1d, X1d2, radius=23)
        b_list1 = [seg11, seg12, seg14, seg15, seg16]

        while rclpy.ok():
                movej(p2, vel=100, acc=100)

                # movejx(x1, vel=30, acc=60, sol=0)

                # movel(x2, velx, accx)

                # movec(c1, c2, velx, accx)

                # movesj(qlist, vel=100, acc=100)

                # movesx(xlist, vel=100, acc=100)

                # move_spiral(rev=9.5,rmax=20.0,lmax=50.0,time=20.0,axis=DR_AXIS_Z,ref=DR_TOOL)
                
                # move_periodic(amp =[10,0,0,0,30,0], period=1.0, atime=0.2, repeat=5, ref=DR_TOOL)
                
                # moveb(b_list1, vel=150, acc=250, ref=DR_BASE, mod=DR_MV_MOD_ABS)

                movej(p1, vel=100, acc=100)

        print('good bye!')
        rclpy.shutdown()

if __name__ == "__main__":
        main()

