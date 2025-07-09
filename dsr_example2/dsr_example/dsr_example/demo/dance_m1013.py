import rclpy
import os
import sys

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m1013"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
        rclpy.init(args=args)

        node = rclpy.create_node('dsr_example_demo_py', namespace=ROBOT_ID)

        DR_init.__dsr__node = node

        try:
                from DSR_ROBOT2 import print_ext_result, movej, movel, movec, move_periodic, move_spiral, set_velx, set_accx, DR_BASE, DR_TOOL, DR_AXIS_X, DR_MV_MOD_ABS
                # print_result("Import DSR_ROBOT2 Success!")
        except ImportError as e:
                print(f"Error importing DSR_ROBOT2 : {e}")
                return
        #################
        ### Set values ##
        set_velx(30, 20)
        set_accx(60, 40)

        JReady = [0, -20, 110, 0, 60, 0]

        TCP_POS = [0, 0, 0, 0, 0, 0]
        J00 = [-180, 0, -145, 0, -35, 0]

        J01r = [-180.0, 71.4, -145.0, 0.0, -9.7, 0.0]
        J02r = [-180.0, 67.7, -144.0, 0.0, 76.3, 0.0]
        J03r = [-180.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        J04r = [-90.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        J04r1 = [-90.0, 30.0, -60.0, 0.0, 30.0, -0.0]
        J04r2 = [-90.0, -45.0, 90.0, 0.0, -45.0, -0.0]
        J04r3 = [-90.0, 60.0, -120.0, 0.0, 60.0, -0.0]
        J04r4 = [-90.0, 0.0, -0.0, 0.0, 0.0, -0.0]

        J05r = [-144.0, -4.0, -84.8, -90.9, 54.0, -1.1]
        J07r = [-152.4, 12.4, -78.6, 18.7, -68.3, -37.7]
        J08r = [-90.0, 30.0, -120.0, -90.0, -90.0, 0.0]

        JEnd = [0.0, -12.6, 101.1, 0.0, 91.5, -0.0]

        dREL1 = [0, 0, 350, 0, 0, 0]
        dREL2 = [0, 0, -350, 0, 0, 0]

        velx = [0, 0]
        accx = [0, 0]

        vel_spi = [400, 400]
        acc_spi = [150, 150]

        J1 = [81.2, 20.8, 127.8, 162.5, 56.1, -37.1]
        X0 = [-88.7, 799.0, 182.3, 95.7, 93.7, 133.9]
        X1 = [304.2, 871.8, 141.5, 99.5, 84.9, 133.4]
        X2 = [437.1, 876.9, 362.1, 99.6, 84.0, 132.1]
        X3 = [-57.9, 782.4, 478.4, 99.6, 84.0, 132.1]

        amp = [0, 0, 0, 30, 30, 0]
        period = [0, 0, 0, 3, 6, 0]

        x01 = [423.6, 334.5, 651.2, 84.7, -180.0, 84.7]
        x02 = [423.6, 34.5, 951.2, 68.2, -180.0, 68.2]
        x03 = [423.6, -265.5, 651.2, 76.1, -180.0, 76.1]
        x04 = [423.6, 34.5, 351.2, 81.3, -180.0, 81.3]
        x0204c = [x02, x04]

        ###########################
        ### A variety of motions ##
        while rclpy.ok():
                movej(JReady, v=20, a=20)

                movej(J1, v=0, a=0, t=3)
                movel(X3, velx, accx, t=2.5)

                for i in range(0, 1):
                        movel(X2, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                        movel(X1, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                        movel(X0, velx, accx, t=2.5) 
                        movel(X1, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                        movel(X2, velx, accx, t=1.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                        movel(X3, velx, accx, t=2.5, radius=50, ref=DR_BASE, mod=DR_MV_MOD_ABS)

                movej(J00, v=60, a=60, t=6)

                movej(J01r, v=0, a=0, t=2, radius=100, mod=DR_MV_MOD_ABS)
                movej(J02r, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS) 
                movej(J03r, v=0, a=0, t=2)

                movej(J04r, v=0, a=0, t=1.5)
                movej(J04r1, v=0, a=0, t=2, radius=50, mod=DR_MV_MOD_ABS)
                movej(J04r2, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)
                movej(J04r3, v=0, a=0, t=4, radius=50, mod=DR_MV_MOD_ABS)
                movej(J04r4, v=0, a=0, t=2)

                movej(J05r, v=0, a=0, t=2.5, radius=100, mod=DR_MV_MOD_ABS) 
                movel(dREL1, velx, accx, t=1, radius=50, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 
                movel(dREL2, velx, accx, t=1.5, radius=100, ref=DR_TOOL, mod=DR_MV_MOD_ABS) 

                movej(J07r, v=60, a=60, t=1.5, radius=100, mod=DR_MV_MOD_ABS) 
                movej(J08r, v=60, a=60, t=2)

                movej(JEnd, v=60, a=60, t=4)

                move_periodic(amp, period, 0, 1, ref=DR_TOOL)
                move_spiral(rev=3, rmax=200, lmax=100, v=vel_spi, a=acc_spi, t=0, axis=DR_AXIS_X, ref=DR_TOOL)

                movel(x01, velx, accx, t=2)
                movel(x04, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                movel(x03, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                movel(x02, velx, accx, t=2, radius=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                movel(x01, velx, accx, t=2)  

                movec(pos1=x02, pos2=x04, v=velx, a=accx, t=4, radius=360, mod=DR_MV_MOD_ABS, ref=DR_BASE)
        
        print('good bye!')
        rclpy.shutdown()

if __name__ == "__main__":
        main()