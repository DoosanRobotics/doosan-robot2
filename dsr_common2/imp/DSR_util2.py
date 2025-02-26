#-*- coding: utf-8 -*-

# ##
# @mainpage
# @file     DSR_util.py
# @brief    Utilities of doosan robot  
# @author   kabdol2<kabkyoum.kim@doosan.com>   
# @version  0.20
# @Last update date     2018-12-17
# @details
#
# history
#  - support multiple robots  
#

import rclpy
import os
import threading, time
import sys
sys.dont_write_bytecode = True

############################################################################################
class CRobotSync:
    def __init__(self, r):
        self.description = "Sync for Multiple Robos"
        self.author = "Doosan Robotics"
        self.nRobot = r
        self.nIsRun = True

        self.nWaitBit  = 0
        self.nCurBit   = 0

        self.bIsWait = list()        
        self.lock    = list() 

        for i in range(r):
            self.lock.append( threading.Lock() )
            self.bIsWait.append(False)
            self.nWaitBit |= 0x1<<i         

    def CleanUp(self):
        if True == self.nIsRun:
            self.nIsRun = False
        print("~CleanUp()")

    def Wait(self, r):
        self.bIsWait[r] = True    
        self.lock[r].acquire()   
        self.bIsWait[r] = False      
        return 0

    def WakeUp(self, r):
        while self.nIsRun: 
            if(True == self.bIsWait[r]):        
                self.lock[r].release()   
                break;
            else:
                time.sleep(0.01)
        return 0

    def WakeUpAll(self):
        self.nCurBit = 0
        while self.nIsRun: 
            for i in range(self.nRobot):
                if(True == self.bIsWait[i]):        
                    self.nCurBit |= 0x1<<i;        
            if(self.nWaitBit == self.nCurBit):
                break;
        for i in range(self.nRobot):
            self.lock[i].release()   
        return 0