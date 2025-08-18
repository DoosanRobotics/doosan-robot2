#
#  dsr_common2
#  Author: Minsoo Song (minsoo.song@doosan.com)
# 
#  Copyright (c) 2025 Doosan Robotics
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
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