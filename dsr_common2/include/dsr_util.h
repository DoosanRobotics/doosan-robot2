/*********************************************************************
 *
 * Utilities of doosan robot  
 * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Doosan Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef __DSR_UTIL_H__
#define __DSR_UTIL_H__

#include <boost/thread/thread.hpp>

#define MAX_ROBOT   8 

void time_sleep(float x){ boost::this_thread::sleep( boost::posix_time::milliseconds(int(x*1000))); }

namespace DSR_Util{

    class CRobotSync{
        int m_nRobot; 
        bool m_nIsRun; 
        bool m_bIsWait[MAX_ROBOT]; 
        unsigned int m_nWaitBit, m_nCurBit;


        boost::mutex m_io_mutex[MAX_ROBOT];
        boost::mutex::scoped_lock* m_pLock[MAX_ROBOT];
        boost::condition_variable m_condition[MAX_ROBOT];

        public:
            CRobotSync(int r){
                m_nRobot = r;    
                m_nIsRun = true;
                m_nWaitBit = m_nCurBit = 0x00;

                for(int i=0; i<m_nRobot; i++)
                    m_nWaitBit |= (0x1<<i);

                m_nCurBit = 0x00;

                for(int i=0; i<m_nRobot; i++){
                    m_bIsWait[i] = false;
                    m_pLock[i] = new boost::mutex::scoped_lock(m_io_mutex[i]);
                }    
            }
            virtual ~CRobotSync(){
                printf("~CRobotSync()\n");
                printf("~CRobotSync()\n");
                printf("~CRobotSync()\n");
                /*
                for(int i=0; i<m_nRobot; i++){
                    if(true == m_bIsWait[i])
                        m_condition[i].notify_one();
                }
                */
                m_nIsRun = false;
                /*    
                for(int i=0; i<m_nRobot; i++){
                    //delete &m_condition[i];
                    //m_io_mutex[i].release();        
                    //if(m_pLock[i]) delete m_pLock[i]; 
                }
                */
            }    
            int Wait(int nId){
                m_bIsWait[nId] = true;
                m_condition[nId].wait( *m_pLock[nId] );
                m_bIsWait[nId] = false;
            }
            int WakeUp(int nId){ 
                while(m_nIsRun){
                    if(true == m_bIsWait[nId]){                       
                        m_condition[nId].notify_one();
                        break;
                    }
                    time_sleep(0.01);    
                }
                return 0;
            } 
            int WakeUpAll(){ 
                m_nCurBit=0;
                while(m_nIsRun){
                    for(int i=0; i<m_nRobot; i++){
                        if(true == m_bIsWait[i])
                            m_nCurBit |= (0x1<<i);
                    }    
                    if(m_nWaitBit == m_nCurBit)
                        break;
                    time_sleep(0.01);    
                }
                for(int i=0; i<m_nRobot; i++)
                    m_condition[i].notify_one();
                return 0;
            } 
    };
}
#endif // end