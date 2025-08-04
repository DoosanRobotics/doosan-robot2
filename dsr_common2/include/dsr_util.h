/*********************************************************************
 *
 * dsr_common2
 * Author: Minsoo Song (minsoo.song@doosan.com)
 * 
 * Copyright (c) 2025 Doosan Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
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