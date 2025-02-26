/*********************************************************************
 *
 * class of doosan robot control 
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

#ifndef __DSR_ROBOT_H__
#define __DSR_ROBOT_H__

#include <ros/ros.h>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <cstdlib>
#include <array>
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>

#include <dsr_msgs2/RobotError.h>
#include <dsr_msgs2/RobotState.h>
#include <dsr_msgs2/RobotStop.h>
#include <dsr_msgs2/ModbusState.h>

#include <dsr_msgs2/SetRobotMode.h>
#include <dsr_msgs2/GetRobotMode.h>
#include <dsr_msgs2/SetRobotSystem.h>
#include <dsr_msgs2/GetRobotSystem.h>
#include <dsr_msgs2/SetRobotSpeedMode.h>
#include <dsr_msgs2/GetRobotSpeedMode.h>
#include <dsr_msgs2/SetSafeStopResetType.h>
#include <dsr_msgs2/GetCurrentPose.h>
//#include <dsr_msgs2/GetLastAlarm.h>

#include <dsr_msgs2/MoveJoint.h>
#include <dsr_msgs2/MoveLine.h>
#include <dsr_msgs2/MoveJointx.h>
#include <dsr_msgs2/MoveCircle.h>
#include <dsr_msgs2/MoveSplineJoint.h>
#include <dsr_msgs2/MoveSplineTask.h>
#include <dsr_msgs2/MoveBlending.h>
#include <dsr_msgs2/MoveSpiral.h>
#include <dsr_msgs2/MovePeriodic.h>
#include <dsr_msgs2/MoveWait.h>
#include <dsr_msgs2/Jog.h>
#include <dsr_msgs2/JogMulti.h>
#include <dsr_msgs2/Trans.h>
#include <dsr_msgs2/Fkin.h>
#include <dsr_msgs2/Ikin.h>
#include <dsr_msgs2/SetRefCoord.h>
#include <dsr_msgs2/MoveHome.h>
#include <dsr_msgs2/CheckMotion.h>
#include <dsr_msgs2/ChangeOperationSpeed.h>
#include <dsr_msgs2/EnableAlterMotion.h>
#include <dsr_msgs2/AlterMotion.h>
#include <dsr_msgs2/DisableAlterMotion.h>
#include <dsr_msgs2/SetSingularityHandling.h>

#include <dsr_msgs2/GetControlMode.h>          
#include <dsr_msgs2/GetControlSpace.h>         
#include <dsr_msgs2/GetCurrentPosj.h>          
#include <dsr_msgs2/GetCurrentVelj.h>          
#include <dsr_msgs2/GetDesiredPosj.h>
#include <dsr_msgs2/GetDesiredVelj.h>          
#include <dsr_msgs2/GetCurrentPosx.h>          
#include <dsr_msgs2/GetCurrentToolFlangePosx.h>
#include <dsr_msgs2/GetCurrentVelx.h>          
#include <dsr_msgs2/GetDesiredPosx.h>
#include <dsr_msgs2/GetDesiredVelx.h>          
#include <dsr_msgs2/GetCurrentSolutionSpace.h> 
#include <dsr_msgs2/GetCurrentRotm.h>          
#include <dsr_msgs2/GetJointTorque.h>          
#include <dsr_msgs2/GetExternalTorque.h>      
#include <dsr_msgs2/GetToolForce.h>            
#include <dsr_msgs2/GetSolutionSpace.h>
#include <dsr_msgs2/GetOrientationError.h>

#include <dsr_msgs2/ParallelAxis1.h>
#include <dsr_msgs2/ParallelAxis2.h>
#include <dsr_msgs2/AlignAxis1.h>
#include <dsr_msgs2/AlignAxis2.h>
#include <dsr_msgs2/IsDoneBoltTightening.h>
#include <dsr_msgs2/ReleaseComplianceCtrl.h>
#include <dsr_msgs2/TaskComplianceCtrl.h>
#include <dsr_msgs2/SetStiffnessx.h>
#include <dsr_msgs2/CalcCoord.h>
#include <dsr_msgs2/SetUserCartCoord1.h>
#include <dsr_msgs2/SetUserCartCoord2.h>
#include <dsr_msgs2/SetUserCartCoord3.h>
#include <dsr_msgs2/OverwriteUserCartCoord.h>
#include <dsr_msgs2/GetUserCartCoord.h>
#include <dsr_msgs2/SetDesiredForce.h>
#include <dsr_msgs2/ReleaseForce.h>
#include <dsr_msgs2/CheckPositionCondition.h>
#include <dsr_msgs2/CheckForceCondition.h>
#include <dsr_msgs2/CheckOrientationCondition1.h>
#include <dsr_msgs2/CheckOrientationCondition2.h>
#include <dsr_msgs2/CoordTransform.h>
#include <dsr_msgs2/GetWorkpieceWeight.h>
#include <dsr_msgs2/ResetWorkpieceWeight.h>


#include <dsr_msgs2/ConfigCreateTcp.h>
#include <dsr_msgs2/ConfigDeleteTcp.h>
#include <dsr_msgs2/GetCurrentTcp.h>
#include <dsr_msgs2/SetCurrentTcp.h>

#include <dsr_msgs2/SetCurrentTool.h>
#include <dsr_msgs2/GetCurrentTool.h>
#include <dsr_msgs2/ConfigCreateTool.h>
#include <dsr_msgs2/ConfigDeleteTool.h>
#include <dsr_msgs2/SetToolShape.h>

#include <dsr_msgs2/SetCtrlBoxDigitalOutput.h>
#include <dsr_msgs2/GetCtrlBoxDigitalInput.h>
#include <dsr_msgs2/SetToolDigitalOutput.h>
#include <dsr_msgs2/GetToolDigitalInput.h>
#include <dsr_msgs2/SetCtrlBoxAnalogOutput.h>
#include <dsr_msgs2/GetCtrlBoxAnalogInput.h>
#include <dsr_msgs2/SetCtrlBoxAnalogOutputType.h>
#include <dsr_msgs2/SetCtrlBoxAnalogInputType.h>
#include <dsr_msgs2/GetCtrlBoxDigitalOutput.h>
#include <dsr_msgs2/GetToolDigitalOutput.h>

#include <dsr_msgs2/SetModbusOutput.h>
#include <dsr_msgs2/GetModbusInput.h>
#include <dsr_msgs2/ConfigCreateModbus.h>
#include <dsr_msgs2/ConfigDeleteModbus.h>

#include <dsr_msgs2/DrlPause.h>
#include <dsr_msgs2/DrlStart.h>
#include <dsr_msgs2/DrlStop.h>
#include <dsr_msgs2/DrlResume.h>
#include <dsr_msgs2/GetDrlState.h>

#include <dsr_msgs2/Robotiq2FOpen.h>
#include <dsr_msgs2/Robotiq2FClose.h>
#include <dsr_msgs2/Robotiq2FMove.h>
#include <dsr_msgs2/SerialSendData.h>

#include "DRFL.h"
#include "DRFC.h"
#include "DRFS.h"

using namespace std;

namespace DSR_Robot{
    class CDsrRobot
    {
        public:
            CDsrRobot(ros::NodeHandle nh, std::string robotID="dsr01", std::string robotModel="m1013");
            virtual ~CDsrRobot();

            int stop(int nMode = STOP_TYPE_QUICK);
            //----- system
            int set_robot_mode(int robot_mode = ROBOT_MODE_MANUAL);
            int get_robot_mode();
            int set_robot_system(int robot_system = ROBOT_SYSTEM_VIRTUAL);
            int get_robot_system();
            int set_robot_speed_mode(int speed_mode = SPEED_NORMAL_MODE);
            int get_robot_speed_mode();
            int set_safe_stop_reset_type(int reset_type = SAFE_STOP_RESET_TYPE_DEFAULT);
            int get_current_pose(int space_type = ROBOT_SPACE_JOINT);
            int get_current_solution_space();
            //int get_last_alarm();
            //----- sync motion
            int movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, float fBlendingRadius = 0.f,
                      int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);             

            int movel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, float fBlendingRadius = 0.f,
                      int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);             

            int movejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime=0.f, float fBlendingRadius = 0.f,
                       int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSolSpace = 0);

            int movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, float fBlendingRadius = 0.f,
                      int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);

            int movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f,
                       int nMoveMode = MOVE_MODE_ABSOLUTE);

            int movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                       int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);

            int moveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                      int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE);

            int move_spiral(float fRevolution, float fMaxRadius, float fMaxLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                            int nTaskAxis = TASK_AXIS_Z, int nMoveReference = MOVE_REFERENCE_TOOL);

            int move_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime = 0.f, 
                              int nRepeat = 1, int nMoveReference = MOVE_REFERENCE_TOOL);
 
            int jog(int jog_axis, int move_reference = MOVE_REFERENCE_BASE, int speed = 10);

            int jog_multi(float jog_axis[NUM_TASK], int move_reference = MOVE_REFERENCE_BASE, int speed = 10);

            //----- async motion
            int amovej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, float fBlendingRadius = 0.f,
                      int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);             

            int amovel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, float fBlendingRadius = 0.f,
                      int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);             

            int amovejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime=0.f, float fBlendingRadius = 0.f,
                       int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSolSpace = 0);

            int amovec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, float fBlendingRadius = 0.f,
                      int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);

            int amovesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f,
                       int nMoveMode = MOVE_MODE_ABSOLUTE);

            int amovesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                       int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);

            int amoveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                      int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE);

            int amove_spiral(float fRevolution, float fMaxRadius, float fMaxLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                            int nTaskAxis = TASK_AXIS_Z, int nMoveReference = MOVE_REFERENCE_TOOL);

            int amove_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime = 0.f, 
                              int nRepeat = 1, int nMoveReference = MOVE_REFERENCE_TOOL);

            //----- Motion Wait
            int move_wait();

            //----- TCP
            int config_create_tcp(string strName, float fTargetPos[NUM_TASK]);
            int config_delete_tcp(string strName);
            int set_current_tcp(string strName);
            string get_current_tcp();

            //----- TOOL
            int config_create_tool(string strName, float fTargetWeight, float fTargetCog[3], float fTargetInertia[NUM_TASK]);
            int config_delete_tool(string strName);
            int set_current_tool(string strName);
            string get_current_tool();

            //----- IO
            int set_digital_output(int nGpioIndex, bool bGpioValue);
            int get_digital_input(int nGpioIndex);
            int set_tool_digital_output(int nGpioIndex, bool bGpioValue);
            int get_tool_digital_input(int nGpioIndex);
            int set_analog_output(int nGpioChannel, float fGpioValue);
            int get_analog_input(int nGpioChannel);
            int set_analog_output_type(int nGpioChannel, int nGpioMode);
            int set_analog_input_type(int nGpioChannel, int nGpioMode);

            //----- MODBUS
            int config_create_modbus(string strName, string strIP, int nPort, int nRegType, int nRegIndex, int nRegValue = 0, int nSlaveID =255);
            int config_delete_modbus(string strName);
            int set_modbus_output(string strName, int nValue);
            int get_modbus_input(string strName);

            //----- DRL        
            int drl_start(int nRobotSystem, string strCode);
            int drl_stop(int nStopMode = STOP_TYPE_QUICK);
            int drl_pause();
            int drl_resume();
            int get_drl_state();

        private:
            int _movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveMode, int nBlendingType, int nSyncType); 
            int _movel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType);             
            int _movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType);
            int _movejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSolSpace, int nSyncType);
            int _move_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, int nRepeat, int nMoveReference, int nSyncType);
            int _move_spiral(float fRevolution, float fMaxRadius, float fMaxLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nTaskAxis, int nMoveReference, int nSyncType);
            int _movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime, int nMoveMode, int nSyncType);
            int _movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nMoveReference, int nMoveMode, int nVelOpt, int nSyncType);
            int _moveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nMoveReference, int nMoveMode, int nSyncType);
            
            //void thread_subscriber();
            //void msgRobotState_cb(const dsr_msgs2::RobotState::ConstPtr& msg);
            ///boost::thread m_thread_sub;

            std::string m_strSrvNamePrefix;
            std::string m_strTopicNamePrefix; 
    };
}
#endif // end