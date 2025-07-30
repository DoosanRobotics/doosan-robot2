/*    ========================================================================
    =                   Doosan Robot Framework Library                        =
    =                   Copyright (c) Doosan Robotics.                        =   
    =_______________________________________________________________________  =
    = Title             : Doosan Robot Framwork Library                       =
    = Author            : Lee Jeong-Woo<jeongwoo1.lee@doosan.com>             =
    = Maintainer        : Minsoo Song<minsoo.song@doosan.com>                 =
    =                     Minju Lee<minju3.lee@doosan.com>                    =
    = Description       : -                                                   =
    ======================================================================== */

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Doosan Robotics
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

#pragma once

#if defined(_WIN32)
#if defined(DRFL_EXPORTS)
#define DRFL_API __declspec(dllexport)
#else
#define DRFL_API __declspec(dllimport)
#endif
#endif

#if !defined(DRFL_API)
#define DRFL_API
#endif

#include "DRFS.h"
#include "DRFC.h"


namespace DRAFramework 
{
    typedef void* LPROBOTCONTROL;

    typedef void (*TOnMonitoringStateCB)(const ROBOT_STATE);
    typedef void (*TOnMonitoringDataCB)(const LPMONITORING_DATA);
    typedef void (*TOnMonitoringCtrlIOCB)(const LPMONITORING_CTRLIO);
    typedef void (*TOnMonitoringModbusCB)(const LPMONITORING_MODBUS);
    typedef void (*TOnLogAlarmCB)(LPLOG_ALARM);
    typedef void (*TOnMonitoringAccessControlCB)(const MONITORING_ACCESS_CONTROL);
    typedef void (*TOnHommingCompletedCB)();
    typedef void (*TOnTpInitializingCompletedCB)();
    typedef void (*TOnProgramStoppedCB)(const PROGRAM_STOP_CAUSE);
    typedef void (*TOnMonitoringSpeedModeCB)(const MONITORING_SPEED);
    typedef void (*TOnMasteringNeedCB)();
    typedef void (*TOnDisconnectedCB)();


    typedef void (*TOnRTMonitoringDataCB)(const LPRT_OUTPUT_DATA_LIST);
    
    
#ifdef __cplusplus
    extern "C" 
    {
#endif
        ////////////////////////////////////////////////////////////////////////////
        // Instance                                                               //
        ////////////////////////////////////////////////////////////////////////////
        // construct
        DRFL_API LPROBOTCONTROL _CreateRobotControl();
        DRFL_API void _DestroyRobotControl(LPROBOTCONTROL pCtrl);

        // connection
        //for ROS org DRFL_API bool _OpenConnection(LPROBOTCONTROL pCtrl, const char* lpszIpAddr = "192.168.137.100");
        DRFL_API bool _OpenConnection(LPROBOTCONTROL pCtrl, const char* lpszIpAddr = "192.168.137.100", unsigned int usPort = 12345);
        DRFL_API void _CloseConnection(LPROBOTCONTROL pCtrl);

        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////
        // get verion string
        DRFL_API bool _GetSystemVersion(LPROBOTCONTROL pCtrl, LPSYSTEM_VERSION pVersion);
        DRFL_API const char* _GetLibraryVersion(LPROBOTCONTROL pCtrl);

        // get robot safety mode(manual, auto)
        DRFL_API ROBOT_MODE _GetRobotMode(LPROBOTCONTROL pCtrl);
        // set robot mode mode(manual, auto)
        DRFL_API bool _SetRobotMode(LPROBOTCONTROL pCtrl, ROBOT_MODE eMode);

       // get robot state( initial, standby, moving, safe-off, teach, ...) 
        DRFL_API ROBOT_STATE _GetRobotState(LPROBOTCONTROL pCtrl);
        // set robot control state
        DRFL_API bool _SetRobotControl(LPROBOTCONTROL pCtrl, ROBOT_CONTROL eControl);
        
        // get robot system(real robot, virtrul robot)
        DRFL_API ROBOT_SYSTEM _GetRobotSystem(LPROBOTCONTROL pCtrl);
        // set robot system(real robot, virtrul robot)
        DRFL_API bool _SetRobotSystem(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM eRobotSystem);

        // set robot speed mode(noraml reduced)
        DRFL_API bool _SetRobotSpeedMode(LPROBOTCONTROL pCtrl, SPEED_MODE eSpeedMode);      
        // get robot speed mode(noraml reduced)
        DRFL_API SPEED_MODE _GetRobotSpeedMode(LPROBOTCONTROL pCtrl);

        // get roobt axis data
        DRFL_API LPROBOT_POSE _GetCurrentPose(LPROBOTCONTROL pCtrl, ROBOT_SPACE eSpaceType = ROBOT_SPACE_JOINT);

        // get current solution space
        DRFL_API unsigned char _GetCurrentSolutionSpace(LPROBOTCONTROL pCtrl);

        // get program running state
        DRFL_API DRL_PROGRAM_STATE _GetProgramState(LPROBOTCONTROL pCtrl);

        // set safe-stop reset type
        DRFL_API void _SetSafeStopResetType(LPROBOTCONTROL pCtrl, SAFE_STOP_RESET_TYPE eResetType = SAFE_STOP_RESET_TYPE_DEFAULT);

        // get robot system alarm
        DRFL_API LPLOG_ALARM _GetLastAlarm(LPROBOTCONTROL pCtrl);
        
                
        ////////////////////////////////////////////////////////////////////////////
        //  access control                                                       //
        ////////////////////////////////////////////////////////////////////////////

        // manage access control
        DRFL_API bool _ManageAccessControl(LPROBOTCONTROL pCtrl, MANAGE_ACCESS_CONTROL eAccessControl = MANAGE_ACCESS_CONTROL_REQUEST);
        
        ////////////////////////////////////////////////////////////////////////////
        // Callback operation                                                    //
        ////////////////////////////////////////////////////////////////////////////
        DRFL_API void _SetOnMonitoringState(LPROBOTCONTROL pCtrl, TOnMonitoringStateCB pCallbackFunc);
        DRFL_API void _SetOnMonitoringData(LPROBOTCONTROL pCtrl, TOnMonitoringDataCB pCallbackFunc);   
        DRFL_API void _SetOnMonitoringCtrlIO(LPROBOTCONTROL pCtrl, TOnMonitoringCtrlIOCB pCallbackFunc);
        DRFL_API void _SetOnMonitoringModbus(LPROBOTCONTROL pCtrl, TOnMonitoringModbusCB pCallbackFunc);
        DRFL_API void _SetOnMonitoringSpeedMode(LPROBOTCONTROL pCtrl, TOnMonitoringSpeedModeCB pCallbackFunc);
        DRFL_API void _SetOnMonitoringAccessControl(LPROBOTCONTROL pCtrl, TOnMonitoringAccessControlCB pCallbackFunc);
        DRFL_API void _SetOnLogAlarm(LPROBOTCONTROL pCtrl, TOnLogAlarmCB pCallbackFunc);
        DRFL_API void _SetOnProgramStopped(LPROBOTCONTROL pCtrl, TOnProgramStoppedCB pCallbackFunc);
        DRFL_API void _SetOnHommingCompleted(LPROBOTCONTROL pCtrl, TOnHommingCompletedCB pCallbackFunc);
        DRFL_API void _SetOnTpInitializingCompleted(LPROBOTCONTROL pCtrl, TOnTpInitializingCompletedCB pCallbackFunc);
        DRFL_API void _SetOnMasteringNeed(LPROBOTCONTROL pCtrl, TOnMasteringNeedCB pCallbackFunc);
        DRFL_API void _SetOnDisconnected(LPROBOTCONTROL pCtrl, TOnDisconnectedCB pCallbackFunc);
        
        ////////////////////////////////////////////////////////////////////////////
        //  motion Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////
        // basci motion(hold to run)
        DRFL_API bool _Jog(LPROBOTCONTROL pCtrl, JOG_AXIS eJogAxis, MOVE_REFERENCE eMoveReference, float fVelocity);
        DRFL_API bool _MultiJog(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], MOVE_REFERENCE eMoveReference, float fVelocity);
        DRFL_API bool _Home(LPROBOTCONTROL pCtrl, unsigned char bRun);

        // stop motion
        DRFL_API bool _MoveStop(LPROBOTCONTROL pCtrl, STOP_TYPE eStopType = STOP_TYPE_QUICK);
        // pause motion
        DRFL_API bool _MovePause(LPROBOTCONTROL pCtrl);
        // resume motion
        DRFL_API bool _MoveResume(LPROBOTCONTROL pCtrl);
        // wait motion
        DRFL_API bool _MoveWait(LPROBOTCONTROL pCtrl);

        // joint motion
        DRFL_API bool _MoveJ(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _MoveJ_ex(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _MoveJAsync(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _MoveJAsync_ex(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        // linear motion
        DRFL_API bool _MoveL(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, DR_MV_APP eAppType=DR_MV_APP_NONE);
        DRFL_API bool _MoveLAsync(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, DR_MV_APP eAppType=DR_MV_APP_NONE);
        // circle motion
        DRFL_API bool _MoveC(LPROBOTCONTROL pCtrl, float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _MoveCAsync(LPROBOTCONTROL pCtrl, float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        // bleind motion
        DRFL_API bool _MoveB(LPROBOTCONTROL pCtrl, MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, DR_MV_APP eAppType=DR_MV_APP_NONE);
        DRFL_API bool _MoveBAsync(LPROBOTCONTROL pCtrl, MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, DR_MV_APP eAppType=DR_MV_APP_NONE);
        // joint motion as task information
        DRFL_API bool _MoveJX(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _MoveJX_ex(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _MoveJXAsync(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _MoveJXAsync_ex(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        // spline motion as joint information
        DRFL_API bool _MoveSJ(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
        DRFL_API bool _MoveSJ_ex(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
        DRFL_API bool _MoveSJAsync(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
        DRFL_API bool _MoveSJAsync_ex(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
        // spline motion as task information
        DRFL_API bool _MoveSX(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);
        DRFL_API bool _MoveSXAsync(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);
        // spiral motion
        DRFL_API bool _MoveSpiral(LPROBOTCONTROL pCtrl, TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
        DRFL_API bool _MoveSpiralAsync(LPROBOTCONTROL pCtrl, TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
        // periodic motion
        DRFL_API bool _MovePeriodic(LPROBOTCONTROL pCtrl, float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned char nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
        DRFL_API bool _MovePeriodicAsync(LPROBOTCONTROL pCtrl, float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned char nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
		// safe motion
		DRFL_API bool _Safe_MoveJ(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _Safe_MoveJ_ex(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
		DRFL_API bool _Safe_MoveL(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _Safe_MoveJX(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        
        ////////////////////////////////////////////////////////////////////////////
        //  GPIO Operations                                                       //
        ////////////////////////////////////////////////////////////////////////////
        // set digital output on flange
        DRFL_API bool _SetToolDigitalOutput(LPROBOTCONTROL pCtrl, GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool bOnOff);
        // get digital input on flange
        DRFL_API bool _GetToolDigitalInput(LPROBOTCONTROL pCtrl, GPIO_TOOL_DIGITAL_INDEX eGpioIndex);
        // set digital ouput on control-box
        DRFL_API bool _SetCtrlBoxDigitalOutput(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool bOnOff);
        // get digital input on control-box
        DRFL_API bool _GetCtrlBoxDigitalInput(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex);

        // set analog ouput on control-box
        DRFL_API bool _SetCtrlBoxAnalogOutput(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float fValue);
        // get analog inut on control-box
        DRFL_API float _GetCtrlBoxAnalogInput(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex);

        // set analog input type on control-box
        DRFL_API bool _SetCtrlBoxAnalogInputType(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT); 
        // set analog output type on control-box
        DRFL_API bool _SetCtrlBoxAnalogOutputType(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT); 

        ////////////////////////////////////////////////////////////////////////////
        //  Modbus Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////
        // set modbus register 
        DRFL_API bool _SetModbusValue(LPROBOTCONTROL pCtrl, const char* lpszSymbol, unsigned short nValue);
        // get modbus register
        DRFL_API unsigned short _GetModbusValue(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // add modbus register
        DRFL_API bool _ConfigCreateModbus(LPROBOTCONTROL pCtrl, const char* lpszSymbol, const char* lpszIpAddress, unsigned short nPort, MODBUS_REGISTER_TYPE eRegType, unsigned short iRegIndex, unsigned short nRegValue = 0, unsigned char nSlaveId = 255);
        // del modbus register
        DRFL_API bool _ConfigDeleteModbus(LPROBOTCONTROL pCtrl, const char* lpszSymbol);

        ////////////////////////////////////////////////////////////////////////////
        //  Configuration Operations                                               //
        ////////////////////////////////////////////////////////////////////////////
        // set tool(end-effector) information
        DRFL_API bool _SetCurrentTool(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // add tool(end-effector) information
        DRFL_API bool _ConfigCreateTool(LPROBOTCONTROL pCtrl, const char* lpszSymbol, float fWeight, float fCog[3], float fInertia[NUM_TASK]);
        // del tool(end-effector) informaiton
        DRFL_API bool _ConfigDeleteTool(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // get tool(end-effector) information
        DRFL_API const char* _GetCurrentTool(LPROBOTCONTROL pCtrl);

        // set robot tcp information
        DRFL_API bool _SetCurrentTCP(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // add robot tcp information
        DRFL_API bool _ConfigCreateTCP(LPROBOTCONTROL pCtrl, const char* lpszSymbol, float fPostion[NUM_TASK]);
        // del robot tcp information
        DRFL_API bool _ConfigDeleteTCP(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // get robot tcp information
        DRFL_API const char* _GetCurrentTCP(LPROBOTCONTROL pCtrl);  

        ////////////////////////////////////////////////////////////////////////////
        //  drl program Operations                                                //
        ////////////////////////////////////////////////////////////////////////////
        // program start
        DRFL_API bool _PlayDrlStart(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM eRobotSystem, const char* lpszDrlProgram);
        // program stop
        DRFL_API bool _PlayDrlStop(LPROBOTCONTROL pCtrl, unsigned char eStopType = 0);
        // program Pause
        DRFL_API bool _PlayDrlPause(LPROBOTCONTROL pCtrl);
        // program Resume
        DRFL_API bool _PlayDrlResume(LPROBOTCONTROL pCtrl);

        void PrintFParam(float* printArr, int iSize, string strFunc);
        void PrintUCParam(unsigned char* printArr, int iSize, string strFunc);
        bool CheckNewFlange();

#ifdef __cplusplus
    };
#endif

#ifdef __cplusplus
    class CDRFL
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Connection                                                             //
        ////////////////////////////////////////////////////////////////////////////
        // construct
        CDRFL() { _rbtCtrl = _CreateRobotControl(); }
        virtual ~CDRFL() { _DestroyRobotControl(_rbtCtrl);   }

        // connection
        //for ROS org bool OpenConnection(string strIpAddr = "192.168.137.100") { return _OpenConnection(_rbtCtrl, strIpAddr.c_str()); };
        bool OpenConnection(string strIpAddr = "192.168.137.100", unsigned int usPort= 12345) { return _OpenConnection(_rbtCtrl, strIpAddr.c_str(), usPort); };
        void CloseConnection() { _CloseConnection(_rbtCtrl); }


        ////////////////////////////////////////////////////////////////////////////
        // Callback operation                                                    //
        ////////////////////////////////////////////////////////////////////////////
        // robot status data
        void SetOnMonitoringState(TOnMonitoringStateCB pCallbackFunc) { _SetOnMonitoringState(_rbtCtrl, pCallbackFunc); };

        // modbus I/O data
        void SetOnMonitoringModbus(TOnMonitoringModbusCB pCallbackFunc) { _SetOnMonitoringModbus(_rbtCtrl, pCallbackFunc); };
        // robot speed mode event
        void SetOnMonitoringSpeedMode(TOnMonitoringSpeedModeCB pCallbackFunc) { _SetOnMonitoringSpeedMode(_rbtCtrl, pCallbackFunc); };
        // robot access control event
        void SetOnMonitoringAccessControl(TOnMonitoringAccessControlCB pCallbackFunc) { _SetOnMonitoringAccessControl(_rbtCtrl, pCallbackFunc); };
        // roobt alaram data
        void SetOnLogAlarm(TOnLogAlarmCB pCallbackFunc)  { _SetOnLogAlarm(_rbtCtrl, pCallbackFunc); };
        // robot homing completed event
        void SetOnHommingCompleted(TOnHommingCompletedCB pCallbackFunc) { _SetOnHommingCompleted(_rbtCtrl, pCallbackFunc); };
        // Tp Initailzing completed
        void SetOnTpInitializingCompleted(TOnTpInitializingCompletedCB pCallbackFunc) { _SetOnTpInitializingCompleted(_rbtCtrl, pCallbackFunc); };
        // robot mastering needed event
        void SetOnMasteringNeed(TOnMasteringNeedCB pCallbackFunc) { _SetOnMasteringNeed(_rbtCtrl, pCallbackFunc); };
        // program stopeed event
        void SetOnProgramStopped(TOnProgramStoppedCB pCallbackFunc) { _SetOnProgramStopped(_rbtCtrl, pCallbackFunc); };
        // robot disconneted event
        void SetOnDisconnected(TOnDisconnectedCB pCallbackFunc) { _SetOnDisconnected(_rbtCtrl, pCallbackFunc); };

        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////
        // get verion string
        bool GetSystemVersion(LPSYSTEM_VERSION pVersion) { return _GetSystemVersion(_rbtCtrl, pVersion); };
        const char* GetLibraryVersion() { return _GetLibraryVersion(_rbtCtrl); };

        // get robot safety mode(manual, auto)
        ROBOT_MODE GetRobotMode() { return _GetRobotMode(_rbtCtrl); };
        // set robot mode mode(manual, auto)
        bool SetRobotMode(ROBOT_MODE eMode) { return _SetRobotMode(_rbtCtrl, eMode); };

        // get robot state( initial, standby, moving, safe-off, teach, ...) 
        ROBOT_STATE GetRobotState() { return _GetRobotState(_rbtCtrl); };
        // set robot state 
        bool SetRobotControl(ROBOT_CONTROL eControl) { return _SetRobotControl(_rbtCtrl, eControl); };
        // get robot system(real robot, virtrul robot)
        ROBOT_SYSTEM GetRobotSystem() { return _GetRobotSystem(_rbtCtrl); };
        // set robot system(real robot, virtrul robot)
        bool SetRobotSystem(ROBOT_SYSTEM eRobotSystem) { return _SetRobotSystem(_rbtCtrl, eRobotSystem); };

        // set robot speed mode(noraml reduced)
        bool SetRobotSpeedMode(SPEED_MODE eSpeedMode) { return _SetRobotSpeedMode(_rbtCtrl, eSpeedMode); };
        // get robot speed mode(noraml reduced)
        SPEED_MODE GetRobotSpeedMode() { return _GetRobotSpeedMode(_rbtCtrl); };
        
        // get roobt axis data
        LPROBOT_POSE GetCurrentPose(ROBOT_SPACE eSpaceType = ROBOT_SPACE_JOINT) { return _GetCurrentPose(_rbtCtrl, eSpaceType); };
       
        unsigned char GetCurrentSolutionSpace() { return _GetCurrentSolutionSpace(_rbtCtrl); };
      
        // get program running state
        DRL_PROGRAM_STATE GetProgramState() { return _GetProgramState(_rbtCtrl); };

        // set safe-stop reset type
        void SetSafeStopResetType(SAFE_STOP_RESET_TYPE eResetType = SAFE_STOP_RESET_TYPE_DEFAULT) { _SetSafeStopResetType(_rbtCtrl, eResetType); }

        // get roobot system alarm
        LPLOG_ALARM GetLastAlarm() { return _GetLastAlarm(_rbtCtrl); };

        ////////////////////////////////////////////////////////////////////////////
        //  access control                                                        //
        ////////////////////////////////////////////////////////////////////////////

        // manage access control
        bool ManageAccessControl(MANAGE_ACCESS_CONTROL eAccessControl = MANAGE_ACCESS_CONTROL_REQUEST) { return _ManageAccessControl(_rbtCtrl, eAccessControl); };
        
        ////////////////////////////////////////////////////////////////////////////
        //  motion Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////
        // basic control(hold to run)
        bool Jog(JOG_AXIS eJogAxis, MOVE_REFERENCE eMoveReference, float fVelocity) { return _Jog(_rbtCtrl, eJogAxis, eMoveReference, fVelocity); };
        bool MultiJog(float fTargetPos[NUM_TASK], MOVE_REFERENCE eMoveReference, float fVelocity) { return _MultiJog(_rbtCtrl, fTargetPos, eMoveReference, fVelocity); };
        bool Home(unsigned char bRun) { return _Home(_rbtCtrl, bRun); };
        // motion control: move stop
        bool MoveStop(STOP_TYPE eStopType = STOP_TYPE_QUICK) { return _MoveStop(_rbtCtrl, eStopType); };
        // motion control: move pause
        bool MovePause() { return _MovePause(_rbtCtrl); };
        // motion control: move resume
        bool MoveResume() { return _MoveResume(_rbtCtrl); };
        // wait motion
        bool MoveWait() { return _MoveWait(_rbtCtrl); };

        
        // motion control: joint move
        bool MoveJ(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _MoveJ(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, fBlendingRadius, eBlendingType); };
        bool MoveJ(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _MoveJ_ex(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, fBlendingRadius, eBlendingType); };
        bool MoveJAsync(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _MoveJAsync(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eBlendingType); };
        bool MoveJAsync(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _MoveJAsync_ex(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eBlendingType); };
        // motion control: linear move
        bool MoveL(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, DR_MV_APP eAppType=DR_MV_APP_NONE) { return _MoveL(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType, eAppType); }
        bool MoveLAsync(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, DR_MV_APP eAppType=DR_MV_APP_NONE) { return _MoveLAsync(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eBlendingType, eAppType); }
        // motion control: circle move
        bool MoveC(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _MoveC(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fTargetAngle1, fTargetAngle2, fBlendingRadius, eBlendingType); };
        bool MoveCAsync(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _MoveCAsync(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fTargetAngle1, fTargetAngle2, eBlendingType); };
        // motion control: blending move
        bool MoveB(MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, DR_MV_APP eAppType=DR_MV_APP_NONE) { return _MoveB(_rbtCtrl, tTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eAppType); };
        bool MoveBAsync(MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, DR_MV_APP eAppType=DR_MV_APP_NONE) { return _MoveBAsync(_rbtCtrl, tTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eAppType); };
        // motion control: joint move as task information
        bool MoveJX(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _MoveJX(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); };
        bool MoveJX(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _MoveJX_ex(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); };
        bool MoveJXAsync(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _MoveJXAsync(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eBlendingType); };
        bool MoveJXAsync(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _MoveJXAsync_ex(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eBlendingType); };
        // spline motion as joint information
        bool MoveSJ(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _MoveSJ(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
        bool MoveSJ(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _MoveSJ_ex(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
        bool MoveSJAsync(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _MoveSJAsync(_rbtCtrl, fTargetPos ,nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
        bool MoveSJAsync(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _MoveSJAsync_ex(_rbtCtrl, fTargetPos ,nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
        // motion control: spline motin as task information
        bool MoveSX(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT) { return _MoveSX(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eVelOpt); };
        bool MoveSXAsync(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT) { return _MoveSXAsync(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eVelOpt); };
        // motion control: move spiral motion
        bool MoveSpiral(TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _MoveSpiral(_rbtCtrl, eTaskAxis, fRevolution, fMaximuRadius, fMaximumLength, fTargetVel, fTargetAcc, fTargetTime, eMoveReference); };
        bool MoveSpiralAsync(TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _MoveSpiralAsync(_rbtCtrl, eTaskAxis, fRevolution, fMaximuRadius, fMaximumLength, fTargetVel, fTargetAcc, fTargetTime, eMoveReference); };
        // motion control: move periodic motion
        bool MovePeriodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned int nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _MovePeriodic(_rbtCtrl, fAmplitude, fPeriodic, fAccelTime, nRepeat, eMoveReference); };
        bool MovePeriodicAsync(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned int nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _MovePeriodicAsync(_rbtCtrl, fAmplitude, fPeriodic, fAccelTime, nRepeat, eMoveReference); };
		// safe motion
		bool Safe_MoveJ(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _Safe_MoveJ(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, fBlendingRadius, eBlendingType); };
        bool Safe_MoveJ(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _Safe_MoveJ_ex(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, fBlendingRadius, eBlendingType); };
        bool Safe_MoveL(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _Safe_MoveL(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); }
        bool Safe_MoveJX(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _Safe_MoveJX(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); };
        

        ////////////////////////////////////////////////////////////////////////////
        //  GPIO Operations                                                       //
        ////////////////////////////////////////////////////////////////////////////
        // set digital output on flange
        bool SetToolDigitalOutput(GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool bOnOff) { return _SetToolDigitalOutput(_rbtCtrl, eGpioIndex, bOnOff); };
        // get digital input on flange
        bool GetToolDigitalInput(GPIO_TOOL_DIGITAL_INDEX eGpioIndex) { return _GetToolDigitalInput(_rbtCtrl, eGpioIndex); };
        // set digital ouput on control-box
    

        // set analog ouput on control-box
        bool SetCtrlBoxAnalogOutput(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float fValue) { return _SetCtrlBoxAnalogOutput(_rbtCtrl, eGpioIndex, fValue); };
        // get analog inut on control-box
        float GetCtrlBoxAnalogInput(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex) { return _GetCtrlBoxAnalogInput(_rbtCtrl, eGpioIndex); };
        // set analog input type on control-box
        bool SetCtrlBoxAnalogInputType(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT) { return _SetCtrlBoxAnalogInputType(_rbtCtrl, eGpioIndex, eAnalogType); }; 
        // set analog output type on control-box
        bool SetCtrlBoxAnalogOutputType(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT) { return _SetCtrlBoxAnalogOutputType(_rbtCtrl, eGpioIndex, eAnalogType); };

        ////////////////////////////////////////////////////////////////////////////
        //  Modbus Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////
        // set modbus register
        bool SetModbusValue(string strSymbol, unsigned short nValue) { return _SetModbusValue(_rbtCtrl, strSymbol.c_str(), nValue); };
        // get modbus register
        unsigned short GetModbusValue(string strSymbol) { return _GetModbusValue(_rbtCtrl, strSymbol.c_str()); };
        // add modbus register
        bool ConfigCreateModbus(string strSymbol, string strIpAddress, unsigned short nPort, MODBUS_REGISTER_TYPE eRegType, unsigned short iRegIndex, unsigned short nRegValue = 0, unsigned char nSlaiveId = 255) { return _ConfigCreateModbus(_rbtCtrl, strSymbol.c_str(), strIpAddress.c_str(), nPort, eRegType, iRegIndex, nRegValue, nSlaiveId); };
        // del modbus register
        bool ConfigDeleteModbus(string strSymbol) { return _ConfigDeleteModbus(_rbtCtrl, strSymbol.c_str()); };

        ////////////////////////////////////////////////////////////////////////////
        //  Configuration Operations                                               //
        ////////////////////////////////////////////////////////////////////////////
        // set tool(end-effector) information
        bool SetCurrentTool(string strSymbol) { return _SetCurrentTool(_rbtCtrl, strSymbol.c_str()); };
        // get tool(end-effector) information
        string GetCurrentTool() { return string(_GetCurrentTool(_rbtCtrl)); };
        // add tool(end-effector) information
        bool ConfigCreateTool(string strSymbol, float fWeight, float fCog[3], float fInertia[NUM_TASK]) { return _ConfigCreateTool(_rbtCtrl, strSymbol.c_str(), fWeight, fCog, fInertia); };
        // del tool(end-effector) informaiton
        bool ConfigDeleteTool(string strSymbol) { return _ConfigDeleteTool(_rbtCtrl, strSymbol.c_str()); };
        
        // set robot tcp information
        bool SetCurrentTCP(string strSymbol) { return _SetCurrentTCP(_rbtCtrl, strSymbol.c_str()); };
        // get robot tcp information
        string GetCurrentTCP() { return string(_GetCurrentTCP(_rbtCtrl)); };  
        // add robot tcp information
        bool ConfigCreateTCP(string strSymbol, float fPostion[NUM_TASK]) { return _ConfigCreateTCP(_rbtCtrl, strSymbol.c_str(), fPostion); };
        // del robot tcp information
        bool ConfigDeleteTCP(string strSymbol) { return _ConfigDeleteTCP(_rbtCtrl, strSymbol.c_str()); };
        
        ////////////////////////////////////////////////////////////////////////////
        //  drl program Operations                                                //
        ////////////////////////////////////////////////////////////////////////////
        //program start
        bool PlayDrlStart(ROBOT_SYSTEM eRobotSystem, string strDrlProgram) { return _PlayDrlStart(_rbtCtrl, eRobotSystem, strDrlProgram.c_str()); };
        //program stop
        bool PlayDrlStop(unsigned char eStopType = 0) { return _PlayDrlStop(_rbtCtrl, eStopType); };
        //program pause
        bool PlayDrlPause()  { return _PlayDrlPause(_rbtCtrl); };
        //program resume
        bool PlayDrlResume() { return _PlayDrlResume(_rbtCtrl); };

protected:
        LPROBOTCONTROL _rbtCtrl;
    };
#endif
}

