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

#include "DRFL.h"

// Specified by user. 2 or 3.
#ifndef DRCF_VERSION
    #define DRCF_VERSION 3
#endif

namespace DRAFramework 
{
    typedef void (*TOnMonitoringDataExCB)(const LPMONITORING_DATA_EX);
    typedef void (*TOnMonitoringCtrlIOExCB)(const LPMONITORING_CTRLIO_EX);
    typedef void (*TOnMonitoringCtrlIOEx2CB)(const LPMONITORING_CTRLIO_EX2);
    typedef void (*TOnTpPopupCB)(LPMESSAGE_POPUP);
    typedef void (*TOnTpLogCB)(const char[256]);
    typedef void (*TOnTpGetUserInputCB)(LPMESSAGE_INPUT);
    typedef void (*TOnTpProgressCB)(LPMESSAGE_PROGRESS); 

    typedef void (*TOnRTMonitoringDataCB)(const LPRT_OUTPUT_DATA_LIST);

    typedef void (*TOnMonitoringSafetyStateCB)(const SAFETY_STATE);
    typedef void (*TOnMonitoringRobotSystemCB)(const ROBOT_SYSTEM);
    typedef void (*TOnMonitoringSafetyStopTypeCB)(const unsigned char);
    typedef void (*TOnMonitoringWeldingDataCB)(const LPROBOT_WELDING_DATA);
    typedef void (*TOnMonitoringAnalogWeldingDataCB)(const LPMONITORING_ALALOG_WELDING);
    typedef void (*TOnMonitoringDigitalWeldingDataCB)(const LPMONITORING_DIGITAL_WELDING);



#ifdef __cplusplus
    extern "C" 
    {
#endif
        //////////////////////
        /////DRL Wrapping/////
        //////////////////////
        
        DRFL_API LPROBOTCONTROL _create_robot_control_udp();
        DRFL_API void _destroy_robot_control_udp(LPROBOTCONTROL pCtrl);

        ////////////////////////////////////////////////////////////////////////////
        // Instance                                                               //
        ////////////////////////////////////////////////////////////////////////////

        // connection
        //for ROS org DRFL_API bool _OpenConnection(LPROBOTCONTROL pCtrl, const char* lpszIpAddr = "192.168.137.100");
        DRFL_API bool _open_connection(LPROBOTCONTROL pCtrl, const char* lpszIpAddr = "192.168.137.100", unsigned int usPort = 12345);
        DRFL_API bool _close_connection(LPROBOTCONTROL pCtrl);

        DRFL_API bool _connect_rt_control(LPROBOTCONTROL pCtrl, const char* lpszIpAddr = "192.168.137.100", unsigned int usPort = 12347);
        DRFL_API bool _disconnect_rt_control(LPROBOTCONTROL pCtrl);

        ////////////////////////////////////////////////////////////////////////////
        // RT Control                                                             //
        ////////////////////////////////////////////////////////////////////////////   

        DRFL_API const char* _get_rt_control_output_version_list(LPROBOTCONTROL pCtrl);
        DRFL_API const char* _get_rt_control_input_version_list(LPROBOTCONTROL pCtrl);
        DRFL_API const char* _get_rt_control_input_data_list(LPROBOTCONTROL pCtrl, const char* szVersion);
        DRFL_API const char* _get_rt_control_output_data_list(LPROBOTCONTROL pCtrl, const char* szVersion);
        DRFL_API bool _set_rt_control_input(LPROBOTCONTROL pCtrl, const char* szVersion, float fPeriod, int nLossCnt);
        DRFL_API bool _set_rt_control_output(LPROBOTCONTROL pCtrl, const char* szVersion, float fPeriod, int nLossCnt);
        
        DRFL_API bool _start_rt_control(LPROBOTCONTROL pCtrl);
        DRFL_API bool _stop_rt_control(LPROBOTCONTROL pCtrl);

        DRFL_API bool _set_velj_rt(LPROBOTCONTROL pCtrl, float fTargetVel[NUM_JOINT]);
        DRFL_API bool _set_accj_rt(LPROBOTCONTROL pCtrl, float fTargetAcc[NUM_JOINT]);
        DRFL_API bool _set_velx_rt(LPROBOTCONTROL pCtrl, float fTransVel, float fRotationVel = -10000);
        DRFL_API bool _set_accx_rt(LPROBOTCONTROL pCtrl, float fTransAcc, float fRotationAcc = -10000);

        DRFL_API LPRT_OUTPUT_DATA_LIST _read_data_rt(LPROBOTCONTROL pCtrl);
        DRFL_API bool _write_data_rt(LPROBOTCONTROL pCtrl, float fExternalForceTorque[NUM_JOINT], int iExternalDI, int iExternalDO, float fExternalAnalogInput[6], float fExternalAnalogOutput[6]);
        
        DRFL_API bool _servoj_rt(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
        DRFL_API bool _servol_rt(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[NUM_TASK], float fTargetAcc[NUM_TASK], float fTargetTime);
        DRFL_API bool _speedj_rt(LPROBOTCONTROL pCtrl, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
        DRFL_API bool _speedl_rt(LPROBOTCONTROL pCtrl, float fTargetVel[NUM_TASK], float fTargetAcc[NUM_TASK], float fTargetTime);
        DRFL_API bool _torque_rt(LPROBOTCONTROL pCtrl, float fMotorTor[NUM_JOINT], float fTargetTime);

        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // get verion string
        DRFL_API bool _get_system_version(LPROBOTCONTROL pCtrl, LPSYSTEM_VERSION pVersion);
        DRFL_API const char* _get_library_version(LPROBOTCONTROL pCtrl);

        // get robot safety mode(manual, auto)
        DRFL_API ROBOT_MODE _get_robot_mode(LPROBOTCONTROL pCtrl);

        // set robot mode mode(manual, auto)
        DRFL_API bool _set_robot_mode(LPROBOTCONTROL pCtrl, ROBOT_MODE eMode);

       // get robot state( initial, standby, moving, safe-off, teach, ...) 
        DRFL_API ROBOT_STATE _get_robot_state(LPROBOTCONTROL pCtrl);
        // set robot control state
        DRFL_API bool _set_robot_control(LPROBOTCONTROL pCtrl, ROBOT_CONTROL eControl);
        DRFL_API CONTROL_MODE _get_control_mode(LPROBOTCONTROL pCtrl);
        
        // get robot system(real robot, virtrul robot)
        DRFL_API ROBOT_SYSTEM _get_robot_system(LPROBOTCONTROL pCtrl);
        // set robot system(real robot, virtrul robot)
        DRFL_API bool _set_robot_system(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM eRobotSystem);

        // set robot speed mode(noraml reduced)
        DRFL_API bool _set_robot_speed_mode(LPROBOTCONTROL pCtrl, SPEED_MODE eSpeedMode);      
        // get robot speed mode(noraml reduced)
        DRFL_API SPEED_MODE _get_robot_speed_mode(LPROBOTCONTROL pCtrl);

        // get roobt axis data
        DRFL_API LPROBOT_POSE _get_current_pose(LPROBOTCONTROL pCtrl, ROBOT_SPACE eSpaceType = ROBOT_SPACE_JOINT);

        // get rotation matrix
        DRFL_API float(* _get_current_rotm(LPROBOTCONTROL pCtrl, COORDINATE_SYSTEM eTargetRef))[3];
        
        // get current solution space
        DRFL_API unsigned char _get_current_solution_space(LPROBOTCONTROL pCtrl);
        
        // get current joint position list
        DRFL_API LPROBOT_POSE _get_current_posj(LPROBOTCONTROL pCtrl);
        // get current control space
        DRFL_API ROBOT_SPACE _get_control_space(LPROBOTCONTROL pCtrl);
        // get current joint velocity
        DRFL_API LPROBOT_VEL _get_current_velj(LPROBOTCONTROL pCtrl);
        // get target joint angle
        DRFL_API LPROBOT_POSE _get_desired_posj(LPROBOTCONTROL pCtrl);
        // get current flange task position
        DRFL_API LPROBOT_POSE _get_current_tool_flange_posx(LPROBOTCONTROL pCtrl);
        // get current task velocity
        DRFL_API LPROBOT_VEL _get_current_velx(LPROBOTCONTROL pCtrl);
        // get target task position
        //DRFL_API LPROBOT_POSE _get_desired_posx(LPROBOTCONTROL pCtrl);
        // get target task velocity
        DRFL_API LPROBOT_VEL _get_desired_velx(LPROBOTCONTROL pCtrl);
        // get current joint sensor torque value
        DRFL_API LPROBOT_FORCE _get_joint_torque(LPROBOTCONTROL pCtrl);
        // get current external force
        DRFL_API LPROBOT_FORCE _get_external_torque(LPROBOTCONTROL pCtrl);
        // get current external force in tool
        DRFL_API LPROBOT_FORCE _get_tool_force(LPROBOTCONTROL pCtrl, COORDINATE_SYSTEM eTargetRef);

        // get program running state
        DRFL_API DRL_PROGRAM_STATE _get_program_state(LPROBOTCONTROL pCtrl);

        // set safe-stop reset type
        DRFL_API bool _set_safe_stop_reset_type(LPROBOTCONTROL pCtrl, SAFE_STOP_RESET_TYPE eResetType = SAFE_STOP_RESET_TYPE_DEFAULT);

        // get robot system alarm
        DRFL_API LPLOG_ALARM _get_last_alarm(LPROBOTCONTROL pCtrl);        
                
        ////////////////////////////////////////////////////////////////////////////
        //  access control                                                       //
        ////////////////////////////////////////////////////////////////////////////

        // manage access control
        DRFL_API bool _manage_access_control(LPROBOTCONTROL pCtrl, MANAGE_ACCESS_CONTROL eAccessControl = MANAGE_ACCESS_CONTROL_REQUEST);
        
        ////////////////////////////////////////////////////////////////////////////
        // Callback operation                                                    //
        ////////////////////////////////////////////////////////////////////////////
        DRFL_API void _set_on_monitoring_state(LPROBOTCONTROL pCtrl, TOnMonitoringStateCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_data(LPROBOTCONTROL pCtrl, TOnMonitoringDataCB pCallbackFunc);   
        DRFL_API void _set_on_monitoring_data_ex(LPROBOTCONTROL pCtrl, TOnMonitoringDataExCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_ctrl_io(LPROBOTCONTROL pCtrl, TOnMonitoringCtrlIOCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_ctrl_io_ex(LPROBOTCONTROL pCtrl, TOnMonitoringCtrlIOExCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_ctrl_io_ex2(LPROBOTCONTROL pCtrl, TOnMonitoringCtrlIOEx2CB pCallbackFunc);
        DRFL_API void _set_on_monitoring_welding_data(LPROBOTCONTROL pCtrl, TOnMonitoringWeldingDataCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_analog_welding_data(LPROBOTCONTROL pCtrl, TOnMonitoringAnalogWeldingDataCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_digital_welding_data(LPROBOTCONTROL pCtrl, TOnMonitoringDigitalWeldingDataCB pCallbackFunc);


        DRFL_API void _set_on_monitoring_modbus(LPROBOTCONTROL pCtrl, TOnMonitoringModbusCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_speed_mode(LPROBOTCONTROL pCtrl, TOnMonitoringSpeedModeCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_access_control(LPROBOTCONTROL pCtrl, TOnMonitoringAccessControlCB pCallbackFunc);
        DRFL_API void _set_on_log_alarm(LPROBOTCONTROL pCtrl, TOnLogAlarmCB pCallbackFunc);
        DRFL_API void _set_on_tp_popup(LPROBOTCONTROL pCtrl, TOnTpPopupCB pCallbackFunc);
        DRFL_API void _set_on_tp_log(LPROBOTCONTROL pCtrl, TOnTpLogCB pCallbackFunc);
        DRFL_API void _set_on_tp_progress(LPROBOTCONTROL pCtrl, TOnTpProgressCB pCallbackFunc);
        DRFL_API void _set_on_tp_get_user_input(LPROBOTCONTROL pCtrl, TOnTpGetUserInputCB pCallbackFunc);
        DRFL_API void _set_on_program_stopped(LPROBOTCONTROL pCtrl, TOnProgramStoppedCB pCallbackFunc);
        DRFL_API void _set_on_homming_completed(LPROBOTCONTROL pCtrl, TOnHommingCompletedCB pCallbackFunc);
        DRFL_API void _set_on_tp_initializing_completed(LPROBOTCONTROL pCtrl, TOnTpInitializingCompletedCB pCallbackFunc);
        DRFL_API void _set_on_mastering_need(LPROBOTCONTROL pCtrl, TOnMasteringNeedCB pCallbackFunc);
        DRFL_API void _set_on_disconnected(LPROBOTCONTROL pCtrl, TOnDisconnectedCB pCallbackFunc);

        DRFL_API void _set_on_monitoring_safety_state(LPROBOTCONTROL pCtrl, TOnMonitoringSafetyStateCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_robot_system(LPROBOTCONTROL pCtrl, TOnMonitoringRobotSystemCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_safety_stop_type(LPROBOTCONTROL pCtrl, TOnMonitoringSafetyStopTypeCB pCallbackFunc);

        DRFL_API void _set_on_rt_monitoring_data(LPROBOTCONTROL pCtrl, TOnRTMonitoringDataCB pCallbackFunc);
        DRFL_API void _set_on_rt_log_alarm(LPROBOTCONTROL pCtrl,TOnLogAlarmCB pCallbackFunc);

        DRFL_API LPROBOT_POSE _trans(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_TASK], float fOffset[NUM_TASK], COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE, COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
        DRFL_API LPROBOT_POSE _ikin(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
		DRFL_API LPINVERSE_KINEMATIC_RESPONSE _ikin_ex(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef, unsigned char iRefPosOpt);
        DRFL_API LPINVERSE_KINEMATIC_RESPONSE _ikin_add_iter_threshold(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef, float fIterThreshold[2]);
        DRFL_API LPINVERSE_KINEMATIC_RESPONSE _ikin_norm(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef,  unsigned char iRefPosOpt);
        
        DRFL_API LPROBOT_POSE _fkin(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_JOINT], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
        DRFL_API LPROBOT_POSE _addto(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_JOINT], float fOffset[NUM_JOINT]);

        DRFL_API unsigned char _get_solution_space(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK]);

        DRFL_API LPROBOT_TASK_POSE _get_current_posx(LPROBOTCONTROL pCtrl, COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE);
        DRFL_API LPROBOT_POSE _get_desired_posx(LPROBOTCONTROL pCtrl, COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE);
        DRFL_API float _get_orientation_error(LPROBOTCONTROL pCtrl, float fPosition1[NUM_TASK], float fPosition2[NUM_TASK], TASK_AXIS eTaskAxis);

        DRFL_API bool _set_workpiece_weight(LPROBOTCONTROL pCtrl, float fWeight = 0.0, float fCog[3] = COG_DEFAULT, COG_REFERENCE eCogRef = COG_REFERENCE_TCP, ADD_UP eAddUp = ADD_UP_REPLACE, float fStartTime = -10000, float fTransitionTIme = -10000);

        DRFL_API float _get_workpiece_weight(LPROBOTCONTROL pCtrl);
        DRFL_API bool _reset_workpiece_weight(LPROBOTCONTROL pCtrl);
         
        DRFL_API bool _tp_popup_response(LPROBOTCONTROL pCtrl, POPUP_RESPONSE eRes);
        DRFL_API bool _tp_get_user_input_response(LPROBOTCONTROL pCtrl, const char* lpszTextString);
        ////////////////////////////////////////////////////////////////////////////
        //  motion Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////
        // basci motion(hold to run)
        DRFL_API bool _jog(LPROBOTCONTROL pCtrl, JOG_AXIS eJogAxis, MOVE_REFERENCE eMoveReference, float fVelocity);
        DRFL_API bool _multi_jog(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], MOVE_REFERENCE eMoveReference, float fVelocity);
        DRFL_API bool _move_home(LPROBOTCONTROL pCtrl, MOVE_HOME eMode = MOVE_HOME_MECHANIC, unsigned char bRun = (unsigned char)1);
        DRFL_API LPROBOT_POSE _get_user_home(LPROBOTCONTROL pCtrl);

        // stop motion
        DRFL_API bool _stop(LPROBOTCONTROL pCtrl, STOP_TYPE eStopType = STOP_TYPE_QUICK);
        // pause motion
        DRFL_API bool _move_pause(LPROBOTCONTROL pCtrl);
        // resume motion
        DRFL_API bool _move_resume(LPROBOTCONTROL pCtrl);
        // wait motion
        DRFL_API bool _mwait(LPROBOTCONTROL pCtrl);

        // joint motion
        DRFL_API bool _movej(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _movej_ex(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _amovej(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _amovej_ex(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        // linear motion
        DRFL_API bool _movel(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, DR_MV_APP eAppType = DR_MV_APP_NONE);
        DRFL_API bool _amovel(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, DR_MV_APP eAppType = DR_MV_APP_NONE);
        // circle motion
        DRFL_API bool _movec(LPROBOTCONTROL pCtrl, float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _movec_ex(LPROBOTCONTROL pCtrl, float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, MOVE_ORIENTATION eOrientation = DR_MV_ORI_TEACH, DR_MV_APP eAppType = DR_MV_APP_NONE);
        DRFL_API bool _amovec(LPROBOTCONTROL pCtrl, float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _amovec_ex(LPROBOTCONTROL pCtrl, float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, MOVE_ORIENTATION eOrientation = DR_MV_ORI_TEACH, DR_MV_APP eAppType = DR_MV_APP_NONE);
        // bleind motion
        DRFL_API bool _moveb(LPROBOTCONTROL pCtrl, MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, DR_MV_APP eAppType = DR_MV_APP_NONE);
        DRFL_API bool _amoveb(LPROBOTCONTROL pCtrl, MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, DR_MV_APP eAppType = DR_MV_APP_NONE);
        // joint motion as task information
        DRFL_API bool _movejx(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _movejx_ex(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _amovejx(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _amovejx_ex(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        // spline motion as joint information
        DRFL_API bool _movesj(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
        DRFL_API bool _movesj_ex(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel[NUMBER_OF_JOINT], float fTargetAcc[NUMBER_OF_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
        DRFL_API bool _amovesj(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
        DRFL_API bool _amovesj_ex(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel[NUMBER_OF_JOINT], float fTargetAcc[NUMBER_OF_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
        // spline motion as task information
        DRFL_API bool _movesx(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);
        DRFL_API bool _amovesx(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);
        // spiral motion
        DRFL_API bool _move_spiral(LPROBOTCONTROL pCtrl, TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
        DRFL_API bool _move_spiral_ex(LPROBOTCONTROL pCtrl, TASK_AXIS eTaskAxis, float fRevolution, float fTargetPos[3], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, SPIRAL_DIR eSpiralDir = DR_SPIRAL_OUTWARD, ROT_DIR eRotDir = DR_ROT_FORWARD);
        DRFL_API bool _amove_spiral(LPROBOTCONTROL pCtrl, TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
        DRFL_API bool _amove_spiral_ex(LPROBOTCONTROL pCtrl, TASK_AXIS eTaskAxis, float fRevolution, float fTargetPos[3], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, SPIRAL_DIR eSpiralDir = DR_SPIRAL_OUTWARD, ROT_DIR eRotDir = DR_ROT_FORWARD);
        // periodic motion
        DRFL_API bool _move_periodic(LPROBOTCONTROL pCtrl, float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned char nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
        DRFL_API bool _amove_periodic(LPROBOTCONTROL pCtrl, float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned char nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);

        // environment adaptive motion
        DRFL_API bool _servoj(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fLimitVel[NUM_JOINT], float fLimitAcc[NUM_JOINT], float fTargetTime, DR_SERVOJ_TYPE eTargetMod = DR_SERVO_OVERRIDE);
        DRFL_API bool _servol(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fLimitVel[2], float fLimitAcc[2], float fTargetTime);

        DRFL_API bool _speedj(LPROBOTCONTROL pCtrl, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
        DRFL_API bool _speedl(LPROBOTCONTROL pCtrl, float fTargetVel[NUM_TASK], float fTargetAcc[2], float fTargetTime);

		DRFL_API bool _servoj_g(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
        DRFL_API bool _servol_g(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime);
 
		DRFL_API bool _movesx_g(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime /* = 0.f */, MOVE_MODE eMoveMode /* = MOVE_MODE_ABSOLUTE */, MOVE_REFERENCE eMoveReference /* = MOVE_REFERENCE_BASE */, SPLINE_VELOCITY_OPTION eVelOpt /* = SPLINE_VELOCITY_OPTION_DEFAULT */);
		DRFL_API bool _movesj_g(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime /* = 0.f */, MOVE_MODE eMoveMode /* = MOVE_MODE_ABSOLUTE */);
		////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////////////////
        //  GPIO Operations                                                       //
        ////////////////////////////////////////////////////////////////////////////
        // set digital output on flange
        DRFL_API bool _set_tool_digital_output(LPROBOTCONTROL pCtrl, GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool bOnOff);
        // get digital input on flange
        DRFL_API bool _get_tool_digital_input(LPROBOTCONTROL pCtrl, GPIO_TOOL_DIGITAL_INDEX eGpioIndex);
        DRFL_API bool _get_tool_digital_output(LPROBOTCONTROL pCtrl, GPIO_TOOL_DIGITAL_INDEX eGpioIndex);
        // set digital ouput on control-box
        DRFL_API bool _set_digital_output(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool bOnOff);
        DRFL_API bool _set_digital_output_ex(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool bOnOff);
        DRFL_API bool _get_digital_output(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex);
        DRFL_API bool _get_digital_output_ex(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex);
        // get digital input on control-box
        DRFL_API bool _get_digital_input(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex);
        DRFL_API bool _get_digital_input_ex(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex);
        // set analog ouput on control-box
        DRFL_API bool _set_analog_output(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float fValue);
        // get analog inut on control-box
        DRFL_API float _get_analog_input(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex);
        DRFL_API float _get_analog_input_ex(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex);
        // set analog input type on control-box
        DRFL_API bool _set_mode_analog_input(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT); 
        // set analog output type on control-box
        DRFL_API bool _set_mode_analog_output(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT); 

        DRFL_API float _get_tool_analog_input(LPROBOTCONTROL pCtrl, int nCh);
        DRFL_API bool _set_tool_digital_output_level(LPROBOTCONTROL pCtrl, int nLv);
        DRFL_API bool _set_tool_digital_output_type(LPROBOTCONTROL pCtrl, int nPort, OUTPUT_TYPE eOutputType);
        DRFL_API bool _set_mode_tool_analog_input(LPROBOTCONTROL pCtrl, int nCh, GPIO_ANALOG_TYPE eAnalogType);

        ////////////////////////////////////////////////////////////////////////////
        //  Modbus Operations                                                     //
		////////////////////////////////////////////////////////////////////////////
        // set modbus register 
        DRFL_API bool _set_modbus_output(LPROBOTCONTROL pCtrl, const char* lpszSymbol, unsigned short nValue);
        // get modbus register
        DRFL_API unsigned short _get_modbus_input(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // add modbus register
        DRFL_API bool _add_modbus_signal(LPROBOTCONTROL pCtrl, const char* lpszSymbol, const char* lpszIpAddress, unsigned short nPort, MODBUS_REGISTER_TYPE eRegType, unsigned short iRegIndex, unsigned short nRegValue = 0, unsigned char nSlaveId = 255);
        // del modbus register
        DRFL_API bool _del_modbus_signal(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // get all of the modbus data list
        DRFL_API LPMODBUS_DATA_LIST _query_modbus_data_list(LPROBOTCONTROL pCtrl);

        ////////////////////////////////////////////////////////////////////////////
        //  Flange Serial Operations                                              //
        ////////////////////////////////////////////////////////////////////////////
        DRFL_API bool _flange_serial_open(LPROBOTCONTROL pCtrl, int nPort, int baudrate = 115200, BYTE_SIZE eByteSize = BYTE_SIZE_EIGHTBITS, PARITY_CHECK eParity = PARITY_CHECK_NONE, STOP_BITS eStopBits = STOPBITS_ONE);
        DRFL_API bool _flange_serial_close(LPROBOTCONTROL pCtrl, int nPort);
        DRFL_API bool _flange_serial_write(LPROBOTCONTROL pCtrl, int nSize, char* pSendData, int nPort = 1);
        DRFL_API LPFLANGE_SER_RXD_INFO_EX _flange_serial_read(LPROBOTCONTROL pCtrl, float fTimeout = -1, int nPort = 1);     
  
        ////////////////////////////////////////////////////////////////////////////
        //  Configuration Operations                                              //
        ////////////////////////////////////////////////////////////////////////////
        // set tool(end-effector) information
        DRFL_API bool _set_tool(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // add tool(end-effector) information
        DRFL_API bool _add_tool(LPROBOTCONTROL pCtrl, const char* lpszSymbol, float fWeight, float fCog[3], float fInertia[NUM_TASK]);
        // del tool(end-effector) informaiton
        DRFL_API bool _del_tool(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // get tool(end-effector) information
        DRFL_API const char* _get_tool(LPROBOTCONTROL pCtrl);

        // set robot tcp information
        DRFL_API bool _set_tcp(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // add robot tcp information
        DRFL_API bool _add_tcp(LPROBOTCONTROL pCtrl, const char* lpszSymbol, float fPostion[NUM_TASK]);
        // del robot tcp information
        DRFL_API bool _del_tcp(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // get robot tcp information
        DRFL_API const char* _get_tcp(LPROBOTCONTROL pCtrl);  

        DRFL_API bool _set_tool_shape(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        DRFL_API const char* _get_tool_shape(LPROBOTCONTROL pCtrl);  
        DRFL_API bool _set_user_home(LPROBOTCONTROL pCtrl);

        DRFL_API int _servo_off(LPROBOTCONTROL pCtrl, STOP_TYPE eStopType);      
        DRFL_API int _check_motion(LPROBOTCONTROL pCtrl);
        DRFL_API bool _release_protective_stop(LPROBOTCONTROL pCtrl, RELEASE_MODE eReleaseMode);
        DRFL_API bool _set_safety_mode(LPROBOTCONTROL pCtrl, SAFETY_MODE eSafetyMode, SAFETY_MODE_EVENT eSafetyEvent);
        DRFL_API bool _set_auto_servo_off(LPROBOTCONTROL pCtrl, bool bFuncEnable, float fElapseTime);

        DRFL_API LPSAFETY_CONFIGURATION_EX _get_safety_configuration(LPROBOTCONTROL pCtrl);
        DRFL_API LPSAFETY_CONFIGURATION_EX2 _get_safety_configuration_ex(LPROBOTCONTROL pCtrl);
        
        DRFL_API LPSAFETY_CONFIGURATION_EX2_V3 _get_safety_configuration_ex_v3(LPROBOTCONTROL pCtrl);

        DRFL_API bool _change_collision_sensitivity(LPROBOTCONTROL pCtrl, float fSensitivity);
        DRFL_API bool _set_palletizing_mode(LPROBOTCONTROL pCtrl, unsigned char iMode);
        
        DRFL_API bool _set_auto_safety_move_stop(LPROBOTCONTROL pCtrl, bool bFuncEnable);
        ////////////////////////////////////////////////////////////////////////////
        //  drl program Operations                                                //
        ////////////////////////////////////////////////////////////////////////////
        // program start
        DRFL_API bool _drl_start(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM eRobotSystem, const char* lpszDrlProgram);
        // program stop
        DRFL_API bool _drl_stop(LPROBOTCONTROL pCtrl, unsigned char eStopType = 0);
        // program Pause
        DRFL_API bool _drl_pause(LPROBOTCONTROL pCtrl);
        // program Resume
        DRFL_API bool _drl_resume(LPROBOTCONTROL pCtrl);
        // program speed
        DRFL_API bool _change_operation_speed(LPROBOTCONTROL pCtrl, float fSpeed);

        ////////////////////////////////////////////////////////////////////////////
        //  force control                                                        //
        ////////////////////////////////////////////////////////////////////////////

        DRFL_API bool _task_compliance_ctrl(LPROBOTCONTROL pCtrl, float fTargetStiffness[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f);
        //DRFL_API bool _joint_compliance_ctrl(LPROBOTCONTROL pCtrl, float fTargetStiffness[NUM_TASK], float fTargetTime = 0.f);
        DRFL_API bool _set_stiffnessx(LPROBOTCONTROL pCtrl, float fTargetStiffness[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f);
        DRFL_API bool _release_compliance_ctrl(LPROBOTCONTROL pCtrl);
        //DRFL_API bool _release_joint_compliance_ctrl(LPROBOTCONTROL pCtrl);
        DRFL_API bool _set_desired_force(LPROBOTCONTROL pCtrl, float fTargetForce[NUM_TASK], unsigned char iTargetDirection[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f, FORCE_MODE eForceMode = FORCE_MODE_ABSOLUTE);
        DRFL_API bool _release_force(LPROBOTCONTROL pCtrl, float fTargetTime = 0.f);

        DRFL_API bool _check_force_condition(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
        DRFL_API bool _check_position_condition_abs(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
        DRFL_API bool _check_position_condition_rel(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
        DRFL_API bool _check_position_condition(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], MOVE_MODE eMode = MOVE_MODE_ABSOLUTE, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
        DRFL_API bool _check_orientation_condition_abs(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin[NUM_TASK], float fTargetMax[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
        DRFL_API bool _check_orientation_condition_rel(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
        DRFL_API bool _is_done_bolt_tightening(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetTor = 0.f, float fTimeout = 0.f);

        DRFL_API bool _parallel_axis1(LPROBOTCONTROL pCtrl, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE);
        DRFL_API bool _align_axis1(LPROBOTCONTROL pCtrl, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE);
        DRFL_API bool _parallel_axis2(LPROBOTCONTROL pCtrl, float fTargetVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE);
        DRFL_API bool _align_axis2(LPROBOTCONTROL pCtrl, float fTargetVec[3], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE);

        ////////////////////////////////////////////////////////////////////////////
        //  coordinate system control                                             //
        ////////////////////////////////////////////////////////////////////////////

        DRFL_API int _set_user_cart_coord1(LPROBOTCONTROL pCtrl, int iReqId, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
        DRFL_API int _set_user_cart_coord2(LPROBOTCONTROL pCtrl, float fTargetPos[3][NUM_TASK], float fTargetOrg[3], COORDINATE_SYSTEM fTargetRef);
        DRFL_API int _set_user_cart_coord3(LPROBOTCONTROL pCtrl, float fTargetVec[2][3], float fTargetOrg[3], COORDINATE_SYSTEM fTargetRef);
        DRFL_API LPROBOT_POSE _coord_transform(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eInCoordSystem, COORDINATE_SYSTEM eOutCoordSystem);
        DRFL_API bool _set_ref_coord(LPROBOTCONTROL pCtrl, COORDINATE_SYSTEM eTargetCoordSystem);
        DRFL_API LPROBOT_POSE _calc_coord(LPROBOTCONTROL pCtrl, unsigned short nCnt, unsigned short nInputMode, COORDINATE_SYSTEM eTargetRef, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fTargetPos4[NUM_TASK]);
        DRFL_API LPUSER_COORDINATE _get_user_cart_coord(LPROBOTCONTROL pCtrl, int iReqId);
        DRFL_API int _overwrite_user_cart_coord(LPROBOTCONTROL pCtrl, bool bTargetUpdate, int iReqId, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
        DRFL_API bool _enable_alter_motion(LPROBOTCONTROL pCtrl, int iCycleTime, PATH_MODE ePathMode, COORDINATE_SYSTEM eTargetRef, float fLimitDpos[2], float fLimitDposPer[2]);
        DRFL_API bool _disable_alter_motion(LPROBOTCONTROL pCtrl);
        DRFL_API bool _alter_motion(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK]);
        DRFL_API bool _set_singularity_handling(LPROBOTCONTROL pCtrl, SINGULARITY_AVOIDANCE eMode);
        DRFL_API bool _config_program_watch_variable(LPROBOTCONTROL pCtrl, VARIABLE_TYPE eDivision, DATA_TYPE eType, const char* szName, const char* szData);
        DRFL_API bool _save_sub_program(LPROBOTCONTROL pCtrl, int iTargetType, const char* szFileName, const char* lpszTextString);
        DRFL_API bool _setup_monitoring_version(LPROBOTCONTROL pCtrl, int iVersion);
        DRFL_API bool _system_shut_down(LPROBOTCONTROL pCtrl);

        //brief(270) :app_weld_weave_cond_trapezoidal
        DRFL_API bool _app_weld_weave_cond_trapezoidal(LPROBOTCONTROL pCtrl, CONFIG_TRAPEZOID_WEAVING_SETTING pConfigtrapezoidweavingsetting);
        //brief(271) :app_weld_weave_cond_zigzag
        DRFL_API bool _app_weld_weave_cond_zigzag(LPROBOTCONTROL pCtrl, float fOffsetY, float fOffsetZ, float fGradient, float fWeavingWidth, float fWeavingCycle);
        //brief(272) :app_weld_weave_cond_circular
        DRFL_API bool _app_weld_weave_cond_circular(LPROBOTCONTROL pCtrl, float fOffsetY, float fOffsetZ, float fGradient, float fwWdt[2], float fwT[2]);
        //brief(273) :app_weld_weave_cond_sinusoidal
        DRFL_API bool _app_weld_weave_cond_sinusoidal(LPROBOTCONTROL pCtrl, float fOffsetY, float fOffsetZ, float fGradient, float fWeavingWidth, float fWeavingCycle);
        //brief(274) :app_weld_enable_analog
        DRFL_API bool _app_weld_enable_analog(LPROBOTCONTROL pCtrl, CONFIG_ANALOG_WELDING_INTERFACE pConfiganalogweldinginterface);
        //brief(275) :app_weld_set_weld_cond_analog
        DRFL_API bool _app_weld_set_weld_cond_analog(LPROBOTCONTROL pCtrl, CONFIG_ANALOG_WELDING_SETTING pConfigAnalogWeldingSetting);
        //brief(276) :app_weld_adj_welding_cond_analog
        DRFL_API bool _app_weld_adj_welding_cond_analog(LPROBOTCONTROL pCtrl, unsigned char bRealTime, unsigned char bResetFlag, float fTargetVol, float fFeedingVel, float fTargetVel, float fOffsetY, float fOffsetZ, float fWidthRate);
        //brief(277) :app_weld_set_interface_eip_r2m_process
        DRFL_API bool _app_weld_set_interface_eip_r2m_process(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS pConfigdigitalweldinginterfaceprocess);
        //brief(278) :app_weld_set_interface_eip_r2m_mode
        DRFL_API bool _app_weld_set_interface_eip_r2m_mode(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_MODE pConfigdigitalweldinginterfacemode);
        //brief(279) :app_weld_set_interface_eip_r2m_test
        DRFL_API bool _app_weld_set_interface_eip_r2m_test(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_TEST pConfigdigitalweldinginterfacetest);
        
        //brief(280) :app_weld_set_interface_eip_r2m_condition
        DRFL_API bool _app_weld_set_interface_eip_r2m_condition(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_CONDITION pConfigdigitalweldinginterfacecondition);
        //brief(281) :app_weld_set_interface_eip_r2m_option
        DRFL_API bool _app_weld_set_interface_eip_r2m_option(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_OPTION pConfigdigitalweldinginterfaceoption);
        //brief(282) :app_weld_set_interface_eip_m2r_process2
        DRFL_API bool _app_weld_set_interface_eip_m2r_process2(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS2 pConfigdigitalweldinginterfaceprocess2);
        //brief(283) :app_weld_set_interface_eip_m2r_monitoring
        DRFL_API bool _app_weld_set_interface_eip_m2r_monitoring(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_MONITORING pConfigdigitalweldinginterfacemonitoring);
        //brief(284) :app_weld_set_interface_eip_m2r_other
        DRFL_API bool _app_weld_set_interface_eip_m2r_other(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_OTHER pConfigdigitalweldinginterfacemonitoring);
        //brief(285) :app_weld_reset_interface
        DRFL_API bool _app_weld_reset_interface(LPROBOTCONTROL pCtrl, unsigned char bReset);
        //brief(286) :app_weld_enable_digital
        DRFL_API bool _app_weld_enable_digital(LPROBOTCONTROL pCtrl, unsigned char bMode);
        //brief(286) :app_weld_disable_digital
        DRFL_API bool _app_weld_disable_digital(LPROBOTCONTROL pCtrl, unsigned char bMode);
        //brief(287) :app_weld_set_weld_cond_digital
        DRFL_API bool _app_weld_set_weld_cond_digital(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_CONDITION pConfigdigitalweldingcondition);
        //brief(288) :app_weld_adj_welding_cond_digital
        DRFL_API bool _app_weld_adj_welding_cond_digital(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_ADJUST pConfigdigitalweldingadjust);
        //brief(289) :measure_welding_tcp
        DRFL_API LPMEASURE_TCP_RESPONSE _measure_welding_tcp(LPROBOTCONTROL pCtrl, unsigned char iMode, float fStickout, float fTargetPos[9][NUMBER_OF_JOINT]);
        
        //brief(290) :set_welding_cockpit_setting
        DRFL_API bool _set_welding_cockpit_setting(LPROBOTCONTROL pCtrl, unsigned char bEnable, unsigned char bWeldingType);
        //brief(291) :set_digital_welding_signal_output
        DRFL_API bool _set_digital_welding_signal_output(LPROBOTCONTROL pCtrl, unsigned char cDataType, float fData);
        //brief(292) :set_digital_welding_monitoring_mode
        DRFL_API bool _set_digital_welding_monitoring_mode(LPROBOTCONTROL pCtrl, unsigned char bEnable);
        //brief(293) :app_weld_adj_motion_offset
        DRFL_API bool _app_weld_adj_motion_offset(LPROBOTCONTROL pCtrl, float fOffsetY, float fOffsetZ);
        //brief(294) :set_welding_cockpit_setting_time_setting
        DRFL_API void _set_welding_cockpit_setting_time_setting(LPROBOTCONTROL pCtrl, int time);
        //LED custom setting
        DRFL_API bool _state_led_reset(LPROBOTCONTROL _rbtCtrl);
        DRFL_API bool _set_state_led_off(LPROBOTCONTROL _rbtCtrl);
        DRFL_API bool _set_state_led_color(LPROBOTCONTROL _rbtCtrl, int red, int green, int blue);
        DRFL_API unsigned char _get_state_led_rule(LPROBOTCONTROL _rbtCtrl);

#ifdef __cplusplus
    };
#endif

#ifdef __cplusplus
    class CDRFLEx : public CDRFL
    {
    public:
        CDRFLEx() { _rbtCtrlUDP = _create_robot_control_udp(); }
        virtual ~CDRFLEx() { _destroy_robot_control_udp(_rbtCtrlUDP);   }
        // connection
        //for ROS org bool OpenConnection(string strIpAddr = "192.168.137.100") { return _OpenConnection(_rbtCtrl, strIpAddr.c_str()); };
        bool open_connection(string strIpAddr = "192.168.137.100", unsigned int usPort= 12345) { return _open_connection(_rbtCtrl, strIpAddr.c_str(), usPort); };
        bool close_connection() { return _close_connection(_rbtCtrl); };

        bool connect_rt_control(string strIpAddr = "192.168.137.100", unsigned int usPort= 12347) { return _connect_rt_control(_rbtCtrlUDP, strIpAddr.c_str(), usPort); };
        bool disconnect_rt_control() { return _disconnect_rt_control(_rbtCtrlUDP); };

        ////////////////////////////////////////////////////////////////////////////
        // RT Control                                                             //
        ////////////////////////////////////////////////////////////////////////////   

        string get_rt_control_output_version_list() { return _get_rt_control_output_version_list(_rbtCtrlUDP); };
        string get_rt_control_input_version_list() { return _get_rt_control_input_version_list(_rbtCtrlUDP); };
        string get_rt_control_input_data_list(string strVersion) { return _get_rt_control_input_data_list(_rbtCtrlUDP, strVersion.c_str()); };
        string get_rt_control_output_data_list(string strVersion) { return _get_rt_control_output_data_list(_rbtCtrlUDP, strVersion.c_str()); };
        bool set_rt_control_input(string strVersion, float fPeriod, int nLossCnt){ return _set_rt_control_input(_rbtCtrlUDP, strVersion.c_str(), fPeriod, nLossCnt); };
        bool set_rt_control_output(string strVersion, float fPeriod, int nLossCnt){ return _set_rt_control_output(_rbtCtrlUDP, strVersion.c_str(), fPeriod, nLossCnt); };

        bool start_rt_control(){ return _start_rt_control(_rbtCtrlUDP); };
        bool stop_rt_control(){ return _stop_rt_control(_rbtCtrlUDP); };
        
        bool set_velj_rt(float vel[NUM_JOINT]){ return _set_velj_rt(_rbtCtrlUDP, vel); };
        bool set_accj_rt(float acc[NUM_JOINT]){ return _set_accj_rt(_rbtCtrlUDP, acc); };
        bool set_velx_rt(float fTransVel, float fRotationVel = -10000){ return _set_velx_rt(_rbtCtrlUDP, fTransVel, fRotationVel); };
        bool set_accx_rt(float fTransAcc, float fRotationAcc = -10000){ return _set_accx_rt(_rbtCtrlUDP, fTransAcc, fRotationAcc); };

        LPRT_OUTPUT_DATA_LIST read_data_rt(){ return _read_data_rt(_rbtCtrlUDP); };
        bool write_data_rt(float fExternalForceTorque[NUM_JOINT], int iExternalDI, int iExternalDO, float fExternalAnalogInput[6], float fExternalAnalogOutput[6]){ return _write_data_rt(_rbtCtrlUDP, fExternalForceTorque, iExternalDI, iExternalDO, fExternalAnalogInput, fExternalAnalogOutput); };
        
        bool servoj_rt(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime){ return _servoj_rt(_rbtCtrlUDP, fTargetPos, fTargetVel, fTargetAcc, fTargetTime); };
        bool servol_rt(float fTargetPos[NUM_TASK], float fTargetVel[NUM_TASK], float fTargetAcc[NUM_TASK], float fTargetTime){ return _servol_rt(_rbtCtrlUDP, fTargetPos, fTargetVel, fTargetAcc, fTargetTime); };
        bool speedj_rt(float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime){ return _speedj_rt(_rbtCtrlUDP, fTargetVel, fTargetAcc, fTargetTime); };
        bool speedl_rt(float fTargetVel[NUM_TASK], float fTargetAcc[NUM_TASK], float fTargetTime){ return _speedl_rt(_rbtCtrlUDP, fTargetVel, fTargetAcc, fTargetTime); };
        bool torque_rt(float fMotorTor[NUM_JOINT], float fTargetTime){ return _torque_rt(_rbtCtrlUDP, fMotorTor, fTargetTime); };
        //bool change_operation_speed_rt(float fSpeedRate){ return _change_operation_speed_rt(_rbtCtrlUDP, fSpeedRate); }; //차후 개발

        ////////////////////////////////////////////////////////////////////////////
        // Callback operation                                                    //
        ////////////////////////////////////////////////////////////////////////////
        // robot status data
        void set_on_monitoring_state(TOnMonitoringStateCB pCallbackFunc) { _set_on_monitoring_state(_rbtCtrl, pCallbackFunc); };
        /*Depreciated version. callbacks invoked after setup_monitoring_version(0)*/
        void set_on_monitoring_data(TOnMonitoringDataCB pCallbackFunc) { _set_on_monitoring_data(_rbtCtrl, pCallbackFunc); };
        /*Latest version. callbacks invoked after setup_monitoring_version(1)*/
        void set_on_monitoring_data_ex(TOnMonitoringDataExCB pCallbackFunc) { _set_on_monitoring_data_ex(_rbtCtrl, pCallbackFunc); };
        /*Depreciated version. callbacks invoked after setup_monitoring_version(0)*/
        void set_on_monitoring_ctrl_io(TOnMonitoringCtrlIOCB pCallbackFunc) { _set_on_monitoring_ctrl_io(_rbtCtrl, pCallbackFunc); };
        // ctrl-box I/O data
#if DRCF_VERSION == 2
        /*Latest version. callbacks invoked after setup_monitoring_version(1)*/
        void set_on_monitoring_ctrl_io_ex(TOnMonitoringCtrlIOExCB pCallbackFunc) { _set_on_monitoring_ctrl_io_ex(_rbtCtrl, pCallbackFunc); };
        void set_on_monitoring_welding_data(TOnMonitoringWeldingDataCB pCallbackFunc) { _set_on_monitoring_welding_data(_rbtCtrl, pCallbackFunc);};
        void set_on_monitoring_analog_welding_data(TOnMonitoringAnalogWeldingDataCB pCallbackFunc) { _set_on_monitoring_analog_welding_data(_rbtCtrl, pCallbackFunc);};
        void set_on_monitoring_digital_welding_data(TOnMonitoringDigitalWeldingDataCB pCallbackFunc) { _set_on_monitoring_digital_welding_data(_rbtCtrl, pCallbackFunc);};
#elif DRCF_VERSION == 3
        /*Latest version. callbacks invoked after setup_monitoring_version(1)*/
        void set_on_monitoring_ctrl_io_ex(TOnMonitoringCtrlIOEx2CB pCallbackFunc) { _set_on_monitoring_ctrl_io_ex2(_rbtCtrl, pCallbackFunc); };
#endif

        // modbus I/O data
        void set_on_monitoring_modbus(TOnMonitoringModbusCB pCallbackFunc) { _set_on_monitoring_modbus(_rbtCtrl, pCallbackFunc); };
        // robot speed mode event
        void set_on_monitoring_speed_mode(TOnMonitoringSpeedModeCB pCallbackFunc) { _set_on_monitoring_speed_mode(_rbtCtrl, pCallbackFunc); };
        // robot access control event
        void set_on_monitoring_access_control(TOnMonitoringAccessControlCB pCallbackFunc) { _set_on_monitoring_access_control(_rbtCtrl, pCallbackFunc); };
        // roobt alaram data
        void set_on_log_alarm(TOnLogAlarmCB pCallbackFunc)  { _set_on_log_alarm(_rbtCtrl, pCallbackFunc); };
        // tp popup message data
        void set_on_tp_popup(TOnTpPopupCB pCallbackFunc) { _set_on_tp_popup(_rbtCtrl, pCallbackFunc); };
        // tp log message data
        void set_on_tp_log(TOnTpLogCB pCallbackFunc) { _set_on_tp_log(_rbtCtrl, pCallbackFunc); };
        // tp progress message data
        void set_on_tp_progress(TOnTpProgressCB pCallbackFunc) { _set_on_tp_progress(_rbtCtrl, pCallbackFunc); };
        // tp user input message data
        void set_on_tp_get_user_input(TOnTpGetUserInputCB pCallbackFunc){ _set_on_tp_get_user_input(_rbtCtrl, pCallbackFunc); };
        // robot homing completed event
        void set_on_homming_completed(TOnHommingCompletedCB pCallbackFunc) { _set_on_homming_completed(_rbtCtrl, pCallbackFunc); };
        // Tp Initailzing completed
        void set_on_tp_initializing_completed(TOnTpInitializingCompletedCB pCallbackFunc) { _set_on_tp_initializing_completed(_rbtCtrl, pCallbackFunc); };
        // robot mastering needed event
        void set_on_mastering_need(TOnMasteringNeedCB pCallbackFunc) { _set_on_mastering_need(_rbtCtrl, pCallbackFunc); };
        // program stopeed event
        void set_on_program_stopped(TOnProgramStoppedCB pCallbackFunc) { _set_on_program_stopped(_rbtCtrl, pCallbackFunc); };
        // robot disconneted event
        void set_on_disconnected(TOnDisconnectedCB pCallbackFunc) { _set_on_disconnected(_rbtCtrl, pCallbackFunc); };    

        void set_on_monitoring_safety_state(TOnMonitoringSafetyStateCB pCallbackFunc) { _set_on_monitoring_safety_state(_rbtCtrl, pCallbackFunc); };
        void set_on_monitoring_robot_system(TOnMonitoringRobotSystemCB pCallbackFunc) { _set_on_monitoring_robot_system(_rbtCtrl, pCallbackFunc); };
        //void set_on_monitoring_update_module(TOnMonitoringUpdateModuleCB pCallbackFunc) { _set_on_monitoring_update_module(_rbtCtrl, pCallbackFunc); };
        void set_on_monitoring_safety_stop_type(TOnMonitoringSafetyStopTypeCB pCallbackFunc) { _set_on_monitoring_safety_stop_type(_rbtCtrl, pCallbackFunc); };

        void set_on_rt_monitoring_data(TOnRTMonitoringDataCB pCallbackFunc){ _set_on_rt_monitoring_data(_rbtCtrlUDP, pCallbackFunc); };
        void set_on_rt_log_alarm(TOnLogAlarmCB pCallbackFunc)  { _set_on_rt_log_alarm(_rbtCtrlUDP, pCallbackFunc); };

        LPROBOT_POSE trans(float fSourcePos[NUM_TASK], float fOffset[NUM_TASK], COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE, COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE){return _trans(_rbtCtrl, fSourcePos, fOffset, eSourceRef, eTargetRef);};
        LPROBOT_POSE ikin(float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE){return _ikin(_rbtCtrl, fSourcePos, iSolutionSpace, eTargetRef); };
        LPINVERSE_KINEMATIC_RESPONSE ikin(float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef, unsigned char iRefPosOpt){return _ikin_ex(_rbtCtrl, fSourcePos, iSolutionSpace, eTargetRef, iRefPosOpt); };
        LPINVERSE_KINEMATIC_RESPONSE ikin(float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef, float fIterThreshold[NUMBER_OF_ITER_THRESHOULD]){return _ikin_add_iter_threshold(_rbtCtrl, fSourcePos, iSolutionSpace, eTargetRef, fIterThreshold); };
        LPINVERSE_KINEMATIC_RESPONSE ikin_norm(float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef, unsigned char iRefPosOpt){return _ikin_norm(_rbtCtrl, fSourcePos, iSolutionSpace, eTargetRef, iRefPosOpt); };
		LPROBOT_POSE fkin(float fSourcePos[NUM_JOINT], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE){return _fkin(_rbtCtrl, fSourcePos, eTargetRef); };        
        LPROBOT_POSE addto(float fSourcePos[NUM_JOINT], float fOffset[NUM_JOINT]) { return _addto(_rbtCtrl, fSourcePos, fOffset); };

        unsigned char get_solution_space(float fTargetPos[NUM_JOINT]){ return _get_solution_space(_rbtCtrl, fTargetPos);};
        LPROBOT_TASK_POSE get_current_posx(COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE){return _get_current_posx(_rbtCtrl, eCoodType); };
        LPROBOT_POSE get_desired_posx(COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE){return _get_desired_posx(_rbtCtrl, eCoodType);};
        float get_orientation_error(float fPosition1[NUM_TASK], float fPosition2[NUM_TASK], TASK_AXIS eTaskAxis){return _get_orientation_error(_rbtCtrl, fPosition1, fPosition2, eTaskAxis); };
        bool set_workpiece_weight(float fWeight = 0.0, float fCog[3] = COG_DEFAULT, COG_REFERENCE eCogRef = COG_REFERENCE_TCP, ADD_UP eAddUp = ADD_UP_REPLACE, float fStartTime = -10000, float fTransitionTIme = -10000){return _set_workpiece_weight(_rbtCtrl, fWeight, fCog, eCogRef, eAddUp, fStartTime, fTransitionTIme); };
        float get_workpiece_weight(){return _get_workpiece_weight(_rbtCtrl);};
        bool reset_workpiece_weight(){return _reset_workpiece_weight(_rbtCtrl);};
        bool tp_popup_response(POPUP_RESPONSE eRes){return _tp_popup_response(_rbtCtrl, eRes);};
        bool tp_get_user_input_response(string strUserInput){return _tp_get_user_input_response(_rbtCtrl, strUserInput.c_str());};
      
        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////
        // get verion string
        bool get_system_version(LPSYSTEM_VERSION pVersion) { return _get_system_version(_rbtCtrl, pVersion); };
        const char* get_library_version() { return _get_library_version(_rbtCtrl); };

        // get robot safety mode(manual, auto)
        ROBOT_MODE get_robot_mode() { return _get_robot_mode(_rbtCtrl); };
        // set robot mode mode(manual, auto)
        bool set_robot_mode(ROBOT_MODE eMode) { return _set_robot_mode(_rbtCtrl, eMode); };

        // get robot state( initial, standby, moving, safe-off, teach, ...) 
        ROBOT_STATE get_robot_state() { return _get_robot_state(_rbtCtrl); };
        // set robot state 
        bool set_robot_control(ROBOT_CONTROL eControl) { return _set_robot_control(_rbtCtrl, eControl); };
        CONTROL_MODE get_control_mode(){ return _get_control_mode(_rbtCtrl);};
        // get robot system(real robot, virtrul robot)
        ROBOT_SYSTEM get_robot_system() { return _get_robot_system(_rbtCtrl); };
        // set robot system(real robot, virtrul robot)
        bool set_robot_system(ROBOT_SYSTEM eRobotSystem) { return _set_robot_system(_rbtCtrl, eRobotSystem); };

        // set robot speed mode(noraml reduced)
        bool set_robot_speed_mode(SPEED_MODE eSpeedMode) { return _set_robot_speed_mode(_rbtCtrl, eSpeedMode); };
        // get robot speed mode(noraml reduced)
        SPEED_MODE get_robot_speed_mode() { return _get_robot_speed_mode(_rbtCtrl); };
        
        // get roobt axis data
        LPROBOT_POSE get_current_pose(ROBOT_SPACE eSpaceType = ROBOT_SPACE_JOINT) { return _get_current_pose(_rbtCtrl, eSpaceType); };
        float(* get_current_rotm(COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE))[3]{ return _get_current_rotm(_rbtCtrl, eTargetRef); };
        unsigned char get_current_solution_space() { return _get_current_solution_space(_rbtCtrl); };
        // get current joint position list
        LPROBOT_POSE get_current_posj() { return _get_current_posj(_rbtCtrl); };
        // get current control space
        ROBOT_SPACE get_control_space() { return _get_control_space(_rbtCtrl); };
        // get current joint velocity
        LPROBOT_VEL get_current_velj() { return _get_current_velj(_rbtCtrl); };
        // get target joint position
        LPROBOT_POSE get_desired_posj() { return _get_desired_posj(_rbtCtrl); };
        // get flange task position
        LPROBOT_POSE get_current_tool_flange_posx() { return _get_current_tool_flange_posx(_rbtCtrl); };
        // get current task velocity
        LPROBOT_VEL get_current_velx() { return _get_current_velx(_rbtCtrl); };
        // get target task position
        //LPROBOT_POSE get_desired_posx() { return _GetDesiredPosX(_rbtCtrl); };
        // get target task velocity
        LPROBOT_VEL get_desired_velx() { return _get_desired_velx(_rbtCtrl); }; 
        // get current joint sensor torque value
        LPROBOT_FORCE get_joint_torque() { return _get_joint_torque(_rbtCtrl); };
        // get current external force
        LPROBOT_FORCE get_external_torque() { return _get_external_torque(_rbtCtrl); };
        // get current external force in tool
        LPROBOT_FORCE get_tool_force(COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE) { return _get_tool_force(_rbtCtrl, eTargetRef); };

        // get program running state
        DRL_PROGRAM_STATE get_program_state() { return _get_program_state(_rbtCtrl); };

        // set safe-stop reset type
        bool set_safe_stop_reset_type(SAFE_STOP_RESET_TYPE eResetType = SAFE_STOP_RESET_TYPE_DEFAULT) { return _set_safe_stop_reset_type(_rbtCtrl, eResetType); }

        // get roobot system alarm
        LPLOG_ALARM get_last_alarm() { return _get_last_alarm(_rbtCtrl); };
        
        ////////////////////////////////////////////////////////////////////////////
        //  access control                                                        //
        ////////////////////////////////////////////////////////////////////////////

        // manage access control
        bool manage_access_control(MANAGE_ACCESS_CONTROL eAccessControl = MANAGE_ACCESS_CONTROL_REQUEST) { return _manage_access_control(_rbtCtrl, eAccessControl); };
        
        ////////////////////////////////////////////////////////////////////////////
        //  motion Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////
        // basic control(hold to run)
        bool jog(JOG_AXIS eJogAxis, MOVE_REFERENCE eMoveReference, float fVelocity) { return _jog(_rbtCtrl, eJogAxis, eMoveReference, fVelocity); };
        bool multi_jog(float fTargetPos[NUM_TASK], MOVE_REFERENCE eMoveReference, float fVelocity) { return _multi_jog(_rbtCtrl, fTargetPos, eMoveReference, fVelocity); };
        bool move_home(MOVE_HOME eMode = MOVE_HOME_MECHANIC, unsigned char bRun = (unsigned)1) { return _move_home(_rbtCtrl, eMode, bRun); };
        LPROBOT_POSE get_user_home(){ return _get_user_home(_rbtCtrl); };
        // motion control: move stop
        bool stop(STOP_TYPE eStopType = STOP_TYPE_QUICK) { return _stop(_rbtCtrl, eStopType); };
        // motion control: move pause
        bool move_pause() { return _move_pause(_rbtCtrl); };
        // motion control: move resume
        bool move_resume() { return _move_resume(_rbtCtrl); };
        // wait motion
        bool mwait() { return _mwait(_rbtCtrl); };

        
        // motion control: joint move
        bool movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movej(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, fBlendingRadius, eBlendingType); };
        bool movej(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movej_ex(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, fBlendingRadius, eBlendingType); };
        bool amovej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _amovej(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eBlendingType); };
        bool amovej(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _amovej_ex(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eBlendingType); };
        // motion control: linear move
        bool movel(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, DR_MV_APP eAppType = DR_MV_APP_NONE) { return _movel(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType, eAppType); }
        bool amovel(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, DR_MV_APP eAppType = DR_MV_APP_NONE) { return _amovel(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eBlendingType, eAppType); }
        // motion control: circle move
#if DRCF_VERSION == 2
        bool movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, MOVE_ORIENTATION eOrientation = DR_MV_ORI_TEACH, DR_MV_APP eAppType = DR_MV_APP_NONE) { return _movec_ex(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fTargetAngle1, fTargetAngle2, fBlendingRadius, eBlendingType, eOrientation, eAppType); };
        bool amovec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, MOVE_ORIENTATION eOrientation = DR_MV_ORI_TEACH, DR_MV_APP eAppType = DR_MV_APP_NONE) { return _amovec_ex(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fTargetAngle1, fTargetAngle2, eBlendingType, eOrientation, eAppType); };
#elif DRCF_VERSION == 3        
        bool movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movec(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fTargetAngle1, fTargetAngle2, fBlendingRadius, eBlendingType); };
        bool amovec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _amovec(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fTargetAngle1, fTargetAngle2, eBlendingType); };
#endif
        // motion control: blending move
        bool moveb(MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, DR_MV_APP eAppType = DR_MV_APP_NONE) { return _moveb(_rbtCtrl, tTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eAppType); };
        bool amoveb(MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, DR_MV_APP eAppType = DR_MV_APP_NONE) { return _amoveb(_rbtCtrl, tTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eAppType); };
        // motion control: joint move as task information
        bool movejx(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movejx(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); };
        bool movejx(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movejx_ex(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); };
        bool amovejx(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _amovejx(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eBlendingType); };
        bool amovejx(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _amovejx_ex(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eBlendingType); };
        // spline motion as joint information
        bool movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _movesj(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
        bool movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel[NUMBER_OF_JOINT], float fTargetAcc[NUMBER_OF_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _movesj_ex(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
        bool amovesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _amovesj(_rbtCtrl, fTargetPos ,nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
        bool amovesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel[NUMBER_OF_JOINT], float fTargetAcc[NUMBER_OF_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _amovesj_ex(_rbtCtrl, fTargetPos ,nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
        // motion control: spline motin as task information
        bool movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT) { return _movesx(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eVelOpt); };
        bool amovesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT) { return _amovesx(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eVelOpt); };
        // motion control: move spiral motion
        bool move_spiral(TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _move_spiral(_rbtCtrl, eTaskAxis, fRevolution, fMaximuRadius, fMaximumLength, fTargetVel, fTargetAcc, fTargetTime, eMoveReference); };
        bool move_spiral(TASK_AXIS eTaskAxis, float fRevolution, float fTargetPos[3], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, SPIRAL_DIR eSpiralDir = DR_SPIRAL_OUTWARD, ROT_DIR eRotDir = DR_ROT_FORWARD) { return _move_spiral_ex(_rbtCtrl, eTaskAxis, fRevolution, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveReference, eMoveMode, eSpiralDir, eRotDir); };
        bool amove_spiral(TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _amove_spiral(_rbtCtrl, eTaskAxis, fRevolution, fMaximuRadius, fMaximumLength, fTargetVel, fTargetAcc, fTargetTime, eMoveReference); };
        bool amove_spiral(TASK_AXIS eTaskAxis, float fRevolution, float fTargetPos[3], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, SPIRAL_DIR eSpiralDir = DR_SPIRAL_OUTWARD, ROT_DIR eRotDir = DR_ROT_FORWARD) { return _amove_spiral_ex(_rbtCtrl, eTaskAxis, fRevolution, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveReference, eMoveMode, eSpiralDir, eRotDir); };
        // motion control: move periodic motion
        bool move_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned int nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _move_periodic(_rbtCtrl, fAmplitude, fPeriodic, fAccelTime, nRepeat, eMoveReference); };
        bool amove_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned int nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _amove_periodic(_rbtCtrl, fAmplitude, fPeriodic, fAccelTime, nRepeat, eMoveReference); };
		
        // environment adaptive motion
        bool servoj(float fTargetPos[NUM_JOINT], float fLimitVel[NUM_JOINT], float fLimitAcc[NUM_JOINT], float fTargetTime, DR_SERVOJ_TYPE eTargetMod = DR_SERVO_OVERRIDE){ return _servoj(_rbtCtrl, fTargetPos, fLimitVel, fLimitAcc, fTargetTime, eTargetMod); };
        bool servol(float fTargetPos[NUM_TASK], float fLimitVel[2], float fLimitAcc[2], float fTargetTime){ return _servol(_rbtCtrl, fTargetPos, fLimitVel, fLimitAcc, fTargetTime); };

        bool speedj(float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime){ return _speedj(_rbtCtrl, fTargetVel, fTargetAcc, fTargetTime); };
        bool speedl(float fTargetVel[NUM_TASK], float fTargetAcc[2], float fTargetTime){ return _speedl(_rbtCtrl, fTargetVel, fTargetAcc, fTargetTime); };

        bool servoj_g(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime=0.f){ return _servoj_g(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime); };
        bool servol_g(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime=0.f){ return _servol_g(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime); };

        bool movesx_g(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT) { return _movesx_g(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eVelOpt); };
        bool movesj_g(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _movesj_g(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };



        ////////////////////////////////////////////////////////////////////////////
        //  GPIO Operations                                                       //
        ////////////////////////////////////////////////////////////////////////////
        // set digital output on flange
        bool set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool bOnOff) { return _set_tool_digital_output(_rbtCtrl, eGpioIndex, bOnOff); };
        // get digital input on flange
        bool get_tool_digital_input(GPIO_TOOL_DIGITAL_INDEX eGpioIndex) { return _get_tool_digital_input(_rbtCtrl, eGpioIndex); };
        bool get_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX eGpioIndex) { return _get_tool_digital_output(_rbtCtrl, eGpioIndex); };
#if DRCF_VERSION == 2
        bool get_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex) { return _get_digital_output(_rbtCtrl, eGpioIndex); };
        bool get_digital_input(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex) { return _get_digital_input(_rbtCtrl, eGpioIndex); };
        bool set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool bOnOff) { return _set_digital_output(_rbtCtrl, eGpioIndex, bOnOff); };
        float get_analog_input(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex) { return _get_analog_input(_rbtCtrl, eGpioIndex); };
#elif DRCF_VERSION == 3
        bool get_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex) { return _get_digital_output_ex(_rbtCtrl, eGpioIndex); };
        bool get_digital_input(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex) { return _get_digital_input_ex(_rbtCtrl, eGpioIndex); };
        bool set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool bOnOff) { return _set_digital_output_ex(_rbtCtrl, eGpioIndex, bOnOff); };
        float get_analog_input(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex) { return _get_analog_input_ex(_rbtCtrl, eGpioIndex); };
#endif
        bool set_analog_output(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float fValue) { return _set_analog_output(_rbtCtrl, eGpioIndex, fValue); };

        // set analog input type on control-box
        bool set_mode_analog_input(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT) { return _set_mode_analog_input(_rbtCtrl, eGpioIndex, eAnalogType); }; 
        // set analog output type on control-box
        bool set_mode_analog_output(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT) { return _set_mode_analog_output(_rbtCtrl, eGpioIndex, eAnalogType); };

        float get_tool_analog_input(int nCh){ return _get_tool_analog_input(_rbtCtrl, nCh); };
        bool set_tool_digital_output_level(int nLv){ return _set_tool_digital_output_level(_rbtCtrl, nLv); };
        bool set_tool_digital_output_type(int nPort, OUTPUT_TYPE eOutputType){ return _set_tool_digital_output_type(_rbtCtrl, nPort, eOutputType); };
        bool set_mode_tool_analog_input(int nCh, GPIO_ANALOG_TYPE eAnalogType){ return _set_mode_tool_analog_input(_rbtCtrl, nCh, eAnalogType); };
        

        ////////////////////////////////////////////////////////////////////////////
        //  Modbus Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////
        // set modbus register
        bool set_modbus_output(string strSymbol, unsigned short nValue) { return _set_modbus_output(_rbtCtrl, strSymbol.c_str(), nValue); };
        // get modbus register
        unsigned short get_modbus_input(string strSymbol) { return _get_modbus_input(_rbtCtrl, strSymbol.c_str()); };
        // add modbus register
        bool add_modbus_signal(string strSymbol, string strIpAddress, unsigned short nPort, MODBUS_REGISTER_TYPE eRegType, unsigned short iRegIndex, unsigned short nRegValue = 0, unsigned char nSlaiveId = 255) { return _add_modbus_signal(_rbtCtrl, strSymbol.c_str(), strIpAddress.c_str(), nPort, eRegType, iRegIndex, nRegValue, nSlaiveId); };
        // del modbus register
        bool del_modbus_signal(string strSymbol) { return _del_modbus_signal(_rbtCtrl, strSymbol.c_str()); };
        // get all of the modbus data list
        LPMODBUS_DATA_LIST query_modbus_data_list() { return _query_modbus_data_list(_rbtCtrl); };

        ////////////////////////////////////////////////////////////////////////////
        //  Flange Serial Operations                                              //
        ////////////////////////////////////////////////////////////////////////////
        bool flange_serial_open(int nPort, int baudrate = 115200, BYTE_SIZE eByteSize = BYTE_SIZE_EIGHTBITS, PARITY_CHECK eParity = PARITY_CHECK_NONE, STOP_BITS eStopBits = STOPBITS_ONE){ return _flange_serial_open(_rbtCtrl, nPort, baudrate, eByteSize, eParity, eStopBits); };
        bool flange_serial_close(int nPort){ return _flange_serial_close(_rbtCtrl, nPort); };
        bool flange_serial_write(int nSize, char* pSendData, int nPort = 1){ return _flange_serial_write(_rbtCtrl, nSize, pSendData, nPort); };
        LPFLANGE_SER_RXD_INFO_EX flange_serial_read(float fTimeout = -1, int nPort = 1){ return _flange_serial_read(_rbtCtrl, fTimeout, nPort); };      

        ////////////////////////////////////////////////////////////////////////////
        //  Configuration Operations                                               //
        ////////////////////////////////////////////////////////////////////////////
        // set tool(end-effector) information
        bool set_tool(string strSymbol) { return _set_tool(_rbtCtrl, strSymbol.c_str()); };
        // get tool(end-effector) information
        string get_tool() { return string(_get_tool(_rbtCtrl)); };
        // add tool(end-effector) information
        bool add_tool(string strSymbol, float fWeight, float fCog[3], float fInertia[NUM_TASK]) { return _add_tool(_rbtCtrl, strSymbol.c_str(), fWeight, fCog, fInertia); };
        // del tool(end-effector) informaiton
        bool del_tool(string strSymbol) { return _del_tool(_rbtCtrl, strSymbol.c_str()); };
        
        // set robot tcp information
        bool set_tcp(string strSymbol) { return _set_tcp(_rbtCtrl, strSymbol.c_str()); };
        // get robot tcp information
        string get_tcp() { return string(_get_tcp(_rbtCtrl)); };  
        // add robot tcp information
        bool add_tcp(string strSymbol, float fPostion[NUM_TASK]) { return _add_tcp(_rbtCtrl, strSymbol.c_str(), fPostion); };
        // del robot tcp information
        bool del_tcp(string strSymbol) { return _del_tcp(_rbtCtrl, strSymbol.c_str()); };
        
        // set robot tool shape information
        bool set_tool_shape(string strSymbol){return _set_tool_shape(_rbtCtrl, strSymbol.c_str());};
        // get robot tool shape information 
        string get_tool_shape(){ return _get_tool_shape(_rbtCtrl);};

        bool set_user_home(){ return _set_user_home(_rbtCtrl); };

        int servo_off(STOP_TYPE eStopType) { return _servo_off(_rbtCtrl, eStopType); };
        bool release_protective_stop(RELEASE_MODE eReleaseMode){ return _release_protective_stop(_rbtCtrl, eReleaseMode); };


        bool set_safety_mode(SAFETY_MODE eSafetyMode, SAFETY_MODE_EVENT eSafetyEvent){ return _set_safety_mode(_rbtCtrl, eSafetyMode, eSafetyEvent); };
        bool set_auto_servo_off(bool bFuncEnable, float fElapseTime){ return _set_auto_servo_off(_rbtCtrl, bFuncEnable, fElapseTime); };
        bool change_collision_sensitivity(float fSensitivity){ return _change_collision_sensitivity(_rbtCtrl, fSensitivity); };
        bool set_palletizing_mode(unsigned char iMode) { return _set_palletizing_mode(_rbtCtrl, iMode); };

        // LPSAFETY_CONFIGURATION_EX get_safety_configuration(){ return _get_safety_configuration(_rbtCtrl); };
        #if DRCF_VERSION == 2

        LPSAFETY_CONFIGURATION_EX2 get_safety_configuration(){ return _get_safety_configuration_ex(_rbtCtrl); };
        #elif DRCF_VERSION == 3

        LPSAFETY_CONFIGURATION_EX2_V3 get_safety_configuration(){ return _get_safety_configuration_ex_v3(_rbtCtrl); };
        #endif

        int check_motion() {return _check_motion(_rbtCtrl);};
        
        bool set_auto_safety_move_stop(bool bFuncEnable) {return _set_auto_safety_move_stop(_rbtCtrl, bFuncEnable); };
        ////////////////////////////////////////////////////////////////////////////
        //  drl program Operations                                                //
        ////////////////////////////////////////////////////////////////////////////
        //program start
        bool drl_start(ROBOT_SYSTEM eRobotSystem, string strDrlProgram) { return _drl_start(_rbtCtrl, eRobotSystem, strDrlProgram.c_str()); };
        //program stop
        bool drl_stop(unsigned char eStopType = 0) { return _drl_stop(_rbtCtrl, eStopType); };
        //program pause
        bool drl_pause() { return _drl_pause(_rbtCtrl); };
        //program resume
        bool drl_resume() { return _drl_resume(_rbtCtrl); };
        bool change_operation_speed(float fSpeed) { return _change_operation_speed(_rbtCtrl, fSpeed); };

        ////////////////////////////////////////////////////////////////////////////
        //  force control                                                        //
        ////////////////////////////////////////////////////////////////////////////

        bool task_compliance_ctrl(float fTargetStiffness[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f) { return _task_compliance_ctrl(_rbtCtrl, fTargetStiffness, eForceReference, fTargetTime); };
        //bool EnterJointCompliance(float fTargetStiffness[NUM_TASK], float fTargetTime = 0.f){ return _EnterJointCompliance(_rbtCtrl, fTargetStiffness, fTargetTime);};
        bool set_stiffnessx(float fTargetStiffness[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f) { return _set_stiffnessx(_rbtCtrl, fTargetStiffness, eForceReference, fTargetTime); };
        bool release_compliance_ctrl() { return _release_compliance_ctrl(_rbtCtrl); };
        //bool LeaveJointCompliance() { return _LeaveJointCompliance(_rbtCtrl);};
        bool set_desired_force(float fTargetForce[NUM_TASK], unsigned char iTargetDirection[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f, FORCE_MODE eForceMode = FORCE_MODE_ABSOLUTE) { return _set_desired_force(_rbtCtrl, fTargetForce, iTargetDirection, eForceReference, fTargetTime, eForceMode); };
        bool release_force(float fTargetTime = 0.f) { return _release_force(_rbtCtrl, fTargetTime); };

        bool check_force_condition(FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL) { return _check_force_condition(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, eForceReference); };
        bool check_position_condition_abs(FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL) { return _check_position_condition_abs(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, eForceReference); };
        bool check_position_condition_rel(FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL) { return _check_position_condition_rel(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, fTargetPos, eForceReference); };
        bool check_position_condition(FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], MOVE_MODE eMode = MOVE_MODE_ABSOLUTE, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL) { return _check_position_condition(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, fTargetPos, eMode, eForceReference); };

        bool check_orientation_condition(FORCE_AXIS eForceAxis, float fTargetMin[NUM_TASK], float fTargetMax[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL) { return _check_orientation_condition_abs(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, eForceReference); };
        bool check_orientation_condition(FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL) { return _check_orientation_condition_rel(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, fTargetPos, eForceReference); };
        bool is_done_bolt_tightening(FORCE_AXIS eForceAxis, float fTargetTor = 0.f, float fTimeout = 0.f) { return _is_done_bolt_tightening(_rbtCtrl, eForceAxis, fTargetTor, fTimeout); };
        bool parallel_axis(float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE) { return _parallel_axis1(_rbtCtrl, fTargetPos1, fTargetPos2, fTargetPos3, eTaskAxis, eSourceRef); };
        bool align_axis(float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE) { return _align_axis1(_rbtCtrl, fTargetPos1, fTargetPos2, fTargetPos3, fSourceVec, eTaskAxis, eSourceRef); };
        bool parallel_axis(float fTargetVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef) { return _parallel_axis2(_rbtCtrl, fTargetVec, eTaskAxis, eSourceRef); };
        bool align_axis(float fTargetVec[3], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef) { return _align_axis2(_rbtCtrl, fTargetVec, fSourceVec, eTaskAxis, eSourceRef); };

        ////////////////////////////////////////////////////////////////////////////
        //  coordinate system control                                             //
        ////////////////////////////////////////////////////////////////////////////

        int set_user_cart_coord(int iReqId, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE) { return _set_user_cart_coord1(_rbtCtrl, iReqId, fTargetPos, eTargetRef); };
        int set_user_cart_coord(float fTargetPos[3][NUM_TASK], float fTargetOrg[3], COORDINATE_SYSTEM fTargetRef = COORDINATE_SYSTEM_BASE) { return _set_user_cart_coord2(_rbtCtrl, fTargetPos, fTargetOrg, fTargetRef); };
        int set_user_cart_coord(float fTargetVec[2][3], float fTargetOrg[3], COORDINATE_SYSTEM fTargetRef = COORDINATE_SYSTEM_BASE) { return _set_user_cart_coord3(_rbtCtrl, fTargetVec, fTargetOrg, fTargetRef); };
        LPROBOT_POSE coord_transform(float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eInCoordSystem, COORDINATE_SYSTEM eOutCoordSystem) { return _coord_transform(_rbtCtrl, fTargetPos, eInCoordSystem, eOutCoordSystem); };
        bool set_ref_coord(COORDINATE_SYSTEM eTargetCoordSystem) { return _set_ref_coord(_rbtCtrl, eTargetCoordSystem); };
        LPROBOT_POSE calc_coord(unsigned short nCnt, unsigned short nInputMode, COORDINATE_SYSTEM eTargetRef, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fTargetPos4[NUM_TASK]) { return _calc_coord(_rbtCtrl, nCnt, nInputMode, eTargetRef, fTargetPos1, fTargetPos2, fTargetPos3, fTargetPos4); };
        LPUSER_COORDINATE get_user_cart_coord(int iReqId) { return _get_user_cart_coord(_rbtCtrl, iReqId); };
        int overwrite_user_cart_coord(bool bTargetUpdate, int iReqId, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE) { return _overwrite_user_cart_coord(_rbtCtrl, bTargetUpdate, iReqId, fTargetPos, eTargetRef); };
        bool enable_alter_motion(int iCycleTime, PATH_MODE ePathMode, COORDINATE_SYSTEM eTargetRef, float fLimitDpos[2], float fLimitDposPer[2]) { return _enable_alter_motion(_rbtCtrl, iCycleTime, ePathMode, eTargetRef, fLimitDpos, fLimitDposPer); };
        bool disable_alter_motion() { return _disable_alter_motion(_rbtCtrl); };
        bool alter_motion(float fTargetPos[NUM_TASK]) { return _alter_motion(_rbtCtrl, fTargetPos); };
        bool set_singularity_handling(SINGULARITY_AVOIDANCE eMode) { return _set_singularity_handling(_rbtCtrl, eMode); };
        bool config_program_watch_variable(VARIABLE_TYPE eDivision, DATA_TYPE eType, string strName, string strData) { return _config_program_watch_variable(_rbtCtrl, eDivision, eType, strName.c_str(), strData.c_str()); };
        bool save_sub_program(int iTargetType, string strFileName, string strDrlProgram) { return _save_sub_program(_rbtCtrl, iTargetType, strFileName.c_str(), strDrlProgram.c_str()); };
        
        /*
            If specified 0, depreciated callbacks invoked (set_on_monitoring_data, set_on_monitoring_ctrl_io)
            If specified 1, latest callbacks invoked (set_on_monitoring_data_ex, set_on_monitoring_ctrl_io_ex)
        */
        bool setup_monitoring_version(int iVersion) { return _setup_monitoring_version(_rbtCtrl, iVersion); };
        bool system_shut_down() { return _system_shut_down(_rbtCtrl); };
        // LED
        bool state_led_reset() {return _state_led_reset(_rbtCtrl);};
        bool set_state_led_off() {return _set_state_led_off(_rbtCtrl);};
        bool set_state_led_color(int red, int green, int blue) {return _set_state_led_color(_rbtCtrl, red, green, blue);};
        unsigned char get_state_led_rule() {return _get_state_led_rule(_rbtCtrl);};

        ////////////////////////////////////////////////////////////////////////////
        //  welding                                                //
        ////////////////////////////////////////////////////////////////////////////
#if DRCF_VERSION == 2
        bool app_weld_weave_cond_trapezoidal(CONFIG_TRAPEZOID_WEAVING_SETTING pConfigtrapezoidweavingsetting) { return _app_weld_weave_cond_trapezoidal(_rbtCtrl, pConfigtrapezoidweavingsetting); };
        //brief(271) :app_weld_weave_cond_zigzag
        bool app_weld_weave_cond_zigzag(float fOffsetY, float fOffsetZ, float fGradient, float fWeavingWidth, float fWeavingCycle) { return _app_weld_weave_cond_zigzag(_rbtCtrl, fOffsetY, fOffsetZ, fGradient, fWeavingWidth, fWeavingCycle); };
        //brief(272) :app_weld_weave_cond_circular
        bool app_weld_weave_cond_circular(float fOffsetY, float fOffsetZ, float fGradient, float fwWdt[2], float fwT[2]) { return _app_weld_weave_cond_circular(_rbtCtrl, fOffsetY, fOffsetZ, fGradient, fwWdt, fwT); };
        //brief(273) :app_weld_weave_cond_sinusoidal
        bool app_weld_weave_cond_sinusoidal(float fOffsetY, float fOffsetZ, float fGradient, float fWeavingWidth, float fWeavingCycle) { return _app_weld_weave_cond_sinusoidal(_rbtCtrl, fOffsetY, fOffsetZ, fGradient, fWeavingWidth, fWeavingCycle); };
        //brief(274) :app_weld_enable_analog
        bool app_weld_enable_analog(CONFIG_ANALOG_WELDING_INTERFACE pConfiganalogweldinginterface) { return _app_weld_enable_analog(_rbtCtrl, pConfiganalogweldinginterface); };
        //brief(275) :app_weld_set_weld_cond_analog
        bool app_weld_set_weld_cond_analog(CONFIG_ANALOG_WELDING_SETTING pConfigAnalogWeldingSetting) { return _app_weld_set_weld_cond_analog(_rbtCtrl, pConfigAnalogWeldingSetting); };
        //brief(276) :app_weld_adj_welding_cond_analog
        bool app_weld_adj_welding_cond_analog(unsigned char bRealTime, unsigned char bResetFlag, float fTargetVol, float fFeedingVel, float fTargetVel, float fOffsetY, float fOffsetZ, float fWidthRate) { return _app_weld_adj_welding_cond_analog(_rbtCtrl, bRealTime, bResetFlag, fTargetVol, fFeedingVel, fTargetVel, fOffsetY, fOffsetZ, fWidthRate); };
        //brief(277) :app_weld_set_interface_eip_r2m_process
        bool app_weld_set_interface_eip_r2m_process(CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS pConfigdigitalweldinginterfaceprocess) { return _app_weld_set_interface_eip_r2m_process(_rbtCtrl, pConfigdigitalweldinginterfaceprocess); };
        //brief(278) :app_weld_set_interface_eip_r2m_mode
        bool app_weld_set_interface_eip_r2m_mode(CONFIG_DIGITAL_WELDING_INTERFACE_MODE pConfigdigitalweldinginterfacemode) { return _app_weld_set_interface_eip_r2m_mode(_rbtCtrl, pConfigdigitalweldinginterfacemode); };
        //brief(279) :app_weld_set_interface_eip_r2m_test
        bool app_weld_set_interface_eip_r2m_test(CONFIG_DIGITAL_WELDING_INTERFACE_TEST pConfigdigitalweldinginterfacetest) { return _app_weld_set_interface_eip_r2m_test(_rbtCtrl, pConfigdigitalweldinginterfacetest); };
        
        //brief(280) :app_weld_set_interface_eip_r2m_condition
        bool app_weld_set_interface_eip_r2m_condition(CONFIG_DIGITAL_WELDING_INTERFACE_CONDITION pConfigdigitalweldinginterfacecondition) { return _app_weld_set_interface_eip_r2m_condition(_rbtCtrl, pConfigdigitalweldinginterfacecondition); };
        //brief(281) :app_weld_set_interface_eip_r2m_option
        bool app_weld_set_interface_eip_r2m_option(CONFIG_DIGITAL_WELDING_INTERFACE_OPTION pConfigdigitalweldinginterfaceoption) { return _app_weld_set_interface_eip_r2m_option(_rbtCtrl, pConfigdigitalweldinginterfaceoption); };
        //brief(282) :app_weld_set_interface_eip_m2r_process2
        bool app_weld_set_interface_eip_m2r_process2(CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS2 pConfigdigitalweldinginterfaceprocess2) { return _app_weld_set_interface_eip_m2r_process2(_rbtCtrl, pConfigdigitalweldinginterfaceprocess2); };
        //brief(283) :app_weld_set_interface_eip_m2r_monitoring
        bool app_weld_set_interface_eip_m2r_monitoring(CONFIG_DIGITAL_WELDING_INTERFACE_MONITORING pConfigdigitalweldinginterfacemonitoring) { return _app_weld_set_interface_eip_m2r_monitoring(_rbtCtrl, pConfigdigitalweldinginterfacemonitoring); };
        //brief(284) :app_weld_set_interface_eip_m2r_other
        bool app_weld_set_interface_eip_m2r_other(CONFIG_DIGITAL_WELDING_INTERFACE_OTHER pConfigdigitalweldinginterfacemonitoring) { return _app_weld_set_interface_eip_m2r_other(_rbtCtrl, pConfigdigitalweldinginterfacemonitoring); };
        //brief(285) :app_weld_reset_interface
        bool app_weld_reset_interface(unsigned char bReset) { return _app_weld_reset_interface(_rbtCtrl, bReset); };
        //brief(286) :app_weld_enable_digital
        bool app_weld_enable_digital(unsigned char bMode) { return _app_weld_enable_digital(_rbtCtrl, bMode); };
        //brief(286) :app_weld_disable_digital
        bool app_weld_disable_digital(unsigned char bMode) { return _app_weld_disable_digital(_rbtCtrl, bMode); };
        //brief(287) :app_weld_set_weld_cond_digital
        bool app_weld_set_weld_cond_digital(CONFIG_DIGITAL_WELDING_CONDITION pConfigdigitalweldingcondition) { return _app_weld_set_weld_cond_digital(_rbtCtrl, pConfigdigitalweldingcondition); };
        //brief(288) :app_weld_adj_welding_cond_digital
        bool app_weld_adj_welding_cond_digital(CONFIG_DIGITAL_WELDING_ADJUST pConfigdigitalweldingadjust) { return _app_weld_adj_welding_cond_digital(_rbtCtrl, pConfigdigitalweldingadjust); };
        //brief(289) :measure_welding_tcp
        LPMEASURE_TCP_RESPONSE measure_welding_tcp(unsigned char iMode, float fStickout, float fTargetPos[9][NUMBER_OF_JOINT]) { return _measure_welding_tcp(_rbtCtrl, iMode, fStickout, fTargetPos); };
        //brief(290) :set_welding_cockpit_setting
        bool set_welding_cockpit_setting(unsigned char bEnable, unsigned char bWeldingType) { return _set_welding_cockpit_setting(_rbtCtrl, bEnable, bWeldingType); };
        //brief(291) :set_digital_welding_signal_output
        bool set_digital_welding_signal_output(unsigned char cDataType, float fData) { return _set_digital_welding_signal_output(_rbtCtrl, cDataType, fData); };
        //brief(292) :set_digital_welding_monitoring_mode
        bool set_digital_welding_monitoring_mode(unsigned char bEnable) { return _set_digital_welding_monitoring_mode(_rbtCtrl, bEnable); };
        //brief(293) :app_weld_adj_motion_offset
        bool app_weld_adj_motion_offset(float fOffsetY, float fOffsetZ) { return _app_weld_adj_motion_offset(_rbtCtrl, fOffsetY, fOffsetZ); };
        //brief(294) :set_welding_cockpit_setting_time_setting
        void set_welding_cockpit_setting_time_setting(int time) { return _set_welding_cockpit_setting_time_setting(_rbtCtrl, time); };

#endif

protected:
        LPROBOTCONTROL _rbtCtrlUDP;
    };
#endif
}

