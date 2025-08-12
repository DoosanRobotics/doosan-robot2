#-*- coding: utf-8 -*-

# ##
# @mainpage
# @file     DSR_ROBOT2.py
# @brief    Doosan Robotics ROS2 service I/F module
# @author   kabdol2<kabkyoum.kim@doosan.com>   
# @version  0.10
# @Last update date     2021-02-05
# @details
#
# history
#  -   
#
__ROS2__ = True

import rclpy
import os
import threading, time
from std_msgs.msg import String,Int32,Int32MultiArray,Float32,Float64,Float32MultiArray,Float64MultiArray,MultiArrayLayout,MultiArrayDimension
import sys
sys.dont_write_bytecode = True

#import numpy as np

from DRFC import *
from DR_common2 import *

from dsr_msgs2.msg import *
from dsr_msgs2.srv import *

#-----------------------------------------------------------------------------------
import DR_init
g_node = None
g_node = DR_init.__dsr__node

_robot_id    = DR_init.__dsr__id
_robot_model = DR_init.__dsr__model
_srv_name_prefix   = '' # _robot_id + '/' #ROS2  
_topic_name_prefix = _srv_name_prefix

print("_robot_id ={0}".format(_robot_id))
print("_robot_model ={0}".format(_robot_model))
print("_srv_name_prefix ={0}".format(_srv_name_prefix))
print("_topic_name_prefix ={0}".format(_topic_name_prefix))
#-----------------------------------------------------------------------------------


############### connect to dsr_control2 (ros2 service) ####################################################################### 

#  system Operations
_ros2_set_robot_mode             = g_node.create_client(SetRobotMode,           _srv_name_prefix +"system/set_robot_mode")
_ros2_get_robot_mode             = g_node.create_client(GetRobotMode,           _srv_name_prefix +"system/get_robot_mode")
_ros2_set_robot_system           = g_node.create_client(SetRobotSystem,         _srv_name_prefix +"system/set_robot_system")
_ros2_get_robot_system           = g_node.create_client(GetRobotSystem,         _srv_name_prefix +"system/get_robot_system")
_ros2_get_robot_state            = g_node.create_client(GetRobotState,          _srv_name_prefix +"system/get_robot_state")
_ros2_set_robot_speed_mode       = g_node.create_client(SetRobotSpeedMode,      _srv_name_prefix +"system/set_robot_speed_mode")
_ros2_get_robot_speed_mode       = g_node.create_client(GetRobotSpeedMode,      _srv_name_prefix +"system/get_robot_speed_mode")
_ros2_set_safe_stop_reset_type   = g_node.create_client(SetSafeStopResetType,   _srv_name_prefix +"system/set_safe_stop_reset_type")
_ros2_get_last_alarm             = g_node.create_client(GetLastAlarm,           _srv_name_prefix +"system/get_last_alarm")
_ros2_get_current_pose           = g_node.create_client(GetCurrentPose,         _srv_name_prefix +"system/get_current_pose")

#  motion Operations
_ros2_servoj_stream_pub          = g_node.create_publisher(ServojStream,        _topic_name_prefix + "servoj_stream", 10)
_ros2_servol_stream_pub          = g_node.create_publisher(ServolStream,        _topic_name_prefix + "servol_stream", 10)
_ros2_speedj_stream_pub          = g_node.create_publisher(SpeedjStream,        _topic_name_prefix + "speedj_stream", 10)
_ros2_speedl_stream_pub          = g_node.create_publisher(SpeedlStream,        _topic_name_prefix + "speedl_stream", 10)
_ros2_servoj_rt_stream_pub       = g_node.create_publisher(ServojRtStream,      _topic_name_prefix + "servoj_rt_stream", 10)
_ros2_servol_rt_stream_pub       = g_node.create_publisher(ServolRtStream,      _topic_name_prefix + "servol_rt_stream", 10)
_ros2_speedj_rt_stream_pub       = g_node.create_publisher(SpeedjRtStream,      _topic_name_prefix + "speedj_rt_stream", 10)
_ros2_speedl_rt_stream_pub       = g_node.create_publisher(SpeedlRtStream,      _topic_name_prefix + "speedl_rt_stream", 10)
_ros2_torque_rt_stream_pub       = g_node.create_publisher(TorqueRtStream,      _topic_name_prefix + "torque_rt_stream", 10)
_ros2_alter_motion_stream_pub    = g_node.create_publisher(AlterMotionStream,   _topic_name_prefix + "alter_motion_stream", 10)
_ros2_set_safety_mode            = g_node.create_client(SetSafetyMode,          _srv_name_prefix +"system/set_safety_mode")

_ros2_movej                      = g_node.create_client(MoveJoint,              _srv_name_prefix + "motion/move_joint")
_ros2_movel                      = g_node.create_client(MoveLine,               _srv_name_prefix +"motion/move_line")
_ros2_movejx                     = g_node.create_client(MoveJointx,             _srv_name_prefix +"motion/move_jointx")
_ros2_movec                      = g_node.create_client(MoveCircle,             _srv_name_prefix +"motion/move_circle")
_ros2_movesj                     = g_node.create_client(MoveSplineJoint,        _srv_name_prefix +"motion/move_spline_joint")
_ros2_movesx                     = g_node.create_client(MoveSplineTask,         _srv_name_prefix +"motion/move_spline_task")
_ros2_moveb                      = g_node.create_client(MoveBlending,           _srv_name_prefix +"motion/move_blending")
_ros2_move_spiral                = g_node.create_client(MoveSpiral,             _srv_name_prefix +"motion/move_spiral")
_ros2_move_periodic              = g_node.create_client(MovePeriodic,           _srv_name_prefix +"motion/move_periodic")
_ros2_move_wait                  = g_node.create_client(MoveWait,               _srv_name_prefix +"motion/move_wait")
_ros2_jog                        = g_node.create_client(Jog,                    _srv_name_prefix +"motion/jog")
_ros2_jog_multi                  = g_node.create_client(JogMulti,               _srv_name_prefix +"motion/jog_multi")
_ros2_trans                      = g_node.create_client(Trans,                  _srv_name_prefix +"motion/trans")
_ros2_fkin                       = g_node.create_client(Fkin,                   _srv_name_prefix +"motion/fkin")
_ros2_ikin                       = g_node.create_client(Ikin,                   _srv_name_prefix +"motion/ikin")
_ros2_set_ref_coord              = g_node.create_client(SetRefCoord,            _srv_name_prefix +"motion/set_ref_coord")
_ros2_move_home                  = g_node.create_client(MoveHome,               _srv_name_prefix +"motion/move_home")
_ros2_check_motion               = g_node.create_client(CheckMotion,            _srv_name_prefix +"motion/check_motion")
_ros2_change_operation_speed     = g_node.create_client(ChangeOperationSpeed,   _srv_name_prefix +"motion/change_operation_speed")
_ros2_enable_alter_motion        = g_node.create_client(EnableAlterMotion,      _srv_name_prefix +"motion/enable_alter_motion")
_ros2_alter_motion               = g_node.create_client(AlterMotion,            _srv_name_prefix +"motion/alter_motion")
_ros2_disable_alter_motion       = g_node.create_client(DisableAlterMotion,     _srv_name_prefix +"motion/disable_alter_motion")
_ros2_set_singularity_handling    = g_node.create_client(SetSingularityHandling,_srv_name_prefix +"motion/set_singularity_handling")


# Auxiliary Control Operations
_ros2_get_control_mode               = g_node.create_client(GetControlMode,     _srv_name_prefix +"aux_control/get_control_mode")
_ros2_get_control_space              = g_node.create_client(GetControlSpace,    _srv_name_prefix +"aux_control/get_control_space")
 
_ros2_get_current_posj               = g_node.create_client(GetCurrentPosj,     _srv_name_prefix +"aux_control/get_current_posj")
_ros2_get_current_velj               = g_node.create_client(GetCurrentVelj,     _srv_name_prefix +"aux_control/get_current_velj")
_ros2_get_desired_posj               = g_node.create_client(GetDesiredPosj,     _srv_name_prefix +"aux_control/get_desired_posj")
_ros2_get_desired_velj               = g_node.create_client(GetDesiredVelj,     _srv_name_prefix +"aux_control/get_desired_velj")
 
_ros2_get_current_posx               = g_node.create_client(GetCurrentPosx,     _srv_name_prefix +"aux_control/get_current_posx")
_ros2_get_current_velx               = g_node.create_client(GetCurrentVelx,     _srv_name_prefix +"aux_control/get_current_velx")    
_ros2_get_desired_posx               = g_node.create_client(GetDesiredPosx,     _srv_name_prefix +"aux_control/get_desired_posx")
_ros2_get_desired_velx               = g_node.create_client(GetDesiredVelx,     _srv_name_prefix +"aux_control/get_desired_velx")
 
_ros2_get_current_tool_flange_posx   = g_node.create_client(GetCurrentToolFlangePosx,   _srv_name_prefix +"aux_control/get_current_tool_flange_posx")
 
_ros2_get_current_solution_space     = g_node.create_client(GetCurrentSolutionSpace,    _srv_name_prefix +"aux_control/get_current_solution_space")
_ros2_get_current_rotm               = g_node.create_client(GetCurrentRotm,             _srv_name_prefix +"aux_control/get_current_rotm")    
_ros2_get_joint_torque               = g_node.create_client(GetJointTorque,             _srv_name_prefix +"aux_control/get_joint_torque")
_ros2_get_external_torque            = g_node.create_client(GetExternalTorque,          _srv_name_prefix +"aux_control/get_external_torque")
_ros2_get_tool_force                 = g_node.create_client(GetToolForce,               _srv_name_prefix +"aux_control/get_tool_force")
_ros2_get_solution_space             = g_node.create_client(GetSolutionSpace,           _srv_name_prefix +"aux_control/get_solution_space")
_ros2_get_orientation_error          = g_node.create_client(GetOrientationError,        _srv_name_prefix +"aux_control/get_orientation_error")

# Force/Stiffness Control & others Operations
_ros2_get_workpiece_weight        = g_node.create_client(GetWorkpieceWeight,            _srv_name_prefix +"force/get_workpiece_weight")
_ros2_reset_workpiece_weight      = g_node.create_client(ResetWorkpieceWeight,          _srv_name_prefix +"force/reset_workpiece_weight")
 
_ros2_parallel_axis1              = g_node.create_client(ParallelAxis1,                 _srv_name_prefix +"force/parallel_axis1")
_ros2_parallel_axis2              = g_node.create_client(ParallelAxis2,                 _srv_name_prefix +"force/parallel_axis2")
_ros2_align_axis1                 = g_node.create_client(AlignAxis1,                    _srv_name_prefix +"force/align_axis1")
_ros2_align_axis2                 = g_node.create_client(AlignAxis2,                    _srv_name_prefix +"force/align_axis2")
_ros2_is_done_bolt_tightening     = g_node.create_client(IsDoneBoltTightening,          _srv_name_prefix +"force/is_done_bolt_tightening")
_ros2_release_compliance_ctrl     = g_node.create_client(ReleaseComplianceCtrl,         _srv_name_prefix +"force/release_compliance_ctrl")
_ros2_task_compliance_ctrl        = g_node.create_client(TaskComplianceCtrl,            _srv_name_prefix +"force/task_compliance_ctrl")
_ros2_set_stiffnessx              = g_node.create_client(SetStiffnessx,                 _srv_name_prefix +"force/set_stiffnessx")
_ros2_calc_coord                  = g_node.create_client(CalcCoord,                     _srv_name_prefix +"force/calc_coord")
_ros2_set_user_cart_coord1        = g_node.create_client(SetUserCartCoord1,             _srv_name_prefix +"force/set_user_cart_coord1")
_ros2_set_user_cart_coord2        = g_node.create_client(SetUserCartCoord2,             _srv_name_prefix +"force/set_user_cart_coord2")
_ros2_set_user_cart_coord3        = g_node.create_client(SetUserCartCoord3,             _srv_name_prefix +"force/set_user_cart_coord3")
_ros2_overwrite_user_cart_coord   = g_node.create_client(OverwriteUserCartCoord,        _srv_name_prefix +"force/overwrite_user_cart_coord")
_ros2_get_user_cart_coord         = g_node.create_client(GetUserCartCoord,              _srv_name_prefix +"force/get_user_cart_coord")
_ros2_set_desired_force           = g_node.create_client(SetDesiredForce,               _srv_name_prefix +"force/set_desired_force")
_ros2_release_force               = g_node.create_client(ReleaseForce,                  _srv_name_prefix +"force/release_force")
_ros2_check_position_condition    = g_node.create_client(CheckPositionCondition,        _srv_name_prefix +"force/check_position_condition")
_ros2_check_force_condition       = g_node.create_client(CheckForceCondition,           _srv_name_prefix +"force/check_force_condition")
_ros2_check_orientation_condition1= g_node.create_client(CheckOrientationCondition1,    _srv_name_prefix +"force/check_orientation_condition1")
_ros2_check_orientation_condition2= g_node.create_client(CheckOrientationCondition2,    _srv_name_prefix +"force/check_orientation_condition2")
_ros2_coord_transform             = g_node.create_client(CoordTransform,                _srv_name_prefix +"force/coord_transform")

#  GPIO Operations
_ros2_set_digital_output         = g_node.create_client(SetCtrlBoxDigitalOutput,        _srv_name_prefix +"io/set_ctrl_box_digital_output")
_ros2_get_digital_input          = g_node.create_client(GetCtrlBoxDigitalInput,         _srv_name_prefix +"io/get_ctrl_box_digital_input")
_ros2_set_tool_digital_output    = g_node.create_client(SetToolDigitalOutput,           _srv_name_prefix +"io/set_tool_digital_output")
_ros2_get_tool_digital_input     = g_node.create_client(GetToolDigitalInput,            _srv_name_prefix +"io/get_tool_digital_input")
_ros2_set_analog_output          = g_node.create_client(SetCtrlBoxAnalogOutput,         _srv_name_prefix +"io/set_ctrl_box_analog_output")
_ros2_get_analog_input           = g_node.create_client(GetCtrlBoxAnalogInput,          _srv_name_prefix +"io/get_ctrl_box_analog_input")
_ros2_set_mode_analog_output     = g_node.create_client(SetCtrlBoxAnalogOutputType,     _srv_name_prefix +"io/set_ctrl_box_analog_output_type")  
_ros2_set_mode_analog_input      = g_node.create_client(SetCtrlBoxAnalogInputType,      _srv_name_prefix +"io/set_ctrl_box_analog_input_type")   
_ros2_get_digital_output         = g_node.create_client(GetCtrlBoxDigitalOutput,        _srv_name_prefix +"io/get_ctrl_box_digital_output")
_ros2_get_tool_digital_output    = g_node.create_client(GetToolDigitalOutput,           _srv_name_prefix +"io/get_tool_digital_output")

#  Modbus Operations, 
_ros2_set_modbus_output          = g_node.create_client(SetModbusOutput,                _srv_name_prefix +"modbus/set_modbus_output")
_ros2_get_modbus_input           = g_node.create_client(GetModbusInput,                 _srv_name_prefix +"modbus/get_modbus_input")
_ros2_add_modbus_signal          = g_node.create_client(ConfigCreateModbus,             _srv_name_prefix +"modbus/config_create_modbus")
_ros2_del_modbus_signal          = g_node.create_client(ConfigDeleteModbus,             _srv_name_prefix +"modbus/config_delete_modbus")

# TCP Operations, 
_ros2_set_current_tcp            = g_node.create_client(SetCurrentTcp,                  _srv_name_prefix +"tcp/set_current_tcp")
_ros2_get_current_tcp            = g_node.create_client(GetCurrentTcp,                  _srv_name_prefix +"tcp/get_current_tcp")        
_ros2_config_create_tcp          = g_node.create_client(ConfigCreateTcp,                _srv_name_prefix +"tcp/config_create_tcp")
_ros2_config_delete_tcp          = g_node.create_client(ConfigDeleteTcp,                _srv_name_prefix +"tcp/config_delete_tcp")

# Tool Operations, 
_ros2_set_current_tool           = g_node.create_client(SetCurrentTool,                 _srv_name_prefix +"tool/set_current_tool")
_ros2_get_current_tool           = g_node.create_client(GetCurrentTool,                 _srv_name_prefix +"tool/get_current_tool")      
_ros2_config_create_tool         = g_node.create_client(ConfigCreateTool,               _srv_name_prefix +"tool/config_create_tool")
_ros2_config_delete_tool         = g_node.create_client(ConfigDeleteTool,               _srv_name_prefix +"tool/config_delete_tool")
_ros2_set_tool_shape             = g_node.create_client(SetToolShape,                   _srv_name_prefix +"tool/set_tool_shape")

# DRL Operations, 
_ros2_drl_pause                  = g_node.create_client(DrlPause,                       _srv_name_prefix +"drl/drl_pause")
_ros2_drl_resume                 = g_node.create_client(DrlResume,                      _srv_name_prefix +"drl/drl_resume")
_ros2_drl_start                  = g_node.create_client(DrlStart,                       _srv_name_prefix +"drl/drl_start")
_ros2_drl_stop                   = g_node.create_client(DrlStop,                        _srv_name_prefix +"drl/drl_stop")
_ros2_get_drl_state              = g_node.create_client(GetDrlState,                    _srv_name_prefix +"drl/get_drl_state")

# RT
_ros2_connect_rt_control         = g_node.create_client(ConnectRtControl,       _srv_name_prefix + "realtime/connect_rt_control")
_ros2_disconnect_rt_control      = g_node.create_client(DisconnectRtControl,    _srv_name_prefix + "realtime/disconnect_rt_control")
_ros2_get_rt_control_output_version_list = g_node.create_client(GetRtControlOutputVersionList, _srv_name_prefix + "realtime/get_rt_control_output_version_list")
_ros2_get_rt_control_input_version_list = g_node.create_client(GetRtControlInputVersionList, _srv_name_prefix + "realtime/get_rt_control_input_version_list")
_ros2_get_rt_control_input_data_list = g_node.create_client(GetRtControlInputDataList, _srv_name_prefix + "realtime/get_rt_control_input_data_list")
_ros2_get_rt_control_output_data_list = g_node.create_client(GetRtControlOutputDataList, _srv_name_prefix + "realtime/get_rt_control_output_data_list")
_ros2_start_rt_control = g_node.create_client(StartRtControl, _srv_name_prefix + "realtime/start_rt_control")
_ros2_stop_rt_control  = g_node.create_client(StopRtControl,  _srv_name_prefix + "realtime/stop_rt_control")
_ros2_set_rt_control_input  = g_node.create_client(SetRtControlInput,  _srv_name_prefix + "realtime/set_rt_control_input")
_ros2_set_rt_control_output = g_node.create_client(SetRtControlOutput, _srv_name_prefix + "realtime/set_rt_control_output")
_ros2_set_velj_rt = g_node.create_client(SetVeljRt, _srv_name_prefix + "realtime/set_velj_rt")
_ros2_set_accj_rt = g_node.create_client(SetAccjRt, _srv_name_prefix + "realtime/set_accj_rt")
_ros2_set_velx_rt = g_node.create_client(SetVelxRt, _srv_name_prefix + "realtime/set_velx_rt")
_ros2_set_accx_rt = g_node.create_client(SetAccxRt, _srv_name_prefix + "realtime/set_accx_rt")
_ros2_read_data_rt  = g_node.create_client(ReadDataRt,  _srv_name_prefix + "realtime/read_data_rt")
_ros2_write_data_rt = g_node.create_client(WriteDataRt, _srv_name_prefix + "realtime/write_data_rt")

########################################################################################################################################

# point count
POINT_COUNT = 6

# solution space
DR_SOL_MIN = 0
DR_SOL_MAX = 7

# posb seg_type
DR_LINE   = 0
DR_CIRCLE = 1

# move reference
DR_BASE        = 0
DR_TOOL        = 1
DR_WORLD       = 2
DR_TC_USER_MIN = 101
DR_TC_USER_MAX = 200

# move mod
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

# move reaction
DR_MV_RA_NONE      = 0
DR_MV_RA_DUPLICATE = 0
DR_MV_RA_OVERRIDE  = 1

# move command type
DR_MV_COMMAND_NORM = 0

# movesx velocity
DR_MVS_VEL_NONE    = 0
DR_MVS_VEL_CONST   = 1

# motion state
DR_STATE_IDLE   = 0
DR_STATE_INIT   = 1
DR_STATE_BUSY   = 2
DR_STATE_BLEND  = 3
DR_STATE_ACC    = 4
DR_STATE_CRZ    = 5
DR_STATE_DEC    = 6

# axis
DR_AXIS_X = 0
DR_AXIS_Y = 1
DR_AXIS_Z = 2
DR_AXIS_A = 10
DR_AXIS_B = 11
DR_AXIS_C = 12

# collision sensitivity
DR_COLSENS_DEFAULT = 20
DR_COLSENS_MIN = 1   #10   2017/11/08 변경 
DR_COLSENS_MAX = 300 #100  2017/11/08 변경

# speed
DR_OP_SPEED_MIN = 1
DR_OP_SPEED_MAX = 100

# stop
DR_QSTOP_STO = 0
DR_QSTOP     = 1
DR_SSTOP     = 2
DR_HOLD      = 3

DR_STOP_FIRST = DR_QSTOP_STO
DR_STOP_LAST = DR_HOLD

# condition
DR_COND_NONE = -10000

# digital I/O
DR_DIO_MIN_INDEX = 1
DR_DIO_MAX_INDEX = 16   #8 16개로 확장됨 2017/08/18  

# tool digital I/O
DR_TDIO_MIN_INDEX = 1
DR_TDIO_MAX_INDEX = 6

# I/O value
ON = 1
OFF = 0

# Analog I/O mode
DR_ANALOG_CURRENT = 0
DR_ANALOG_VOLTAGE = 1

# modbus type
DR_MODBUS_DIG_INPUT  = 0
DR_MODBUS_DIG_OUTPUT = 1
DR_MODBUS_REG_INPUT  = 2
DR_MODBUS_REG_OUTPUT = 3
DR_DISCRETE_INPUT    = DR_MODBUS_DIG_INPUT
DR_COIL              = DR_MODBUS_DIG_OUTPUT
DR_INPUT_REGISTER    = DR_MODBUS_REG_INPUT
DR_HOLDING_REGISTER  = DR_MODBUS_REG_OUTPUT

DR_MODBUS_ACCESS_MAX    = 32
DR_MAX_MODBUS_NAME_SIZE = 32

# tp_popup pm_type
DR_PM_MESSAGE = 0
DR_PM_WARNING = 1
DR_PM_ALARM   = 2
DR_TP_POPUP_BUTTON_TYPE_STOP_RESUME = 0 # add 2019/04/01
DR_TP_POPUP_BUTTON_TYPE_STOP        = 1 # add 2019/04/01

# tp_get_user_input type
DR_VAR_INT   = 0
DR_VAR_FLOAT = 1
DR_VAR_STR   = 2
DR_VAR_BOOL  = 3 # add 2020/01/29

# len
DR_VELJ_DT_LEN = 6
DR_ACCJ_DT_LEN = 6

DR_VELX_DT_LEN = 2
DR_ACCX_DT_LEN = 2

DR_ANGLE_DT_LEN   = 2
DR_COG_DT_LEN     = 3
DR_WEIGHT_DT_LEN  = 3
DR_VECTOR_DT_LEN  = 3
DR_ST_DT_LEN      = 6
DR_FD_DT_LEN      = 6
DR_DIR_DT_LEN     = 6
DR_INERTIA_DT_LEN = 6
DR_VECTOR_U1_LEN  = 3
DR_VECTOR_V1_LEN  = 3

# set_singular_handling mode 
DR_AVOID     = 0
DR_TASK_STOP = 1
DR_VAR_VEL   = 2        #add 2019/04/01 

# object container type
DR_FIFO      = 0
DR_LIFO      = 1

# set_desired_force mod
DR_FC_MOD_ABS = 0
DR_FC_MOD_REL = 1

# global variable type
DR_GLOBAL_VAR_TYPE_BOOL   = 0
DR_GLOBAL_VAR_TYPE_INT    = 1
DR_GLOBAL_VAR_TYPE_FLOAT  = 2
DR_GLOBAL_VAR_TYPE_STR    = 3
DR_GLOBAL_VAR_TYPE_POSJ   = 4
DR_GLOBAL_VAR_TYPE_POSX   = 5
DR_GLOBAL_VAR_TYPE_UNKNOWN= 6

# Industrial Ethernet(EtherNet/IP, PROFINET) Slave address
DR_IE_SLAVE_GPR_ADDR_START     = 0
DR_IE_SLAVE_GPR_ADDR_END       =23
DR_IE_SLAVE_GPR_ADDR_END_BIT   =63

# 경로 수정 기능
DR_DPOS = 0
DR_DVEL = 1

# Homing 기능
DR_HOME_TARGET_MECHANIC = 0
DR_HOME_TARGET_USER     = 1

# movec ori 옵션 
DR_MV_ORI_TEACH  = 0    #교시자세
DR_MV_ORI_FIXED  = 1    #고정자세
DR_MV_ORI_RADIAL = 2    #원주구속자세

# app_type
DR_MV_APP_NONE   = 0
DR_MV_APP_WELD   = 1

# =============================================================================================
# global variable

DR_CONFIG_PRT_EXT_RESULT = True # False
DR_CONFIG_PRT_RESULT     = True # False

_g_blend_state  = False
_g_blend_radius = 0.0

_g_velj = [0.0] * DR_VELJ_DT_LEN
_g_accj = [0.0] * DR_ACCJ_DT_LEN

_g_velx = [0.0] * DR_VELX_DT_LEN
_g_velx[0]= 0.0
_g_velx[1]= DR_COND_NONE

_g_accx = [0.0] * DR_ACCX_DT_LEN
_g_accx[0]= 0.0
_g_accx[1]= DR_COND_NONE

_g_coord = DR_BASE
_g_drl_result_th = None

#_g_tp_lock = threading.Lock()       # only 1 execution allowed with TP

_g_test_cnt =0
_g_test_max =0

_g_analog_output_mode_ch1 = -1
_g_analog_output_mode_ch2 = -1

#DR_SUB_PROGRAM_LIST = dict()    #서브 프로그램 저장 리스트

########################################################################################################################################

def wait(second):
    time.sleep(second)

def print_ext_result(str):
    if DR_CONFIG_PRT_EXT_RESULT:
        # print("[{0}] / {1}".format(strftime("%Y-%m-%d %H:%M:%S", gmtime()), str))
        print("__{0}".format(str))

def print_result(str):
    if DR_CONFIG_PRT_RESULT:
        # print("[{0}] / {1}".format(strftime("%Y-%m-%d %H:%M:%S", gmtime()), str))
        print("__{0}".format(str))

def _check_valid_vel_acc_joint(vel, acc, time):
    if float(time) == 0.0 or float(time) == -10000.0:
        for item in vel:
            if float(item) != 0.0:
                break
        else:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v (0.0, when time = 0.0)", True)

        for item in acc:
            if float(item) != 0.0:
                break
        else:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a (0.0, when time = 0.0)", True)

    return

def _check_valid_vel_acc_task(vel, acc, time):

    if float(time) == 0.0 or float(time) == -10000.0:
        if vel[0] <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel1, v1 (0.0, when time = 0.0)", True)
        else:
            if (vel[1]!=DR_COND_NONE) and (vel[1]<=0):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel2, v2 (0.0, when time = 0.0)", True)

        if acc[0] <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc1, a1 (0.0, when time = 0.0)", True)
        else:
            if (acc[1]!=DR_COND_NONE) and (acc[1]<=0):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc2, a2 (0.0, when time = 0.0)", True)

    return


def set_velj(vel):
    vel_list = None

    # vel
    if type(vel) == int or type(vel) == float:
        if vel <= 0:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel")

        vel_list = [vel] * DR_VELJ_DT_LEN
    elif type(vel) == list and len(vel) == POINT_COUNT:
        vel_list = vel

        if is_number(vel_list) != True:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel")

        for item in vel:
            if item <= 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel")
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel")

    # set global velj
    global _g_velj

    _g_velj = vel_list

    return 0

def set_accj(acc):
    acc_list = None

    # acc
    if type(acc) == int or type(acc) == float:
        acc_list = [acc] * DR_ACCJ_DT_LEN

        if acc <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc")
    elif type(acc) == list and len(acc) == POINT_COUNT:
        acc_list = acc

        if is_number(acc_list) != True:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc")

        for item in acc:
            if item <= 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc")
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc")

    # set global velj
    global _g_accj

    _g_accj = acc_list

    return 0

def set_velx(vel1, vel2=DR_COND_NONE):
    #print("vel1={0}, vel2={1}".format(vel1, vel2))

    # vel1
    if type(vel1) != int and type(vel1) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel1")

    if vel1 <= 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel1")

    # vel2
    if type(vel2) != int and type(vel2) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel2")

    if vel2 != DR_COND_NONE:
        if vel2 <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel2")

    # set global velx
    global _g_velx

    _g_velx = [vel1, vel2]

    return 0

def set_accx(acc1, acc2=DR_COND_NONE):
    #print("acc1={0}, acc2={1}".format(acc1, acc2))

    # acc1
    if type(acc1) != int and type(acc1) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc1")

    if acc1 <= 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc1")

    # acc2
    if type(acc2) != int and type(acc2) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc2")

    if acc2 != DR_COND_NONE:
        if acc2 <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc2")

    # set global accx
    global _g_accx

    _g_accx = [acc1, acc2]

    return 0

# convert : list -> Float64MultiArray
def _ros_listToFloat64MultiArray(list_src):
    _res = []
    for i in list_src:
        item = Float64MultiArray()
        if __ROS2__:
            item.data = [float(x) for x in i]
        else:
            item.data = i
        _res.append(item)
    #print(_res)
    #print(len(_res))
    return _res

# convert : Float64MultiArray -> list
def _ros_Float64MultiArrayTolist(multi_arr_f64):
    _res = []
    for i in range( len(multi_arr_f64) ):
        _res.append( list(multi_arr_f64[i].data) )   
    #print(_res)
    #print(len(_res))
    return _res


##### SYSTEM ##############################################################################################################################
def set_robot_mode(robot_mode):
    if type(robot_mode) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : robot_mode")

    # ROS2 service call
    if __ROS2__:
        #srv = _ros_set_robot_mode(robot_mode)
        req = SetRobotMode.Request()
        req.robot_mode = robot_mode

        #ret = 0 if (srv.success == True) else -1
        while not _ros2_set_robot_mode.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Set Robot Mode Service is not available, waiting for service to becom available...")

        future = _ros2_set_robot_mode.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_robot_mode Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    return ret

def get_robot_mode():
    
    # ROS2 service call
    if __ROS2__:
        req = GetRobotMode.Request()

        #ret = 0 if (srv.success == True) else -1
        future = _ros2_get_robot_mode.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_robot_mode Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.robot_mode            
    return ret

def set_robot_system(robot_system):
    if type(robot_system) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : robot_system")
    # ROS2 service call
    if __ROS2__:
        req = SetRobotSystem.Request()
        req.robot_system = robot_system

        future = _ros2_set_robot_system.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_robot_system Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1   
                g_node.get_logger().info('set_robot_system Service call failed')
            else:        
                ret = 0 if (result.success == True) else -1                
    return ret


def get_robot_system():
    # ROS2 service call
    if __ROS2__:
        req = GetRobotSystem.Request()

        future = _ros2_get_robot_system.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_robot_system Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.robot_system            
    return ret

def get_robot_state():
    # ROS2 service call
    if __ROS2__:
        req = GetRobotState.Request()

        future = _ros2_get_robot_state.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_robot_state Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.robot_state            
    return ret

def set_robot_speed_mode(speed_mode):
    if type(speed_mode) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed_mode")

    # ROS2 service call
    if __ROS2__:
        req = SetRobotSpeedMode.Request()
        req.speed_mode = speed_mode

        future = _ros2_set_robot_speed_mode.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_robot_speed_mode Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    return ret

def get_robot_speed_mode():
    # ROS2 service call
    if __ROS2__:
        req = GetRobotSpeedMode.Request()

        future = _ros2_get_robot_speed_mode.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_robot_speed_mode Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.speed_mode            
    return ret

def set_safe_stop_reset_type(reset_type):
    if type(reset_type) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : reset_type")

    # ROS2 service call
    if __ROS2__:
        req = SetSafeStopResetType.Request()
        req.reset_type = reset_type

        future = _ros2_set_safe_stop_reset_type.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_safe_stop_reset_type Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    return ret


def get_current_pose(space_type):
    if type(space_type) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : space_type")

    if __ROS2__:
        req = GetCurrentPose.Request()
        req.space_type = space_type

        future = _ros2_get_current_pose.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_current_pose Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.pos            
    return ret

def get_current_solution_space():
    # ROS2 service call
    if __ROS2__:
        req = GetCurrentSolutionSpace.Request()

        future = _ros2_get_current_solution_space.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_current_solution_space Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.sol_space            
    return ret

def get_last_alarm():
     #ROS2 service call
    if __ROS2__:
        req = GetLastAlarm.Request()

        future = _ros2_get_last_alarm.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_last_alarm Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.log_alarm            
    return ret

##### Auxiliary Control ##############################################################################################################################
def get_control_mode():
    if __ROS2__:
        req = GetControlMode.Request()  

        future = _ros2_get_control_mode.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_control_mode Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.control_mode            
    return ret

def get_control_space():
    if __ROS2__:
        req = GetControlSpace.Request()  

        future = _ros2_get_control_space.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_control_space Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.space            
    return ret

def get_current_posj():
    if __ROS2__:
        req = GetCurrentPosj.Request()  

        future = _ros2_get_current_posj.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_current_posj Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:     
                pos = list(result.pos)  # Convert tuple to list   
                # set posj
                cur_pos = posj(pos)
                ret = cur_pos           
    return ret

def get_current_velj():
    if __ROS2__:
        req = GetCurrentVelj.Request()  

        future = _ros2_get_current_velj.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_current_velj Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = list(result.joint_speed)  # Convert tuple to list            
    return ret

def get_desired_posj():
    if __ROS2__:
        req = GetDesiredPosj.Request()  

        future = _ros2_get_desired_posj.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_desired_posj Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                pos = list(result.pos)  # Convert tuple to list            
                ret = posj(pos)
    return ret

def get_desired_velj():
    if __ROS2__:
        req = GetDesiredVelj.Request()  

        future = _ros2_get_desired_velj.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_desired_velj Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = list(result.joint_vel)  # Convert tuple to list           
    return ret

def get_current_posx(ref=None):

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref != DR_BASE and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS2__:
        req = GetCurrentPosx.Request()  
        req.ref = _ref

        future = _ros2_get_current_posx.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_current_posx Service call failed %r' % (e,))
        else:
            if result == None:
                return None, None    
            else:        
                posx_info = _ros_Float64MultiArrayTolist(result.task_pos_info) # Convert Float64MultiArray to list
                pos = []
                for i in range(POINT_COUNT):
                    pos.append(posx_info[0][i])
                sol = int(round( posx_info[0][6] ))
                conv_posx = posx(pos)
                return conv_posx, sol
    return 0

def get_current_tool_flange_posx(ref=None):

    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS2__:
        req =  GetCurrentToolFlangePosx.Request()  
        req.ref = _ref

        future = _ros2_get_current_tool_flange_posx.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_current_tool_flange_posx Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.pos            

                pos = list(result.pos)  # Convert tuple to list
                # set posx type
                ret = posx(pos)
    return ret

def get_current_velx(ref=None):
    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS2__:
        req =  GetCurrentVelx.Request()  
        req.ref = _ref

        future = _ros2_get_current_velx.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_current_velx Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.vel            

    return ret

def get_desired_posx(ref=None):
    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref != DR_BASE and _ref != DR_TOOL and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS2__:
        req =  GetDesiredPosx.Request()  
        req.ref = _ref

        future = _ros2_get_desired_posx.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_desired_posx Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                pos = list(result.pos)  # Convert tuple to list
                conv_posx = posx(pos)
                ret = conv_posx
    return ret

def get_desired_velx(ref=None):
    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS2__:
        req =  GetDesiredVelx.Request()  
        req.ref = _ref

        future = _ros2_get_desired_velx.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_desired_velx Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = list(result.vel)  # Convert tuple to list            
    return ret

def get_current_solution_space():
    if __ROS2__:
        req =  GetCurrentSolutionSpace.Request()  

        future = _ros2_get_current_solution_space.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_current_solution_space Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.sol_space            
    return ret

def get_current_rotm(ref=None):

    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if __ROS2__:
        req =  GetCurrentRotm.Request()  
        req.ref = _ref

        future = _ros2_get_current_rotm.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_current_rotm Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = _ros_Float64MultiArrayTolist(result.rot_matrix) # Convert Float64MultiArray to list 

    return ret

def get_joint_torque():
    if __ROS2__:
        req =  GetJointTorque.Request()  

        future = _ros2_get_joint_torque.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_joint_torque Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                 ret = list(result.jts)  # Convert tuple to list  
    return ret

def get_external_torque():
    if __ROS2__:
        req =  GetExternalTorque.Request()  

        future = _ros2_get_external_torque.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_external_torque Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = list(result.ext_torque)  # Convert tuple to list  
    return ret

def get_tool_force(ref=None):

    _ref = DR_BASE if ref is None else ref

    if _ref not in [DR_BASE, DR_TOOL, DR_WORLD]:
        raise DR_Error(DR_ERROR_VALUE, f"Invalid value : ref({_ref})")

    ret = -1

    if __ROS2__:
        req = GetToolForce.Request()
        req.ref = _ref

        future = _ros2_get_tool_force.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
            if result and result.success:
                ret = list(result.tool_force)
            else:
                g_node.get_logger().warn("get_tool_force: service call returned unsuccessful or empty result")
        except Exception as e:
            g_node.get_logger().error(f"get_tool_force: service call failed - {e}")

    return ret

def get_solution_space(pos):
    _pos = get_posj(pos)

    if __ROS2__:
        req =  GetSolutionSpace.Request()  
        req.pos = [float(x) for x in _pos]

        future = _ros2_get_solution_space.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_solution_space Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.sol_space            
    return ret

def get_orientation_error(xd, xc, axis):
    # xd, xc
    _xd = get_posx(xd)
    _xc = get_posx(xc)

    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if (axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z and \
        axis != DR_AXIS_A and axis != DR_AXIS_B and axis != DR_AXIS_C):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

    if __ROS2__:
        req =  GetOrientationError.Request()  
        req.xd = [float(x) for x in _xd]
        req.xc = [float(x) for x in _xc]
        req.axis = axis

        future = _ros2_get_orientation_error.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_orientation_error Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.ori_error            
    return ret

##### MOTION ##############################################################################################################################
def trans(pos, delta, ref=None, ref_out=DR_BASE):
    # pos, delta
    _pos = get_posx(pos)
    _delta = get_posx(delta)
    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref
    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
    if _ref != DR_BASE and _ref != DR_TOOL and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    # check ref_out
    if type(ref_out) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref_out")
    if ref_out != DR_BASE and ref_out != DR_WORLD and (ref_out < DR_TC_USER_MIN or ref_out > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref_out({0})".format(ref_out))
    # ROS service call
    if __ROS2__: 
        req = Trans.Request()
        req.pos     = [float(x) for x in _pos]
        req.delta   = [float(x) for x in _delta]
        req.ref     = _ref
        req.ref_out = ref_out            

        future = _ros2_trans.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('trans Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.trans_pos            
    return ret

def fkin(pos, ref=None):

    _pos = get_posj(pos)

    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    # ROS service call
    if __ROS2__: 
        req = Fkin.Request()
        req.pos = [float(x) for x in _pos]
        req.ref = _ref
      
        future = _ros2_fkin.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('fkin Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.conv_posx            
    return ret

def ikin(pos, sol_space, ref=None):

    _pos = get_posx(pos)

    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    # sol_space
    if type(sol_space) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : sol_space")

    if sol_space < DR_SOL_MIN or sol_space > DR_SOL_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : sol_space({0})".format(sol_space))

    # ROS service call
    if __ROS2__: 
        req = Ikin.Request()
        req.pos       = [float(x) for x in _pos]
        req.sol_space = sol_space        
        req.ref       = _ref

        future = _ros2_ikin.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('ikin Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.conv_posj            
    return ret

def set_ref_coord(coord):
    # coord
    if type(coord) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : coord")

    if coord != DR_BASE and coord != DR_TOOL and coord != DR_WORLD and not (DR_TC_USER_MIN <= coord <= DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : coord({0})".format(coord))

    # ROS service call
    if __ROS2__: 
        req = SetRefCoord.Request()
        req.coord = coord

        future = _ros2_set_ref_coord.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_ref_coord Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                

    # set global accx
    global _g_coord

    _g_coord = coord

    #print_result("{0} = set_ref_coord(coord:{1})".format(ret, coord))
    return ret

def alter_motion_stream(pos):
    _pos = get_posx(pos)

    if __ROS2__:
        msg = AlterMotionStream()
        msg.pos = [float(p) for p in _pos]
        
        _ros2_alter_motion_stream_pub.publish(msg)
        
        return 0
    else:
        raise DR_Error(DR_ERROR_TYPE, "alter_motion is only supported in ROS2 mode.")

def servoj(pos, vel=None, acc=None, time=None, mode=DR_SERVO_QUEUE, v=None, a=None, t=None, m=None):
    # pos
    _pos = get_posj(pos)

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velj
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accj
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # _mode
    _mode = get_param(mode, m)
    if _mode == None:
        _mode = DR_SERVO_QUEUE
    if type(_mode) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mode, m")
    if _mode != DR_SERVO_OVERRIDE and _mode != DR_SERVO_QUEUE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mode, m")

    # check vel, acc, time
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # Create and publish the message
    if __ROS2__:
        msg = ServojStream()
        msg.pos = [float(p) for p in _pos]
        msg.vel = [float(v) for v in _vel]
        msg.acc = [float(a) for a in _acc]
        msg.time = float(_time)
        msg.mode = _mode
        
        _ros2_servoj_stream_pub.publish(msg)
        
        return 0

def servol(pos, vel=None, acc=None, time=None, v=None, a=None, t=None):
    # Constants for servol
    DR_VELL_DT_LEN = 2
    DR_ACCL_DT_LEN = 2

    # pos
    _pos = get_posx(pos)

    # _vel
    temp = get_param(vel, v)
    if temp is None:
        _vel = _g_velx
    else:
        if isinstance(temp, (int, float)):
            _vel = [temp] * DR_VELL_DT_LEN
        elif isinstance(temp, list) and len(temp) == DR_VELL_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type for vel/v: Must be a number or a list of 2 numbers.")

    if not is_number(_vel):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value in vel/v.")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Velocity components cannot be negative.")

    # _acc
    temp = get_param(acc, a)
    if temp is None:
        _acc = _g_accx
    else:
        if isinstance(temp, (int, float)):
            _acc = [temp] * DR_ACCL_DT_LEN
        elif isinstance(temp, list) and len(temp) == DR_ACCL_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type for acc/a: Must be a number or a list of 2 numbers.")

    if not is_number(_acc):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value in acc/a.")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Acceleration components cannot be negative.")

    # _time
    _time = get_param(time, t)
    if _time is None:
        _time = 0.0

    if not isinstance(_time, (int, float)):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for time/t.")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Time cannot be negative.")

    # check vel, acc, time
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # Create and publish the message
    if __ROS2__:
        msg = ServolStream()
        msg.pos = [float(p) for p in _pos]
        msg.vel = [float(v) for v in _vel]
        msg.acc = [float(a) for a in _acc]
        msg.time = float(_time)
        
        _ros2_servol_stream_pub.publish(msg)
        
        return 0

def speedj(vel=None, acc=None, time=None, v=None, a=None, t=None):

    # _vel
    _vel_param = get_param(vel, v)
    if _vel_param is None:
        raise DR_Error(DR_ERROR_VALUE, "Missing required argument: vel or v")

    if type(_vel_param) == int or type(_vel_param) == float:
        _vel = [_vel_param] * DR_VELJ_DT_LEN
    elif type(_vel_param) == list and len(_vel_param) == DR_VELJ_DT_LEN:
        _vel = _vel_param
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    _acc_param = get_param(acc, a)
    if _acc_param is None:
        raise DR_Error(DR_ERROR_VALUE, "Missing required argument: acc or a")

    if type(_acc_param) == int or type(_acc_param) == float:
        _acc = [_acc_param] * DR_ACCJ_DT_LEN
    elif type(_acc_param) == list and len(_acc_param) == DR_ACCJ_DT_LEN:
        _acc = _acc_param
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time is None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # Create and publish the message
    if __ROS2__:
        msg = SpeedjStream()
        msg.vel = [float(item) for item in _vel]
        msg.acc = [float(item) for item in _acc]
        msg.time = float(_time)
        
        _ros2_speedj_stream_pub.publish(msg)
        
        return 0

def speedl(vel, acc, time=None, v=None, a=None, t=None):
    # Constants for speedl
    DR_VELL_DT_LEN = 6
    DR_ACCL_DT_LEN = 2

    # _vel
    _vel_param = get_param(vel, v)
    if _vel_param is None:
        raise DR_Error(DR_ERROR_VALUE, "Missing required argument: vel or v")

    if isinstance(_vel_param, (int, float)):
        _vel = [_vel_param] * DR_VELL_DT_LEN
    elif isinstance(_vel_param, list) and len(_vel_param) == DR_VELL_DT_LEN:
        _vel = _vel_param
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for vel/v: Must be a number or a list of 2 numbers.")

    if not is_number(_vel):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value in vel/v.")

    # _acc
    _acc_param = get_param(acc, a)
    if _acc_param is None:
        raise DR_Error(DR_ERROR_VALUE, "Missing required argument: acc or a")

    if isinstance(_acc_param, (int, float)):
        _acc = [_acc_param] * DR_ACCL_DT_LEN
    elif isinstance(_acc_param, list) and len(_acc_param) == DR_ACCL_DT_LEN:
        _acc = _acc_param
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for acc/a: Must be a number or a list of 2 numbers.")

    if not is_number(_acc):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value in acc/a.")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Acceleration components cannot be negative.")

    # _time
    _time = get_param(time, t)
    if _time is None:
        _time = 0.0

    if not isinstance(_time, (int, float)):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for time/t.")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # Create and publish the message
    if __ROS2__:
        msg = SpeedlStream()
        msg.vel = [float(item) for item in _vel]
        msg.acc = [float(item) for item in _acc]
        msg.time = float(_time)
        
        _ros2_speedl_stream_pub.publish(msg)
        
        return 0

def servoj_rt(pos, vel=None, acc=None, time=None, v=None, a=None, t=None):
    # pos
    _pos = get_posj(pos)

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velj
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accj
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # Create and publish the message
    if __ROS2__:
        msg = ServojRtStream()
        msg.pos = [float(p) for p in _pos]
        msg.vel = [float(v) for v in _vel]
        msg.acc = [float(a) for a in _acc]
        msg.time = float(_time)
        
        _ros2_servoj_rt_stream_pub.publish(msg)
        
        return 0
    
def servol_rt(pos, vel=None, acc=None, time=None, v=None, a=None, t=None):

    # pos
    _pos = get_posx(pos)

    # _time
    _time = get_param(time, t)
    if _time is None:
        raise DR_Error(DR_ERROR_VALUE, "Missing required argument: time or t")
    if not isinstance(_time, (int, float)):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for time/t.")
    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # _vel
    _vel_param = get_param(vel, v)
    if _vel_param is None:
        _vel = [DR_COND_NONE] * POINT_COUNT
    elif isinstance(_vel_param, (int, float)):
        _vel = [_vel_param] * POINT_COUNT
    elif isinstance(_vel_param, list) and len(_vel_param) == POINT_COUNT:
        _vel = _vel_param
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for vel/v: Must be a number or a list of 6 numbers.")
    if not is_number(_vel):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value in vel/v.")

    # _acc
    _acc_param = get_param(acc, a)
    if _acc_param is None:
        _acc = [DR_COND_NONE] * POINT_COUNT
    elif isinstance(_acc_param, (int, float)):
        _acc = [_acc_param] * POINT_COUNT
    elif isinstance(_acc_param, list) and len(_acc_param) == POINT_COUNT:
        _acc = _acc_param
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for acc/a: Must be a number or a list of 6 numbers.")
    if not is_number(_acc):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value in acc/a.")

    # check vel, acc, time
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # Create and publish the message
    if __ROS2__:
        msg = ServolRtStream()
        msg.pos = [float(p) for p in _pos]
        msg.vel = [float(v) for v in _vel]
        msg.acc = [float(a) for a in _acc]
        msg.time = float(_time)
        
        _ros2_servol_rt_stream_pub.publish(msg)
        
        return 0

def speedj_rt(vel, acc, time=None, t=None):

    # Constants for speedj_rt
    DR_VELJ_DT_LEN = 6
    DR_ACCJ_DT_LEN = 6

    # _vel
    if not isinstance(vel, list):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for vel: Must be a list of 6 numbers.")
    if len(vel) != DR_VELJ_DT_LEN:
        raise DR_Error(DR_ERROR_VALUE, f"Invalid length for vel: Must be {DR_VELJ_DT_LEN}.")
    if not is_number(vel):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value in vel: All elements must be numbers.")
    _vel = vel

    # _acc
    if not isinstance(acc, list):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for acc: Must be a list of 6 numbers.")
    if len(acc) != DR_ACCJ_DT_LEN:
        raise DR_Error(DR_ERROR_VALUE, f"Invalid length for acc: Must be {DR_ACCJ_DT_LEN}.")
    if not is_number(acc):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value in acc: All elements must be numbers.")
    # For acceleration, it's conventional for them to be non-negative.
    for item in acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Acceleration components cannot be negative.")
    _acc = acc

    # _time
    _time = get_param(time, t)
    if _time is None:
        _time = 0.0  # Default sync time as per DRFL documentation style

    if not isinstance(_time, (int, float)):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for time/t.")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Time cannot be negative.")

    # check vel, acc, time
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # Create and publish the message
    if __ROS2__:
        msg = SpeedjRtStream()
        msg.vel = [float(v) for v in _vel]
        msg.acc = [float(a) for a in _acc]
        msg.time = float(_time)
        
        _ros2_speedj_rt_stream_pub.publish(msg)
        
        return 0

def speedl_rt(vel, acc=None, time=None, t=None):
    DR_VELL_RT_DT_LEN = 6
    DR_ACCL_RT_DT_LEN = 6

    # _vel
    if not isinstance(vel, list):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for vel: Must be a list of 6 numbers.")
    if len(vel) != DR_VELL_RT_DT_LEN:
        raise DR_Error(DR_ERROR_VALUE, f"Invalid length for vel: Must be {DR_VELL_RT_DT_LEN}.")
    if not is_number(vel):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value in vel: All elements must be numbers.")
    _vel = vel

    # _acc
    if acc is None:
        _acc = [DR_COND_NONE] * DR_ACCL_RT_DT_LEN 
    elif isinstance(acc, list):
        if len(acc) != DR_ACCL_RT_DT_LEN:
            raise DR_Error(DR_ERROR_VALUE, f"Invalid length for acc: Must be {DR_ACCL_RT_DT_LEN}.")
        if not is_number(acc):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value in acc: All elements must be numbers.")
        for item in acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Acceleration components cannot be negative.")
        _acc = acc
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for acc: Must be a list of 6 numbers or None.")

    # _time
    _time = get_param(time, t)
    if _time is None:
        _time = 0.0

    if not isinstance(_time, (int, float)):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for time/t.")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Time cannot be negative.")

    # check vel, acc, time
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # Create and publish the message
    if __ROS2__:
        msg = SpeedlRtStream()
        msg.vel = [float(v) for v in _vel]
        msg.acc = [float(a) for a in _acc]
        msg.time = float(_time)
        
        _ros2_speedl_rt_stream_pub.publish(msg)
        
        return 0

def torque_rt(tor, time=None, t=None):
    DR_TORQUE_RT_DT_LEN = 6

    # _tor
    if not isinstance(tor, list):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for tor: Must be a list of 6 numbers.")
    if len(tor) != DR_TORQUE_RT_DT_LEN:
        raise DR_Error(DR_ERROR_VALUE, f"Invalid length for tor: Must be {DR_TORQUE_RT_DT_LEN}.")
    if not is_number(tor):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value in tor: All elements must be numbers.")
    _tor = tor

    # _time
    _time = get_param(time, t)
    if _time is None:
        _time = 0.0  # Default sync time

    if not isinstance(_time, (int, float)):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for time/t.")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Time cannot be negative.")

    # Create and publish the message
    if __ROS2__:
        msg = TorqueRtStream()
        msg.tor = [float(val) for val in _tor]
        msg.time = float(_time)
        
        _ros2_torque_rt_stream_pub.publish(msg)
        
        return 0
    
    print_result("Error: ROS2 is not initialized. Cannot publish torque_rt.")
    return -1

def set_safety_mode(safety_mode, safety_event):
    # Type checking for inputs
    if type(safety_mode) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : safety_mode")
    if type(safety_event) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : safety_event")

    # ROS2 service call
    if __ROS2__:
        # Create a request object
        req = SetSafetyMode.Request()
        req.safety_mode = safety_mode
        req.safety_event = safety_event

        # Wait for the service to be available
        while not _ros2_set_safety_mode.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Set Safety Mode service is not available, waiting for service to become available...")

        future = _ros2_set_safety_mode.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_safety_mode Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1
    return ret

def connect_rt_control(ip_address=None, port=None):
    g_node.get_logger().warning("RT service connection may already have been established by dsr_hw_interface; Need caution using this function again.")
    req_ip = ip_address if ip_address is not None else "192.168.137.100" # Need set to 192.168.137.50 in version below 3.4
    req_port = port if port is not None else 12347

    if ip_address is not None and type(ip_address) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for ip_address: expected str")
    if port is not None and type(port) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for port: expected int")

    ret = -1
    if __ROS2__:
        req = ConnectRtControl.Request()
        req.ip_address = req_ip
        req.port = req_port

        while not _ros2_connect_rt_control.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Connect RT Control Service is not available, waiting for it to become available...")

        future = _ros2_connect_rt_control.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('connect_rt_control Service call failed: %r' % (e,))
        else:
            if result is None:
                g_node.get_logger().error('connect_rt_control Service call returned None')
            else:
                ret = 0 if result.success else -1
    return ret

def disconnect_rt_control():
    ret = -1
    # ROS2 service call
    if __ROS2__:
        req = DisconnectRtControl.Request()

        while not _ros2_disconnect_rt_control.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Disconnect RT Control Service is not available, waiting for it to become available...")

        future = _ros2_disconnect_rt_control.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('disconnect_rt_control Service call failed: %r' % (e,))
        else:
            if result is None:
                g_node.get_logger().error('disconnect_rt_control Service call returned None')
            else:
                ret = 0 if result.success else -1
    return ret

def get_rt_control_output_version_list():
    ret = ""
    # ROS2 service call
    if __ROS2__:
        req = GetRtControlOutputVersionList.Request()

        while not _ros2_get_rt_control_output_version_list.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Get RT Control Output Version List Service is not available, waiting...")

        future = _ros2_get_rt_control_output_version_list.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('get_rt_control_output_version_list Service call failed: %r' % (e,))
        else:
            if result is not None and result.success:
                ret = result.version
            else:
                g_node.get_logger().error('Failed to get RT control output version list.')
    return ret

def get_rt_control_input_version_list():
    ret = ""
    # ROS2 service call
    if __ROS2__:
        req = GetRtControlInputVersionList.Request()

        while not _ros2_get_rt_control_input_version_list.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Get RT Control Input Version List Service is not available, waiting...")

        future = _ros2_get_rt_control_input_version_list.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('get_rt_control_input_version_list Service call failed: %r' % (e,))
        else:
            if result is not None and result.success:
                ret = result.version
            else:
                g_node.get_logger().error('Failed to get RT control input version list.')
    return ret

def get_rt_control_input_data_list(version):
    if type(version) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for version: expected str")

    ret = ""
    # ROS2 service call
    if __ROS2__:
        req = GetRtControlInputDataList.Request()
        req.version = version

        while not _ros2_get_rt_control_input_data_list.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Get RT Control Input Data List Service is not available, waiting...")

        future = _ros2_get_rt_control_input_data_list.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('get_rt_control_input_data_list Service call failed: %r' % (e,))
        else:
            if result is not None and result.success:
                ret = result.data
            else:
                g_node.get_logger().error('Failed to get RT control input data list.')
    return ret

def get_rt_control_output_data_list(version):
    if type(version) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for version: expected str")

    ret = ""
    # ROS2 service call
    if __ROS2__:
        req = GetRtControlOutputDataList.Request()
        req.version = version

        while not _ros2_get_rt_control_output_data_list.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Get RT Control Output Data List Service is not available, waiting...")

        future = _ros2_get_rt_control_output_data_list.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('get_rt_control_output_data_list Service call failed: %r' % (e,))
        else:
            if result is not None and result.success:
                ret = result.data
            else:
                g_node.get_logger().error('Failed to get RT control output data list.')
    return ret

def start_rt_control():
    ret = -1
    # ROS2 service call
    if __ROS2__:
        req = StartRtControl.Request()

        while not _ros2_start_rt_control.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Start RT Control Service is not available, waiting...")

        future = _ros2_start_rt_control.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('start_rt_control Service call failed: %r' % (e,))
        else:
            if result is not None:
                ret = 0 if result.success else -1
            else:
                g_node.get_logger().error('start_rt_control Service call returned None')
    return ret

def stop_rt_control():
    ret = -1
    # ROS2 service call
    if __ROS2__:
        req = StopRtControl.Request()

        while not _ros2_stop_rt_control.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Stop RT Control Service is not available, waiting...")

        future = _ros2_stop_rt_control.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('stop_rt_control Service call failed: %r' % (e,))
        else:
            if result is not None:
                ret = 0 if result.success else -1
            else:
                g_node.get_logger().error('stop_rt_control Service call returned None')
    return ret

def set_rt_control_input(version, period, loss):
    if type(version) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for version: expected str")
    if type(period) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for period: expected float")
    if type(loss) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for loss: expected int")

    ret = -1
    # ROS2 service call
    if __ROS2__:
        req = SetRtControlInput.Request()
        req.version = version
        req.period = period
        req.loss = loss

        while not _ros2_set_rt_control_input.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Set RT Control Input Service is not available, waiting...")

        future = _ros2_set_rt_control_input.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('set_rt_control_input Service call failed: %r' % (e,))
        else:
            if result is not None:
                ret = 0 if result.success else -1
            else:
                g_node.get_logger().error('set_rt_control_input Service call returned None')
    return ret

def set_rt_control_output(version, period, loss):
    if type(version) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for version: expected str")
    if type(period) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for period: expected float")
    if type(loss) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for loss: expected int")
        
    ret = -1
    # ROS2 service call
    if __ROS2__:
        req = SetRtControlOutput.Request()
        req.version = version
        req.period = period
        req.loss = loss

        while not _ros2_set_rt_control_output.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Set RT Control Output Service is not available, waiting...")

        future = _ros2_set_rt_control_output.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('set_rt_control_output Service call failed: %r' % (e,))
        else:
            if result is not None:
                ret = 0 if result.success else -1
            else:
                g_node.get_logger().error('set_rt_control_output Service call returned None')
    return ret

def set_velj_rt(vel):
    if type(vel) not in (list, tuple) or len(vel) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for vel: expected list or tuple of 6 floats")

    ret = -1
    if __ROS2__:
        req = SetVeljRt.Request()
        req.vel = vel

        while not _ros2_set_velj_rt.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Set Velj RT Service is not available, waiting...")

        future = _ros2_set_velj_rt.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('set_velj_rt Service call failed: %r' % (e,))
        else:
            if result is not None:
                ret = 0 if result.success else -1
    return ret

def set_accj_rt(acc):
    if type(acc) not in (list, tuple) or len(acc) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for acc: expected list or tuple of 6 floats")

    ret = -1
    if __ROS2__:
        req = SetAccjRt.Request()
        req.acc = acc

        while not _ros2_set_accj_rt.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Set Accj RT Service is not available, waiting...")

        future = _ros2_set_accj_rt.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('set_accj_rt Service call failed: %r' % (e,))
        else:
            if result is not None:
                ret = 0 if result.success else -1
    return ret

def set_velx_rt(trans, rotation):
    if type(trans) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for trans: expected float")
    if type(rotation) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for rotation: expected float")
        
    ret = -1
    if __ROS2__:
        req = SetVelxRt.Request()
        req.trans = trans
        req.rotation = rotation

        while not _ros2_set_velx_rt.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Set Velx RT Service is not available, waiting...")

        future = _ros2_set_velx_rt.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('set_velx_rt Service call failed: %r' % (e,))
        else:
            if result is not None:
                ret = 0 if result.success else -1
    return ret

def set_accx_rt(trans, rotation):
    if type(trans) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for trans: expected float")
    if type(rotation) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for rotation: expected float")

    ret = -1
    if __ROS2__:
        req = SetAccxRt.Request()
        req.trans = trans
        req.rotation = rotation

        while not _ros2_set_accx_rt.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Set Accx RT Service is not available, waiting...")

        future = _ros2_set_accx_rt.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('set_accx_rt Service call failed: %r' % (e,))
        else:
            if result is not None:
                ret = 0 if result.success else -1
    return ret

def read_data_rt():
    ret = None # Default to None
    # ROS2 service call
    if __ROS2__:
        req = ReadDataRt.Request()

        while not _ros2_read_data_rt.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Read Data RT Service is not available, waiting...")

        future = _ros2_read_data_rt.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('read_data_rt Service call failed: %r' % (e,))
        else:
            if result is not None:
                ret = result.data
            else:
                g_node.get_logger().error('read_data_rt Service call returned None')
    return ret

def write_data_rt(external_force_torque, external_digital_input, external_digital_output, external_analog_input, external_analog_output):
    if type(external_force_torque) not in (list, tuple) or len(external_force_torque) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for external_force_torque: expected list/tuple of 6 floats")
    if type(external_digital_input) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for external_digital_input: expected int")
    if type(external_digital_output) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for external_digital_output: expected int")
    if type(external_analog_input) not in (list, tuple) or len(external_analog_input) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for external_analog_input: expected list/tuple of 6 floats")
    if type(external_analog_output) not in (list, tuple) or len(external_analog_output) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type for external_analog_output: expected list/tuple of 6 floats")

    ret = -1
    # ROS2 service call
    if __ROS2__:
        req = WriteDataRt.Request()
        req.external_force_torque = external_force_torque
        req.external_digital_input = external_digital_input
        req.external_digital_output = external_digital_output
        req.external_analog_input = external_analog_input
        req.external_analog_output = external_analog_output

        while not _ros2_write_data_rt.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Write Data RT Service is not available, waiting...")

        future = _ros2_write_data_rt.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().error('write_data_rt Service call failed: %r' % (e,))
        else:
            if result is not None:
                ret = 0 if result.success else -1
            else:
                g_node.get_logger().error('write_data_rt Service call returned None')
    return ret

def movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=0)
    return ret
def amovej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=1)
    return ret
def _movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
    # _pos
    _pos = get_posj(pos)

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velj
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accj
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # _radius
    _radius = get_param(radius, r)
    if _radius == None:
        global _g_blend_state

        if _g_blend_state == True:
            _radius = _g_blend_radius
        else:
            _radius = 0.0

    if type(_radius) != int and type(_radius) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

    if _radius< 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # ra
    if type(ra) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

    if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__: 
        #SSS srv = _ros_movej(_pos, _vel[0], _acc[0], _time, mod, _radius, ra, _async)
        req = MoveJoint.Request()
        #for i in range(6):
        #    req.pos[i]      = pos[i] 
        req.pos         = [float(x) for x in _pos]
        req.vel         = float(_vel[0])
        req.acc         = float(_acc[0])
        req.time        = float(_time)
        req.mode        = int(mod)
        req.radius      = float(_radius)
        req.blend_type  = int(ra)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        while not _ros2_movej.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("MoveJ Service is not available, waiting for service to becom available...")
        
        future = _ros2_movej.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('movej Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:
                ret = 0 if (result.success == True) else -1                
    else:   
        ret = PythonMgr.py_movej(_pos, _vel, _acc, _time, _radius, mod, ra, _async)
    return ret


def movejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
    ret = _movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, _async=0)
    return ret 
def amovejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
    ret = _movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, _async=1)
    return ret 
def _movejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None, _async=0):
    # _pos
    _pos = get_posx(pos)

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velj
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accj
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")
    
    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # _radius
    _radius = get_param(radius, r)
    if _radius == None:
        global _g_blend_state

        if _g_blend_state == True:
            _radius = _g_blend_radius
        else:
            _radius = 0.0

    if type(_radius) != int and type(_radius) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

    if _radius < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

    # _ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # ra
    if type(ra) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type, ra")

    if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

    # sol
    if type(sol) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : sol")

    if sol < DR_SOL_MIN or sol > DR_SOL_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : sol")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__: 
        #srv = _ros_movejx(_pos, _vel[0], _acc[0], _time, mod, _ref, _radius, ra, sol, _async)   
        req = MoveJointx.Request()
        req.pos         = [float(x) for x in _pos]
        req.vel         = float(_vel[0])
        req.acc         = float(_acc[0])
        req.time        = float(_time)
        req.mode        = int(mod)
        req.ref         = int(_ref)
        req.radius      = float(_radius)
        req.blend_type  = int(ra)
        req.sol         = int(sol)
        req.sync_type   = int(_async)

        #ret = srv.success
        #ret = 0 if (srv.success == True) else -1
        while not _ros2_movejx.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("MoveJx Service is not available, waiting for service to becom available...")

        future = _ros2_movejx.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('movejx Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:    
        ret = PythonMgr.py_movejx(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, sol, _async)
        print_ext_result("{0} = PythonMgr.py_movejx(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref{6}, mod{7}, ra:{8}, sol:{9}, async:{10})" \
                         .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, sol, _async))   
    return ret

def movel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, _async=0)
    return ret
def amovel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, _async=1)
    return ret
def _movel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
    # _pos
    _pos = get_normal_pos(pos, def_type=posx)

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            #_vel = [temp] * DR_VELX_DT_LEN 
            _vel = [0] * DR_VELX_DT_LEN     
            _vel[0] = temp
            _vel[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    #print("_vel[0]={0}, _vel[1]={1}".format(_vel[0],_vel[1]))

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            #_acc = [temp] * DR_ACCX_DT_LEN 
            _acc = [0] * DR_ACCX_DT_LEN     
            _acc[0] = temp
            _acc[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    #print("_acc[0]={0}, _acc[1]={1}".format(_acc[0],_acc[1]))

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # _radius
    _radius = get_param(radius, r)
    if _radius == None:
        global _g_blend_state

        if _g_blend_state == True:
            _radius = _g_blend_radius
        else:
            _radius = 0.0

    if type(_radius) != int and type(_radius) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

    if _radius < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # ra
    if type(ra) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

    if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

    # qcommand
    if type(_pos) == posx:
        qcommand = 0
    else:
        qcommand = 1

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__: 
        req = MoveLine.Request()

        req.pos         = [float(x) for x in _pos]
        req.vel         = [float(x) for x in _vel]
        req.acc         = [float(x) for x in _acc]
        req.time        = float(_time)
        req.radius      = float(_radius)
        req.ref         = int(_ref)
        req.mode        = int(mod)
        req.blend_type  = int(ra)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        while not _ros2_movel.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Service is not available, waiting for service to becom available...")

        future = _ros2_movel.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('movel Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:    
        ret = PythonMgr.py_movel(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, qcommand, _async)
        print_ext_result("{0} = PythonMgr.py_movel(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref:{6}, mod:{7}, ra:{8}, qcommand:{9}, async:{10})" \
                         .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, qcommand, _async))
    return ret



def movec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
    ret = _movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, _async=0)
    return ret
def amovec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
    ret = _movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, _async=1)
    return ret
def _movec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None, _async=0):

    # _pos1, _pos2
    
    _pos1 = get_normal_pos(pos1, def_type=posx)
    _pos2 = get_normal_pos(pos2, def_type=posx)

    if type(_pos1) != type(_pos2):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos1, ps2")

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
            _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
            _vel[0] = temp
            _vel[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
            _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
            _acc[0] = temp
            _acc[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # _radius
    _radius = get_param(radius, r)
    if _radius == None:
        global _g_blend_state

        if _g_blend_state == True:
            _radius = _g_blend_radius
        else:
            _radius = 0.0

    if type(_radius) != int and type(_radius) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

    if _radius < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

    # _ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # _angle
    temp = get_param(angle, an)
    if temp == None:
        _angle = [0, 0]
    else:
        if type(temp) == int or type(temp) == float:
            _angle = [temp, 0]
        elif type(temp) == list and len(temp) == DR_ANGLE_DT_LEN:
            _angle = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : angle, an")

    if is_number(_angle) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : angle, an")

    for item in _angle:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : angle, an")

    # ra
    if type(ra) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

    if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

    # qcommand
    if type(_pos1) == posx:
        qcommand = 0
    else:
        qcommand = 1

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__:
        # make multi pos
        _circle_pos = _ros_listToFloat64MultiArray([_pos1, _pos2])
        #_circle_pos = _ros_listToFloat64MultiArray([ [float(x) for x in _pos1], [float(x) for x in _pos2] ])
        #print(_circle_pos)

        #srv = _ros_movec(_circle_pos, _vel, _acc, _time, mod, _ref, _angle[0], _angle[1], _radius, ra, _async) 
        req = MoveCircle.Request()
        req.pos         = _circle_pos               #??? 검증필요!
        req.vel         = [float(x) for x in _vel]
        req.acc         = [float(x) for x in _acc]
        req.time        = float(_time)
        req.radius      = float(_radius)
        req.ref         = int(_ref)
        req.mode        = int(mod)
        req.angle1      = float(_angle[0])
        req.angle2      = float(_angle[1])
        req.blend_type  = int(ra)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        while not _ros2_movec.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Service is not available, waiting for service to becom available...")

        future = _ros2_movec.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('movec Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:   
        ret = PythonMgr.py_movec(_pos1, _pos2, _vel, _acc, _time, _radius, _ref, mod, _angle, ra, qcommand, _async)
        print_ext_result("{0} = PythonMgr.py_movec(pos1:{1}, pos2:{2}, vel:{3}, acc:{4}, time:{5}, radius:{6}, ref:{7}, mod:{8}, angle:{9}, ra:{10}, qcommand:{11}, async:{12})" \
                         .format(ret, _pos1, _pos2, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, dr_form(_angle), ra, qcommand, _async))
    return ret


def movesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _movesj(pos_list, vel, acc, time, mod, v, a, t, _async=0)
    return ret
def amovesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _movesj(pos_list, vel, acc, time, mod, v, a, t, _async=1)
    return ret
def _movesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None, _async=0):
    # pos_list
    if type(pos_list) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list")

    for item in pos_list:
        if type(item) != posj:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list [item]")

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velj
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accj
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__:
        # make multi pos
        _spline_posj = _ros_listToFloat64MultiArray(pos_list)
        #srv = _ros_movesj(_spline_posj, len(_spline_posj), _vel, _acc, _time, mod, _async)

        req = MoveSplineJoint.Request()

        req.pos         = _spline_posj               #??? 검증필요!
        req.pos_cnt     = len(_spline_posj)
        req.vel         = [float(x) for x in _vel]
        req.acc         = [float(x) for x in _acc]
        req.time        = float(_time)
        req.mode        = int(mod)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        future = _ros2_movesj.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('movesj Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:    
        ret = PythonMgr.py_movesj(pos_list, _vel, _acc, _time, mod, _async)
        print_ext_result("{0} = PythonMgr.py_movesj(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, mod:{5} async:{6})" \
                         .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, mod, _async))
    return ret



def movesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
    ret = _movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, _async=0)
    return ret
def amovesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
    ret = _movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, _async=1)
    return ret
def _movesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None, _async=0):
    # pos_list
    if type(pos_list) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list")

    for item in pos_list:
        if type(item) != posx:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list [item]")

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
            _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
            _vel[0] = temp
            _vel[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
            _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
            _acc[0] = temp
            _acc[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # _ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # vel_opt
    if type(vel_opt) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel_opt")

    if vel_opt != DR_MVS_VEL_NONE and vel_opt != DR_MVS_VEL_CONST:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel_opt")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__:
        # make multi pos
        _spline_posx = _ros_listToFloat64MultiArray(pos_list)
        #srv = _ros_movesx(_spline_posx, len(_spline_posx), _vel, _acc, _time, mod, _ref, vel_opt, _async)
        req = MoveSplineTask.Request()
        req.pos         = _spline_posx               #??? 검증필요!
        req.pos_cnt     = len(_spline_posx)
        req.vel         = [float(x) for x in _vel]
        req.acc         = [float(x) for x in _acc]
        req.time        = float(_time)
        req.ref         = int(_ref)
        req.mode        = int(mod)
        req.opt         = int(vel_opt)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        future = _ros2_movesx.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('movesx Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:
        ret = PythonMgr.py_movesx(pos_list, _vel, _acc, _time, _ref, mod, vel_opt, _async)
        print_ext_result("{0} = PythonMgr.py_movesx(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, vel_opt:{7}, async:{8})" \
                        .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, vel_opt, _async))
    return ret

def moveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _moveb(seg_list, vel, acc, ref, time, mod, v, a, t, _async=0)
    return ret
def amoveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _moveb(seg_list, vel, acc, ref, time, mod, v, a, t, _async=1)
    return ret
def _moveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None, _async=0):

    # seg_list
    if type(seg_list) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_list")

    if len(seg_list) == 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : seg_list")

    _seg_list = []
    if __ROS2__:
        for seg in seg_list:
            _seg_list.append(seg.to_list())
    else:
        for seg in seg_list:
            if type(seg) != posb:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_list(item)")
            else:
                _seg_list.append(seg.to_list())

    if __ROS2__:
        print(len(_seg_list))
        _ros_seg_list = []
        _tmp_list = []

        for s in range(0, len(_seg_list)):
            #print(s)
            for elemnt in range(0, len(_seg_list[s])):
                if _seg_list[s][elemnt] == None:
                    _seg_list[s][elemnt] = [0.0]*POINT_COUNT
         
            # make [pos1] + [pos2] + [type] + [radius]
            _tmp_list = _seg_list[s][1] + _seg_list[s][2] + [_seg_list[s][0]] + [_seg_list[s][3]]
            _ros_seg_list.append(_tmp_list)
            
    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
            _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
            _vel[0] = temp
            _vel[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
            _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
            _acc[0] = temp
            _acc[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS Service call
    if __ROS2__:
        seg = _ros_listToFloat64MultiArray(_ros_seg_list)
        #srv = _ros_moveb(seg, len(_ros_seg_list), _vel, _acc, _time, mod, _ref, _async)    
        req = MoveBlending.Request()
        req.segment     = seg               #??? 검증필요!
        req.pos_cnt     = len(_ros_seg_list)
        req.vel         = [float(x) for x in _vel]
        req.acc         = [float(x) for x in _acc]
        req.time        = float(_time)
        req.ref         = int(_ref)
        req.mode        = int(mod)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        future = _ros2_moveb.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('moveb Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:
        ret = PythonMgr.py_moveb(_seg_list, _vel, _acc, _time, _ref, mod, _async)
        print_ext_result("{0} = PythonMgr.py_moveb(seg_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, async:{7})" \
                         .format(ret, dr_form(_seg_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, _async))
    return ret


def move_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
    ret = _move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, _async=0)
    return ret
def amove_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
    ret = _move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, _async=1)
    return ret
def _move_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None, _async=0):
    # rev
    if type(rev) != int and type(rev) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : rev")

    if rev <= 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : rev (Ranges: rev > 0)")

    # rmax
    if type(rmax) != int and type(rmax) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : rmax")

    if rmax <= 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value: rmax (Ranges: rmax > 0)")

    # lmax
    if type(lmax) != int and type(lmax) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : lmax")

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELX_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCX_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0 and _time != DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis")

    # ref
    if type(ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if ref != DR_BASE and ref != DR_TOOL and (ref < DR_TC_USER_MIN or ref > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__:
        #srv = _ros_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
        req = MoveSpiral.Request()
        req.revolution  = float(rev)
        req.max_radius  = float(rmax)
        req.max_length  = float(lmax)
        req.vel         = [float(x) for x in _vel]
        req.acc         = [float(x) for x in _acc]
        req.time        = float(_time)
        req.task_axis   = int(axis)
        req.ref         = int(ref)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        while not _ros2_move_spiral.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Service is not available, waiting for service to becom available...")

        future = _ros2_move_spiral.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('move_spiral Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    else:    
        ret = PythonMgr.py_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
        print_ext_result("{0} = PythonMgr.py_move_spiral(rev:{1}, rmax:{2}, lmax:{3}, vel:{4}, acc:{5}, time:{6}, axis:{7}, ref:{8}, async:{9})" \
                         .format(ret, dr_form(rev), dr_form(rmax), dr_form(lmax), dr_form(_vel), dr_form(_acc), _time, axis, ref, _async))    
    return ret

def move_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL):
    ret = _move_periodic(amp, period, atime, repeat, ref, _async=0)
    return ret
def amove_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL):
    ret = _move_periodic(amp, period, atime, repeat, ref, _async=1)
    return ret
def _move_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL, _async=0):
    _amp = [POINT_COUNT]
    _period = [POINT_COUNT]
    _atime =0.0
    _repeat =0.0
    _ref=0 

    # amp : float[6] 
    if type(amp) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : amp")
    if len(amp) != POINT_COUNT:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : amp")
    _amp =amp

    # period : float or float[6] 
    if (type(period) != int) and (type(period) != float) and (type(period) != list):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : period")

    if (type(period) == int) or (type(period) == float):
        _period = [period] * POINT_COUNT
    else: #list �� ��� 
        if len(period) != POINT_COUNT:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : period")
        _period = period

    # atime 
    if atime == None:
        _atime = 0.0
    else:
        if (type(atime) != int) and (type(atime) != float):
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : atime")
        if atime < 0.0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : atime")
        _atime = atime

    # repeat 
    if repeat == None:
        _repeat = 1
    else:
        if (type(repeat) != int) and (type(repeat) != float): 
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : repeat")
        if repeat < 0.0: 
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : repeat")
        _repeat = repeat

    # ref
    if ref == None:
        #_ref = _g_coord
        _ref = DR_TOOL
    else:
        if type(ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if ref < DR_BASE or ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")
        _ref = ref

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__:
        #srv = _ros_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)    
        req = MovePeriodic.Request()
        req.amp         = [float(x) for x in _amp]
        req.periodic    = [float(x) for x in _period]
        req.acc         = float(_atime)
        req.repeat      = int(_repeat)
        req.ref         = int(_ref)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        while not _ros2_move_periodic.wait_for_service(timeout_sec=1.0):
            g_node.get_logger().info("Service is not available, waiting for service to becom available...")
        
        future = _ros2_move_periodic.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('move_periodic Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    else:
        ret = PythonMgr.py_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)
        print_ext_result("{0} = PythonMgr.move_periodic(amp:{1}, period:{2}, atime:{3}, repeat{4}, ref:{5}, async:{6})" \
                         .format(ret, dr_form(_amp), dr_form(_period), _atime, _repeat, _ref, _async))
    return ret

def move_home(target=None):
    
    # target
    if target == None:
        _target = DR_HOME_TARGET_MECHANIC
    else:
        _target = target

    if __ROS2__:
        req = MoveHome.Request()    
        req.target = _target

        future = _ros2_move_home.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('move_home Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def mwait(time=0):
    ret = _move_wait(time)
    return ret
def _move_wait(time):
    if type(time) != int and type(time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

    if time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time")

    # ROS service call
    if __ROS2__:
        req = MoveWait.Request()  #ROS 에서는 time 인자를 사용하지 않음. 

        future = _ros2_move_wait.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('mwait Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def check_motion():

    if __ROS2__:
        req = CheckMotion.Request()  

        future = _ros2_check_motion.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('check_motion Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.status            
    return ret

def change_operation_speed(speed):
    if type(speed) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed")

    if speed < DR_OP_SPEED_MIN or speed > DR_OP_SPEED_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : speed({0})".format(speed))

    if __ROS2__:
        req = ChangeOperationSpeed.Request() 
        req.speed = speed

        future = _ros2_change_operation_speed.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('change_operation_speed Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def enable_alter_motion(n, mode, ref=None, limit_dPOS=None, limit_dPOS_per=None):
    # n
    if type(n) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : n")
    if n < 0:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : (n>=0)")
    _n = n #*20

    # mode
    if mode < DR_DPOS or mode > DR_DVEL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mode({0})".format(mode))
    _mode = mode

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref != DR_BASE and _ref != DR_TOOL and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    if(None==limit_dPOS):
        _limit_dPOS = [DR_COND_NONE, DR_COND_NONE]
    else:
        _limit_dPOS = limit_dPOS

    if(None==limit_dPOS_per):
        _limit_dPOS_per = [DR_COND_NONE, DR_COND_NONE]
    else:
        _limit_dPOS_per = limit_dPOS_per

    if __ROS2__:
        req = EnableAlterMotion.Request()  
        req.n              = _n 
        req.mode           = _mode
        req.ref            = _ref
        req.limit_dpos     = [float(x) for x in _limit_dPOS] 
        req.limit_dpos_per = [float(x) for x in _limit_dPOS_per]

        future = _ros2_enable_alter_motion.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('enable_alter_motion Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def alter_motion(dposx):

    # _dposx
    if (type(dposx) == posx):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : dposx")

    _dposx = get_normal_pos(dposx, def_type=posx)

    if __ROS2__:
        req = AlterMotion.Request()  
        req.pos = [float(x) for x in _dposx]

        future = _ros2_alter_motion.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('alter_motion Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def disable_alter_motion():

    if __ROS2__:
        req = DisableAlterMotion.Request()  

        future = _ros2_disable_alter_motion.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('disable_alter_motion Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def set_singular_handling(mode = DR_AVOID):
    if type(mode) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mode")

    if mode < DR_AVOID or mode > DR_VAR_VEL:    #DR_TASK_STOP
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mode({0})".format(mode))

    if __ROS2__:
        req = SetSingularityHandling.Request()  
        req.mode = mode

        future = _ros2_set_singularity_handling.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_singular_handling Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def set_singularity_handling(mode = DR_AVOID):
    ret = set_singular_handling(mode)
    return ret

def jog(jog_axis, ref=0, speed=0):
    if type(jog_axis) != int and type(jog_axis) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : jog_axis")
    
    if type(ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if type(speed) != int and type(speed) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed")
    # ROS service call
    if __ROS2__:
        req = Jog.Request()
        req.jog_axis       = jog_axis                            
        req.move_reference = ref
        req.speed          = float(speed)

        future = _ros2_jog.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('jog Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def jog_multi(jog_axis_list, ref=0, speed=0):
    if type(jog_axis_list) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : jog_axis_list")
   
    if type(ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if type(speed) != int and type(speed) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed")
    # ROS service call
    if __ROS2__:
        req = JogMulti.Request()
        req.jog_axis       = [float(x) for x in jog_axis_list]     
        req.move_reference = ref 
        req.speed          = float(speed) 

        future = _ros2_jog_multi.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('jog_multi Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret


##### force/stiffness Control #########################################################################################################################

def get_workpiece_weight():
    
    print("================================> get_workpiece_weight()")
    if __ROS2__:
        req = GetWorkpieceWeight.Request()  

        future = _ros2_get_workpiece_weight.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_workpiece_weight Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.weight            
    return ret


def reset_workpiece_weight():

    if __ROS2__:
        req = ResetWorkpieceWeight.Request()  

        future = _ros2_reset_workpiece_weight.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('reset_workpiece_weight Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def parallel_axis(*args, **kargs):
    len_args = len(args)
    len_kargs = len(kargs)
    _nType = 0

    # check kargs.key
    for key in kargs.keys():
        if key != "x1" and key != "x2" and key != "x3" and key != "vect" and key != "axis" and key != "ref":
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : x1, x2, x3, vect, axis, ref")

    # get kargs
    _x1 = get_kargs(kargs, "x1")
    _x2 = get_kargs(kargs, "x2")
    _x3 = get_kargs(kargs, "x3")
    _vect = get_kargs(kargs, "vect")
    _axis = get_kargs(kargs, "axis")
    _ref = get_kargs(kargs, "ref")

    # check parameter combination
    if len_args == 0:
        if len_kargs == 2:
            if _vect == None or _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_kargs == 3:
            _nType = 3
            if _vect == None or _axis == None or _ref == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_kargs == 4:
            if _x1 == None or _x2 == None or _x3 == None or _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_kargs == 5:
            _nType = 2
            if _x1 == None or _x2 == None or _x3 == None or _axis == None or _ref == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 1:
        if len_kargs != 1:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        _vect = args[0]

        if _axis == None:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 2:
        if len_kargs == 1:  # +ref
            _nType = 3
            _vect = args[0]
            _axis = args[1]
        else:
            if len_kargs != 0:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        _vect = args[0]
        _axis = args[1]

    elif len_args == 3:
        if len_kargs == 0:  # +ref
            _nType = 3
            _vect = args[0]
            _axis = args[1]
            _ref  = args[2]
        else:
            if len_kargs != 1:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]

            if _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 4:
        if len_kargs == 1:  # + ref
            _nType = 2
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _axis = args[3]
        else:
            if len_kargs != 0:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _axis = args[3]

    elif len_args == 5:
        if len_kargs == 0:  # + ref
            _nType = 2
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _axis = args[3]
            _ref = args[4]
        else:
            if len_kargs != 0:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _axis = args[3]
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    # check parameter type, value
    if _x1 != None:
        _x1 = get_posx(_x1)
        _x2 = get_posx(_x2)
        _x3 = get_posx(_x3)
    else:
        if type(_vect) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vect")
        if len(_vect) != DR_VECTOR_DT_LEN:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vect({0})".format(_vect))

    if type(_axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if _axis != DR_AXIS_X and _axis != DR_AXIS_Y and _axis != DR_AXIS_Z:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(_axis))

    if _nType == 2: # with ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        if __ROS2__:
            req = ParallelAxis1.Request()  
            req.x1   = [float(x) for x in _x1]
            req.x2   = [float(x) for x in _x2]
            req.x3   = [float(x) for x in _x3]
            req.axis = _axis
            req.ref  = _ref

            future = _ros2_parallel_axis1.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('parallel_axis Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

    elif _nType == 3: # with ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        if __ROS2__:
            req = ParallelAxis2.Request()  
            req.vect = [float(x) for x in _vect] 
            req.axis = _axis           
            req.ref  = _ref

            future = _ros2_parallel_axis2.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('parallel_axis Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

    else:   # 기존
        _ref = DR_BASE
        if _x1 != None:
            if __ROS2__:
                req = ParallelAxis1.Request()  
                req.x1   = [float(x) for x in _x1]
                req.x2   = [float(x) for x in _x2]
                req.x3   = [float(x) for x in _x3]
                req.axis = _axis
                req.ref  = _ref

                future = _ros2_parallel_axis1.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('parallel_axis Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = 0 if (result.success == True) else -1     


        else:
            if __ROS2__:
                req = ParallelAxis2.Request()  
                req._vect = vect
                req._axis = axis           
                req.ref   = _ref

                future = _ros2_parallel_axis2.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('parallel_axis Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = 0 if (result.success == True) else -1     

    return ret

def align_axis(*args, **kargs):
    len_args = len(args)
    len_kargs = len(kargs)
    _nType = 0

    # check kargs.key
    for key in kargs.keys():
        if key != "x1" and key != "x2" and key != "x3" and key != "vect" and key != "pos" and key != "axis" and key != "ref":
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    # get kargs
    _x1 = get_kargs(kargs, "x1")
    _x2 = get_kargs(kargs, "x2")
    _x3 = get_kargs(kargs, "x3")
    _vect = get_kargs(kargs, "vect")
    _pos = get_kargs(kargs, "pos")
    _axis = get_kargs(kargs, "axis")
    _ref = get_kargs(kargs, "ref")

    # check parameter combination
    if len_args == 0:
        if len_kargs == 3:
            if _vect == None or _pos == None or _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_kargs == 5:
            if _x1 == None or _x2 == None or _x3 == None or _pos == None or _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_kargs == 6:
            _nType = 2
            if _x1 == None or _x2 == None or _x3 == None or _pos == None or _axis == None or _ref == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_kargs == 4:
            _nType = 3
            if _vect == None or _pos == None or _axis == None or _ref == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 1:
        if len_kargs != 2:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        _vect = args[0]

        if _pos == None:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        if _axis == None:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 2:
        if len_kargs != 1:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        _vect = args[0]
        _pos = args[1]

        if _axis == None:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 3:
        if len_kargs == 0:
            _vect = args[0]
            _pos = args[1]
            _axis = args[2]
        elif len_kargs == 1:
            _nType = 3
            _vect = args[0]
            _pos = args[1]
            _axis = args[2]
        elif len_kargs == 2:
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]

            if _pos == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            if _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 4:
        if len_kargs == 0:
            _nType = 3
            _vect = args[0]
            _pos = args[1]
            _axis = args[2]
            _ref = args[3]
        else:
            if len_kargs != 1:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _pos = args[3]

            if _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    elif len_args == 5:
        if len_kargs == 0:
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _pos = args[3]
            _axis = args[4]
        elif len_kargs == 1:    # + ref
            _nType = 2
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _pos = args[3]
            _axis = args[4]

    elif len_args == 6:
        if len_kargs != 0:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        _nType = 2
        _x1 = args[0]
        _x2 = args[1]
        _x3 = args[2]
        _pos = args[3]
        _axis = args[4]
        _ref  = args[5]

    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    # check parameter type, value
    if _x1 != None:
        _x1 = get_posx(_x1)
        _x2 = get_posx(_x2)
        _x3 = get_posx(_x3)
    else:
        if type(_vect) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vect")
        if len(_vect) != DR_VECTOR_DT_LEN:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vect({0})".format(_vect))

    norm_pos = get_posx(_pos)

    if type(_axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")
    if _axis != DR_AXIS_X and _axis != DR_AXIS_Y and _axis != DR_AXIS_Z:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(_axis))

    if _nType == 2: # with ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        if __ROS2__:
            req = AlignAxis1.Request()  
            req.x1       = [float(x) for x in _x1]
            req.x2       = [float(x) for x in _x2]
            req.x3       = [float(x) for x in _x3]
            req.pos      = [float(x) for x in norm_pos]
            req.axis     = _axis
            req.ref      = _ref

            future = _ros2_align_axis1.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('align_axis Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     


    elif _nType == 3: # with ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        if __ROS2__:
            req = AlignAxis2.Request()  
            req.vect     = [float(x) for x in _vect]
            req.pos      = [float(x) for x in norm_pos]
            req.axis     = _axis
            req.ref      = _ref

            future = _ros2_align_axis2.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('align_axis Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     


    else:   # 기존
        _ref = DR_BASE
        if _x1 != None:
            if __ROS2__:
                req = AlignAxis1.Request()  
                req.x1       = [float(x) for x in _x1]
                req.x2       = [float(x) for x in _x2]
                req.x3       = [float(x) for x in _x3]
                req.pos      = [float(x) for x in norm_pos]
                req.axis     = _axis
                req.ref      = _ref

                future = _ros2_align_axis1.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('align_axis Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = 0 if (result.success == True) else -1     

        else:

            if __ROS2__:
                req = AlignAxis2.Request()  
                req.vect     = [float(x) for x in _vect]
                req.pos      = [float(x) for x in norm_pos]
                req.axis     = _axis
                req.ref      = _ref

                future = _ros2_align_axis2.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('align_axis Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = 0 if (result.success == True) else -1     

    return ret

def is_done_bolt_tightening(m=0, timeout=0, axis=None):
    # m
    if type(m) != int and type(m) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : m")

    # timeout
    if type(timeout) != int and type(timeout) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : timeout")

    if timeout < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : timeout({0})".format(timeout))

    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

    if __ROS2__:
        req = IsDoneBoltTightening.Request()  
        req.m       = float(m)
        req.timeout = float(timeout)
        req.axis    = axis

        future = _ros2_is_done_bolt_tightening.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)
        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('is_done_bolt_tightening Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def release_compliance_ctrl():

    if __ROS2__:
        req = ReleaseComplianceCtrl.Request()  

        future = _ros2_release_compliance_ctrl.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('release_compliance_ctrl Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def task_compliance_ctrl(stx=[3000, 3000, 3000, 200, 200, 200], time=0):
    # st
    if type(stx) != list or len(stx) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : stx")

    if is_number(stx) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : stx({0})".format(stx))

    # _ref
    _ref = _g_coord

    # time
    if type(time) != int and type(time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

    if time != -1 and time < 0:     # -1 : motion default
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))

    if time > 1:
        _time = 1
    else:
        _time = time

    if __ROS2__:
        req = TaskComplianceCtrl.Request()  
        req.stx  = [float(x) for x in stx]
        req.ref  = _ref      
        req.time = float(_time)

        future = _ros2_task_compliance_ctrl.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('task_compliance_ctrl Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret


def set_stiffnessx(stx=[500, 500, 500, 100, 100, 100], time=0):
    # st
    if type(stx) != list or len(stx) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : stx")

    if is_number(stx) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : stx({0})".format(stx))

    # _ref
    _ref = _g_coord

    # time
    if type(time) != int and type(time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

    if time != -1 and time < 0:     # -1 : motion default
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))

    if time > 1:
        _time = 1
    else:
        _time = time

    if __ROS2__:
        req = SetStiffnessx.Request()  
        req.stx  = [float(x) for x in stx]
        req.ref  = _ref      
        req.time = float(_time)

        future = _ros2_set_stiffnessx.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_stiffnessx Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def calc_coord(*args, **kargs):

    #arg(x1, ref, mod)                    arg =0 , kargs =3    _nType=1
    #arg(x1, x2, ref, mod)                arg =0 , kargs =4    _nType=2
    #arg(x1, x2, x3, ref, mod)            arg =0 , kargs =5    _nType=3
    #arg(x1, x2, x3, x4, ref, mod)        arg =0 , kargs =6    _nType=4

    #arg(?, ref, mod)                     arg =1 , kargs =2    _nType=1
    #arg(?, ?, ref, mod)                  arg =2 , kargs =2    _nType=2
    #arg(?, ?, ?, ref, mod)               arg =3 , kargs =2    _nType=3
    #arg(?, ?, ?, ?, ref, mod)            arg =4 , kargs =2    _nType=4

    #arg(?, ?, ?)                         arg =3 , kargs =0    _nType=1
    #arg(?, ?, ?, ?)                      arg =4 , kargs =0    _nType=2
    #arg(?, ?, ?, ?, ?)                   arg =5 , kargs =0    _nType=3
    #arg(?, ?, ?, ?, ?, ?)                arg =6 , kargs =0    _nType=4

    len_args = len(args)
    len_kargs = len(kargs)
    _cnt_pos = 0

    # check kargs.key
    for key in kargs.keys():
        if key != "x1" and key != "x2" and key != "x3" and key != "x4" and key != "ref" and key != "mod":
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : x1, x2, x3, x4, ref, mod")

    # get kargs
    _x1 = get_kargs(kargs, "x1")
    _x2 = get_kargs(kargs, "x2")
    _x3 = get_kargs(kargs, "x3")
    _x4 = get_kargs(kargs, "x4")
    _ref = get_kargs(kargs, "ref")
    _mod = get_kargs(kargs, "mod")

    if(_x1 == None):
        _x1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if(_x2 == None):
        _x2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if(_x3 == None):
        _x3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if(_x4 == None):
        _x4 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    if len_args == 0 and len_kargs ==3:
        _cnt_pos = 1
        print("x1={}, ref={}, mod={}".format(_x1, _ref, _mod))
    elif len_args == 0 and len_kargs ==4:
        _cnt_pos = 2
        print("x1={}, x2={}, ref={}, mod={}".format(_x1, _x2, _ref, _mod))
    elif len_args == 0 and len_kargs ==5:
        _cnt_pos = 3
        print("x1={}, x2={}, x3={}, ref={}, mod={}".format(_x1, _x2, _x3, _ref, _mod))
    elif len_args == 0 and len_kargs ==6:
        _cnt_pos = 4
        print("x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))

    elif len_args == 1 and len_kargs ==2:
        _cnt_pos = 1
        _x1 = args[0]
        print("x1={}, ref={}, mod={}".format(_x1, _ref, _mod))
    elif len_args == 2 and len_kargs ==2:
        _cnt_pos = 2
        _x1 = args[0]
        _x2 = args[1]
        print("x1={}, x2={}, ref={}, mod={}".format(_x1, _x2, _ref, _mod))
    elif len_args == 3 and len_kargs ==2:
        _cnt_pos = 3
        _x1 = args[0]
        _x2 = args[1]
        _x3 = args[2]
        print("x1={}, x2={}, x3={}, ref={}, mod={}".format(_x1, _x2, _x3, _ref, _mod))
    elif len_args == 4 and len_kargs ==2:
        _cnt_pos = 4
        _x1 = args[0]
        _x2 = args[1]
        _x3 = args[2]
        _x4 = args[3]
        print("x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))

    elif len_args == 3 and len_kargs ==0:
        _cnt_pos = 1
        _x1 = args[0]
        _ref =args[1]
        _mod =args[2]
        print("x1={}, ref={}, mod={}".format(_x1, _ref, _mod))
    elif len_args == 4 and len_kargs ==0:
        _cnt_pos = 2
        _x1 = args[0]
        _x2 = args[1]
        _ref =args[2]
        _mod =args[3]
        print("x1={}, x2={}, ref={}, mod={}".format(_x1, _x2, _ref, _mod))
    elif len_args == 5 and len_kargs ==0:
        _cnt_pos = 3
        _x1 = args[0]
        _x2 = args[1]
        _x3 = args[2]
        _ref =args[3]
        _mod =args[4]
        print("x1={}, x2={}, x3={}, ref={}, mod={}".format(_x1, _x2, _x3, _ref, _mod))
    elif len_args == 6 and len_kargs ==0:
        _cnt_pos = 4
        _x1 = args[0]
        _x2 = args[1]
        _x3 = args[2]
        _x4 = args[3]
        _ref =args[4]
        _mod =args[5]
        print("x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid arguments")

    print("FINAL : x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))

    # check parameter type, value
    if _x1 != None: 
        _x1 = get_posx(_x1)
    if _x2 != None: 
        _x2 = get_posx(_x2)
    if _x3 != None: 
        _x3 = get_posx(_x3)
    if _x4 != None: 
        _x4 = get_posx(_x4)
    if _ref !=None:
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
    if _mod != None:
        if _mod != 0 and _mod != 1:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0})".format(_mod))

    if __ROS2__:
        req = CalcCoord.Request()  
        req.input_pos_cnt = _cnt_pos
        req.x1      = [float(x) for x in _x1]
        req.x2      = [float(x) for x in _x2]
        req.x3      = [float(x) for x in _x3]
        req.x4      = [float(x) for x in _x4]
        req.ref     = _ref
        req.mod     = _mod

        future = _ros2_calc_coord.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('calc_coord Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                pos = list(result.conv_posx)  # Convert tuple to list 
                conv_posx = posx(pos)
                ret = pos

    return ret

def set_user_cart_coord(*args, **kargs):
    len_args = len(args)
    len_kargs = len(kargs)
    _nType =0  

    # check kargs.key
    for key in kargs.keys():
        if key != "x1" and key != "x2" and key != "x3" and key != "pos" and key != "u1" and key != "v1" and key != "v1" and key != "ref":
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : x1, x2, x3, pos, u1, v1, ref")

    # get kargs
    _x1 = get_kargs(kargs, "x1")
    _x2 = get_kargs(kargs, "x2")
    _x3 = get_kargs(kargs, "x3")
    _pos= get_kargs(kargs, "pos")
    _u1 = get_kargs(kargs, "u1")
    _v1 = get_kargs(kargs, "v1")
    _ref = get_kargs(kargs, "ref")

    # check parameter combination
    #set_user_cart_coord(x1=?, x2=?, x3=?, pos=?)   arg =0 , kargs =4    _nType=0
    #set_user_cart_coord(u1=?, v1=?, pos=?)         arg =0 , kargs =3 *  _nType=1
    #set_user_cart_coord(x1=?, x2=?, x3=?)          arg =0 , kargs =3 *  _nType=2

    #set_user_cart_coord(x1=?, x2=?, x3=?, ?)       arg =1 , kargs =3    _nType=0
    #set_user_cart_coord(u1=?, v1=?, ?)             arg =1 , kargs =2    _nType=1

    #set_user_cart_coord(?, ?, pos=?)               arg =2 , kargs =1    _nType=1

    #set_user_cart_coord(?, ?, ?, pos=?)            arg =3 , kargs =1    _nType=0
    #set_user_cart_coord(?, ?, ?)                   arg =3 , kargs =0 *  _nType=1
    #set_user_cart_coord(?, ?, ?)                   arg =3 , kargs =0 *  _nType=2  

    #set_user_cart_coord(?, ?, ?, ?)                arg =4 , kargs =0    _nType=0

#----- 신규 추가 명령 2019/11/27 ----------------------------------------------------------------------------------------------------------------
    #set_user_cart_coord(pos=?, ref=?)         arg =0 , kargs =2    _nType=5
    #set_user_cart_coord(pos=?, ?)             arg =1 , kargs =1 *  _nType=5 #python syntax error : positional argument follows keyword argument
    #set_user_cart_coord(?, ref=?)             arg =1 , kargs =1 *  _nType=5
    #set_user_cart_coord(?, ?)                 arg =2 , kargs =0    _nType=5
#------------------------------------------------------------------------------------------------------------------------------------------------

    if len_args == 0 and len_kargs ==2:
        print("new commnad  len_args == 0 and len_kargs ==2")
        #_pos
        #_ref
        print("_pos={},_ref={}".format(_pos, _ref))
        _nType = 5
    elif len_args == 1 and len_kargs ==1:
        print("new commnad  len_args == 1 and len_kargs ==1")
        _pos = args[0]
        #_ref
        print("_pos={},_ref={}".format(_pos, _ref))
        _nType = 5
    elif len_args == 2 and len_kargs ==0:
        print("new commnad  len_args == 2 and len_kargs ==0")
        _pos = args[0]
        _ref = args[1]
        print("_pos={},_ref={}".format(_pos, _ref))
        _nType = 5
    else:
        if len_args == 0:
            if len_kargs == 5:
                if _x1 == None or _x2 == None or _x3 == None or _pos == None or _ref == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                _nType = 3
            elif len_kargs == 4:
                if _ref != None:
                    _nType = 4
                else:
                    if _x1 == None or _x2 == None or _x3 == None or _pos == None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    _nType = 0
            elif len_kargs == 3:
                if _u1 != None: 
                    if _v1 == None or _pos == None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    _nType = 1
                elif _x1 != None:
                    if _x2 == None or _x3 == None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    _nType = 2
                else: 
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_args == 1:
            if len_kargs == 3:
                if _x1 == None or _x2 == None or _x3 == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                _nType = 0
                _pos= args[0]
            elif len_kargs == 2:
                if _u1 == None or _v1 == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                _nType = 1
                _pos= args[0]
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_args == 2:
            if len_kargs == 1:
                if _pos != None: 
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                else:
                    _nType = 1
                    _u1 = args[0]
                    _v1 = args[1]
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_args == 3:
            if len_kargs == 1:
                if _ref != None:
                    _nType = 4
                    _u1 = args[0]
                    _v1 = args[1]
                    _pos = args[2]
                else:
                    if _pos != None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    else:
                        _nType = 0
                        _x1 = args[0]
                        _x2 = args[1]
                        _x3 = args[2]
            elif len_kargs == 0:
                if len(args[0]) == 3:
                    _nType = 1
                    _u1 = args[0]
                    _v1 = args[1]
                    _pos= args[2]
                elif len(args[0]) == 6:
                    _nType = 2
                    _x1 = args[0]
                    _x2 = args[1]
                    _x3 = args[2]
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_args == 4:
            if len_kargs == 0:
                _nType = 0
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _pos= args[3]
            elif len_kargs == 1:
                _nType = 3
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _pos= args[3]
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        elif len_args == 5:
            if len_kargs == 0:
                _nType = 0
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _pos= args[3]
                _ref= args[4]
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    # check parameter type, value
    if _x1 != None: 
        _x1 = get_posx(_x1)
    if _x2 != None: 
        _x2 = get_posx(_x2)
    if _x3 != None: 
        _x3 = get_posx(_x3)
    if _pos != None: 
        _pos = get_posx(_pos)
    if _u1 !=None:
        if type(_u1) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : u1")
        if len(_u1) != DR_VECTOR_U1_LEN:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : u1({0})".format(_u1))
    if _v1 !=None:
        if type(_v1) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : v1")
        if len(_v1) != DR_VECTOR_V1_LEN:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : v1({0})".format(_v1))

    if _nType == 0:
        _ref = DR_BASE
        if __ROS2__:
            req = SetUserCartCoord2.Request()  
            req.x1  = [float(x) for x in _x1]
            req.x2  = [float(x) for x in _x2]
            req.x3  = [float(x) for x in _x3]
            req.pos = [float(x) for x in _pos]
            req.ref = _ref 

            future = _ros2_set_user_cart_coord2.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_user_cart_coord Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.id            


    elif _nType == 1:
        _ref = DR_BASE
        if __ROS2__:
            req = SetUserCartCoord3.Request()  
            req.u1  = [float(x) for x in _u1]
            req.v1  = [float(x) for x in _v1]
            req.pos = [float(x) for x in _pos]
            req.ref = _ref 

            future = _ros2_set_user_cart_coord3.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)
    
            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_user_cart_coord Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.id            

    elif _nType == 2:   #현재 미사용
        #_ref = DR_BASE
        #if __:
        #    req = set_user_cart_coord?(_x1, _x2, _x3, _ref)  
        #    ret = srv.id
        #else:
        #    ret = PythonMgr.py_set_user_cart_coord_ex2(_x1, _x2, _x3, _ref)
        #    #print_ext_result("{0} = PythonMgr.py_set_user_cart_coord_ex2(x1:{1}, x2:{2}, x3:{3}, ref:{4})".format(ret, _x1, _x2, _x3, _ref))
        pass
    elif _nType == 3:   # with ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        if __ROS2__:
            req = SetUserCartCoord2.Request()  
            req.x1 = [float(x) for x in _x1]
            req.x2 = [float(x) for x in _x2]
            req.x3 = [float(x) for x in _x3]
            req.pos = [float(x) for x in _pos]
            req.ref = _ref 

            future = _ros2_set_user_cart_coord2.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)
    
            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_user_cart_coord Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.id            

    elif _nType == 4:   # with ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        if __ROS2__:
            req = SetUserCartCoord3.Request()  
            req.u1  = [float(x) for x in _u1]
            req.v1  = [float(x) for x in _v1]
            req.pos = [float(x) for x in _pos]
            req.ref = _ref 


            future = _ros2_set_user_cart_coord3.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)
    
            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_user_cart_coord Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.id            

    elif _nType == 5:   # (pos, ref) 20191127 신규 추가 
        #print("new command !!!!!!!!!!!!!!!!")
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        if __ROS2__:
            req = SetUserCartCoord1.Request()  
            req.pos = [float(x) for x in _pos]
            req.ref = _ref 

            future = _ros2_set_user_cart_coord1.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)
    
            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_user_cart_coord Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.id            

    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

    return ret

def overwrite_user_cart_coord(id, pos, ref=None, gol=None):

    # id
    _id = id

    # pos
    _pos = get_posx(pos)

    # ref
    if ref == None:
        _ref = DR_BASE
    else:
        _ref = ref
        if _ref != DR_BASE and _ref != DR_WORLD:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    # gol : #1 RCT 요청에 의하여 gol 무조건 0으로 변경 2019/12/09
    #if gol == None:
    #    _gol = 0
    #else:
    #    _gol = gol
    #    if _gol != 0 and _gol != 1:
    #        raise DR_Error(DR_ERROR_VALUE, "Invalid value : gol")
    _gol = 0

    if __ROS2__:
        #req = overwrite_user_cart_coord(_id, _pos, _ref, _gol)  
        req = OverwriteUserCartCoord.Request()
        req.id  = _id
        req.pos = [float(x) for x in _pos]
        req.ref = _ref 

        future = _ros2_overwrite_user_cart_coord.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('overwrite_user_cart_coord Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.id            

    return ret

def get_user_cart_coord(id):

    print("==============================> get_user_cart_coord(id)")
    if __ROS2__:
        req = GetUserCartCoord.Request()  
        req.id  = id

        future = _ros2_get_user_cart_coord.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_user_cart_coord Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1, -1    
            else:        
                pos = list(result.conv_posx)  # Convert tuple to list 
                ref = result.ref
                conv_posx = posx(pos)
                print("==============================> get_user_cart_coord(id) 555 conv_posx={}, ref={}".format(conv_posx, ref))
                return conv_posx, ref
    return 0

def set_desired_force(fd=[0, 0, 0, 0, 0, 0], dir=[0, 0, 0, 0, 0, 0], time=0, mod=DR_FC_MOD_ABS):
    # df
    if type(fd) != list or len(fd) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : fd")

    if is_number(fd) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : fd({0})".format(fd))

    # dir
    if type(dir) != list or len(dir) != 6:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : dir")

    if is_number(dir) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : dir({0})".format(dir))

    # _ref
    _ref = _g_coord

    # time
    if type(time) != int and type(time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

    if time != -1 and time < 0:     # -1 : motion default
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))

    if time > 1:
        _time = 1
    else:
        _time = time

    # _mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    _mod = mod

    if __ROS2__:
        req = SetDesiredForce.Request()  
        req.fd   = [float(x) for x in fd]
        req.dir  = dir
        req.ref  = _ref
        req.time = float(_time)
        req.mod  = _mod

        future = _ros2_set_desired_force.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_desired_force Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret


def release_force(time=0):
    if type(time) != int and type(time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

    if time != -1 and time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))

    if time > 1:
        _time = 1
    else:
        _time = time

    if __ROS2__:
        req = ReleaseForce.Request() 
        req.time = float(_time)

        future = _ros2_release_force.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('release_force Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret


#def check_position_condition(axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None):
def check_position_condition(axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None, mod= DR_MV_MOD_ABS, pos=None):
    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if (axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

    # min
    if type(min) != int and type(min) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")

    # max
    if type(max) != int and type(max) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")

    if min == DR_COND_NONE and max == DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min,max))

    # min < max check
    if min != DR_COND_NONE and max != DR_COND_NONE: #min, max 값이 둘다 설정된 경우에만 검사 실시 2017/08/23
        if min > max:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min,max))

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0})".format(mod))

    # _pos : DR_MV_MOD_REL 인 경우에는 반드시 pos 가 필요하고, otherwise 불필요
    if(mod == DR_MV_MOD_REL):
        if(pos==None):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : need to pos")
        else:
            _pos = get_normal_pos(pos, def_type=posx)
    else:
        if(pos!=None):
            _pos = get_normal_pos(pos, def_type=posx)
        else:
            pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            _pos = get_normal_pos(pos, def_type=posx)

    # check axis
    if (axis == DR_AXIS_A or axis == DR_AXIS_B or axis == DR_AXIS_C):
        _ref = DR_TOOL

    if __ROS2__:
        req = CheckPositionCondition.Request()  
        req.axis = axis
        req.min  = float(min)
        req.max  = float(max)
        req.ref  = _ref
        req.mode = mod
        req.pos = [float(x) for x in _pos]

        future = _ros2_check_position_condition.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('check_position_condition Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret



def check_force_condition(axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None):
    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if (axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z and \
        axis != DR_AXIS_A and axis != DR_AXIS_B and axis != DR_AXIS_C):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

    # min
    if type(min) != int and type(min) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")

    # max
    if type(max) != int and type(max) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")

    if min == DR_COND_NONE and max == DR_COND_NONE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))

    # min check : min 설정되어 있는데 0보다 작은 경우 에러 처리 2017/12/07
    if min != DR_COND_NONE:
        if min < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: min (Ranges: min({0}) >= 0)".format(min))

    # max check : max 설정되어 있는데 0보다 작은 경우 에러 처리 2017/12/07
    if max != DR_COND_NONE:
        if max < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: max (Ranges: max({0}) >= 0)".format(max))

    # min < max check
    if min != DR_COND_NONE and max != DR_COND_NONE: #min, max 값이 둘다 설정된 경우에만 검사 실시 2017/08/23
        if min > max:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

    # check axis
    if (axis == DR_AXIS_A or axis == DR_AXIS_B or axis == DR_AXIS_C):
        _ref = DR_TOOL

    if __ROS2__:
        req = CheckForceCondition.Request()  
        req.axis = axis
        req.min  = float(min)
        req.max  = float(max)
        req.ref  = _ref

        future = _ros2_check_force_condition.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('check_force_condition Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret


def check_orientation_condition(axis, min=None, max=None, ref=None, mod = None, pos=None):
    _cmd_type = 0

    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if (axis != DR_AXIS_A and axis != DR_AXIS_B and axis != DR_AXIS_C):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

    # min, max check type
    if(min != None):
        if type(min) == posx:
            _cmd_type = 0
        elif type(min) == posj:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : min is not posj")
        elif type(min) == list and len(min) == POINT_COUNT:
            _cmd_type = 0
        elif type(min) == int or type(min) == float:
            _cmd_type = 1
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")
    if(max != None):
        if type(max) == posx:
            _cmd_type = 0
        elif type(max) == posj:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : max is not posj")
        elif type(max) == list and len(max) == POINT_COUNT:
            _cmd_type = 0
        elif type(max) == int or type(max) == float:
            _cmd_type = 1
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")

    if _cmd_type == 0:

        # _pos_min
        if(min != None):
            _pos_min = get_normal_pos(min, def_type=posx)
        else:
            _pos_min = [DR_COND_NONE]*POINT_COUNT

        # _pos_max
        if(max != None):
            _pos_max = get_normal_pos(max, def_type=posx)
        else:
            _pos_max = [DR_COND_NONE]*POINT_COUNT

        # _ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        # mod
        if mod == None:
            mod = DR_MV_MOD_ABS
        else:
            if type(mod) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod is not integer")

            if mod != DR_MV_MOD_ABS:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0}) is not DR_MV_MOD_ABS".format(mod))

        print("_cmd_type={0}, axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5}".format(_cmd_type, axis, _pos_min, _pos_max, _ref, mod) )

    elif _cmd_type == 1:
        # min
        if min == None:
            min = DR_COND_NONE
        else:
            if type(min) != int and type(min) != float:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")
        # max
        if max == None:
            max = DR_COND_NONE
        else:
            if type(max) != int and type(max) != float:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")

        if min == DR_COND_NONE and max == DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))

        # min < max check
        if min != DR_COND_NONE and max != DR_COND_NONE: #min, max 값이 둘다 설정된 경우에만 검사 실시 2017/08/23
            if min > max:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))

        # _ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref
    
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        # mod
        if mod == None:
            mod = DR_MV_MOD_REL
        else:
            if type(mod) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod is not integer")

            if mod != DR_MV_MOD_REL:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0}) is not DR_MV_MOD_REL".format(mod))

        # _pos
        if(pos==None):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : need to pos")
        else:
            _pos = get_normal_pos(pos, def_type=posx)

        #print("_cmd_type={0}, axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5}, pos:{6}".format(_cmd_type, axis, min, max, _ref, mod, _pos) )

    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid arguments")


    if _cmd_type == 0:
        if __ROS2__:
            req = CheckOrientationCondition1.Request()  
            req.axis    = axis
            req.min = [float(x) for x in _pos_min]
            req.max = [float(x) for x in _pos_max]
            req.ref     = _ref
            req.mode    = mod

            future = _ros2_check_orientation_condition1.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('check_orientation_condition Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
    else:
        if __ROS2__:
            req = CheckOrientationCondition2.Request()  
            req.axis = axis
            req.min  = float(min)
            req.max  = float(max)
            req.ref  = _ref
            req.mode = mod
            req.pos = [float(x) for x in _pos]

            future = _ros2_check_orientation_condition2.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('check_orientation_condition Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

    return ret

def coord_transform(pose_in, ref_in=DR_BASE, ref_out=None):

    # _pos
    _pos = get_normal_pos(pose_in, def_type=posx)

    # ref_in
    if type(ref_in) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref_in")

    # ref_out
    if type(ref_out) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref_out")

    if __ROS2__:
        req = CoordTransform.Request()    
        req.pos_in  = [float(x) for x in _pos]
        req.ref_in  = ref_in
        req.ref_out = ref_out               

        future = _ros2_coord_transform.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('coord_transform Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                pos = list(result.conv_posx)  # Convert tuple to list 
                trans_posx = posx(pos)
                ret = trans_posx
    return ret

##### I/O #########################################################################################################################

def get_digital_input(index):
    
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

    if __ROS2__:
        req = GetCtrlBoxDigitalInput.Request()
        req.index  = index

        future = _ros2_get_digital_input.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_digital_input Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.value 

    return ret

def get_analog_input(ch):
    if type(ch) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")

    if ch < 1 or ch > 2:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")

    if __ROS2__:
        req = GetCtrlBoxAnalogInput.Request()
        req.channel  = ch

        future = _ros2_get_analog_input.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_analog_input Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.value 

    return ret

def get_tool_digital_input(index):
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if index < DR_TDIO_MIN_INDEX or index > DR_TDIO_MAX_INDEX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

    if __ROS2__:
        req = GetToolDigitalInput.Request()
        req.index  = index

        future = _ros2_get_tool_digital_input.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_tool_digital_input Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.value 

    return ret

def set_digital_output(index, val=None):
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if val != None:
        if type(val) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")
    
        if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
    
        if val != ON and val != OFF:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
    else: # # val ���ڰ� ������ simple style
        if (index < (-DR_DIO_MAX_INDEX)) or (index > DR_DIO_MAX_INDEX) or (index==0): # -16~+16
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
        else:
            if index < 0:
                index = index*(-1) 
                val = OFF
            else:
                index = index
                val = ON

    if __ROS2__:
        req = SetCtrlBoxDigitalOutput.Request() 
        req.index = index
        req.value = val

        future = _ros2_set_digital_output.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_digital_output Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def get_digital_output(index):
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

    if __ROS2__:
        req = GetCtrlBoxDigitalOutput.Request()
        req.index = index

        future = _ros2_get_digital_output.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_digital_output Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.value 

    return ret


def set_analog_output(ch, val):
    global _g_analog_output_mode_ch1
    global _g_analog_output_mode_ch2

    if type(ch) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")

    if type(val) != int and type(val) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")

    if ch < 1 or ch > 2:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")

    #if val < 0 or val > 20.0:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
    if ch == 1:
        if _g_analog_output_mode_ch1 == DR_ANALOG_CURRENT:
            if val < 4 or val > 20.0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        elif _g_analog_output_mode_ch1 == DR_ANALOG_VOLTAGE:
            if val < 0 or val > 10.0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        else: 
            raise DR_Error(DR_ERROR_VALUE, "Analog output mode(ch1) is not set")
    if ch == 2:
        if _g_analog_output_mode_ch2 == DR_ANALOG_CURRENT:
            if val < 4 or val > 20.0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        elif _g_analog_output_mode_ch2 == DR_ANALOG_VOLTAGE:
            if val < 0 or val > 10.0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        else: 
            raise DR_Error(DR_ERROR_VALUE, "Analog output mode(ch2) is not set")

    if __ROS2__:
        req = SetCtrlBoxAnalogOutput.Request() 
        req.channel = ch
        req.value   = float(val)

        future = _ros2_set_analog_output.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_analog_output Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def set_mode_analog_output(ch, mod):
    global _g_analog_output_mode_ch1
    global _g_analog_output_mode_ch2

    if type(ch) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")

    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if ch < 1 or ch > 2:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")

    if mod != DR_ANALOG_CURRENT and mod !=DR_ANALOG_VOLTAGE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    if ch == 1: 
        _g_analog_output_mode_ch1 = mod
    if ch == 2: 
        _g_analog_output_mode_ch2 = mod

    if __ROS2__:
        req = SetCtrlBoxAnalogOutputType.Request()
        req.channel  = ch
        req.mode = mod

        future = _ros2_set_mode_analog_output.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_mode_analog_output Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def set_mode_analog_input(ch, mod):
    if type(ch) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")

    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if ch < 1 or ch > 2:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")

    if mod != DR_ANALOG_CURRENT and mod !=DR_ANALOG_VOLTAGE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    if __ROS2__:
        req = SetCtrlBoxAnalogInputType.Request()
        req.channel  = ch
        req.mode = mod

        future = _ros2_set_mode_analog_input.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_mode_analog_input Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret


def set_tool_digital_output(index, val=None):
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if val != None:
        if type(val) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")

        if index < DR_TDIO_MIN_INDEX or index > DR_TDIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

        if val != ON and val != OFF:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
    else: # val ���ڰ� ������ simple style
        if (index < (-DR_TDIO_MAX_INDEX)) or (index > DR_TDIO_MAX_INDEX) or (index==0): # -6~+6
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
        else:
            if index < 0:
                index = index*(-1) 
                val = OFF
            else:
                index = index
                val = ON

    if __ROS2__:
        req = SetToolDigitalOutput.Request()   
        req.index = index
        req.value = val

        future = _ros2_set_tool_digital_output.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_tool_digital_output Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def get_tool_digital_output(index):
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if index < DR_TDIO_MIN_INDEX or index > DR_TDIO_MAX_INDEX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

    if __ROS2__:
        req = GetToolDigitalOutput.Request()
        req.index = index


        future = _ros2_get_tool_digital_output.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_tool_digital_output Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.value 

    return ret


##### Modbus #########################################################################################################################

def add_modbus_signal(ip, port, name, reg_type, index, value=0, slaveid=255):
    # ip
    if type(ip) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ip")

    #try:
    #    ipaddress.ip_address(ip)
    #except:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : ip")

    # port
    if type(port) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : port")

    if port <= 0 or port > 65535:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : port (Ranges: 1 ~ 65535)")

    # name
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # reg_type
    if type(reg_type) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : reg_type")

    if reg_type != DR_MODBUS_DIG_INPUT and reg_type != DR_MODBUS_DIG_OUTPUT and reg_type != DR_MODBUS_REG_INPUT and reg_type != DR_MODBUS_REG_OUTPUT:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : reg_type (Ranges : 0 ~ 3")

    # index
    if type(index) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

    if index < 0 or index > 65535:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : index (Ranges: 0 ~ 65535)")

    # value
    if type(value) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : value")

    if value < 0 or value > 65535:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : value (Ranges: 0 ~ 65535)")

    # check value
    if reg_type == DR_MODBUS_DIG_OUTPUT or reg_type == DR_MODBUS_REG_OUTPUT:
        _value = value
    else:
        _value = 0

    # slaveid
    if type(slaveid) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : slaveid")
    if slaveid < 0 or slaveid > 255:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : slaveid (Ranges: 0 or 1~247 or 255)")
    elif slaveid > 247 and slaveid < 255:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : slaveid (Ranges: 0 or 1~247 or 255)")

    # ROS service call
    if __ROS2__:
        req = ConfigCreateModbus.Request()
        req.name     = name
        req.ip       = ip
        req.port     = port
        req.reg_type = reg_type
        req.index    = index
        req.value    = _value
        req.slave_id = slaveid

        future = _ros2_add_modbus_signal.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('add_modbus_signal Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def del_modbus_signal(name):
    # name
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS2__:
        req = ConfigDeleteModbus.Request() 
        req.name = name

        future = _ros2_del_modbus_signal.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('del_modbus_signal Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def set_modbus_output(iobus, val):
    
    if type(iobus) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : iobus")

    if type(val) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")

    # not check value (register : 2byte...)
    # if val != ON and val != OFF:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
   
    # ROS service call
    if __ROS2__:
        req = SetModbusOutput.Request() 
        req.name  = iobus
        req.value = val

        future = _ros2_set_modbus_output.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_modbus_output Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def get_modbus_input(iobus):
    if type(iobus) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : iobus")

    # ROS service call
    if __ROS2__:
        req = GetModbusInput.Request()
        req.name  = iobus

        future = _ros2_get_modbus_input.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_modbus_input Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.value 

    return ret

##### TCP/TOOL #########################################################################################################################
def set_tcp(name):

    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # not check value (register : 2byte...)
    # if val != ON and val != OFF:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")

    # ROS service call
    if __ROS2__:
        req = SetCurrentTcp.Request()
        req.name = name

        future = _ros2_set_current_tcp.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_tcp Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def get_tcp():
    # ROS service call
    if __ROS2__:
        req = GetCurrentTcp.Request()        

        future = _ros2_get_current_tcp.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_tcp Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.info 

    return ret

def set_tool(name):

    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # not check value (register : 2byte...)
    # if val != ON and val != OFF:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")

    # ROS service call
    if __ROS2__:
        req = SetCurrentTool.Request()    
        req.name = name

        future = _ros2_set_current_tool.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_tool Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret


def get_tool():
    # ROS service call
    if __ROS2__:
        req = GetCurrentTool.Request() 

        future = _ros2_get_current_tool.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_tool Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.info 

    return ret

def set_tool_shape(name):

    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    if __ROS2__:
        req = SetToolShape.Request()  
        req.name = name

        future = _ros2_set_tool_shape.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_tool_shape Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def add_tcp(name, pos):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS2__:
        req = ConfigCreateTcp.Request()  
        req.name = name
        req.pos = [float(x) for x in pos]

        future = _ros2_config_create_tcp.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('add_tcp Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def del_tcp(name):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS2__:
        req = ConfigDeleteTcp.Request()  
        req.name = name

        future = _ros2_config_delete_tcp.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('del_tcp Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def add_tool(name, weight, cog, inertia):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS2__:
        req = ConfigCreateTool.Request() 
        req.name    = name
        req.weight  = float(weight)
        req.cog     = [float(x) for x in cog]
        req.inertia = [float(x) for x in inertia]        

        future = _ros2_config_create_tool.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('add_tool Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

def del_tool(name):
    if type(name) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS2__:
        req = ConfigDeleteTool.Request()  
        req.name    = name

        future = _ros2_config_delete_tool.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('del_tool Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    return ret

##### DRL #########################################################################################################################

def drl_script_run(robotSystem, code):
    if type(code) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

    # ROS service call
    if __ROS2__:
        req = DrlStart.Request()
        req.robot_system = robotSystem
        req.code         = code

        future = _ros2_drl_start.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('drl_script_run Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret


def drl_script_stop(stop_mode):
    if type(stop_mode) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : stop_mode")
    print("drl_script_stop")
    # ROS service call
    if __ROS2__:
        req = DrlStop.Request()  
        req.stop_mode = stop_mode

        future = _ros2_drl_stop.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('drl_script_stop Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def drl_script_pause():
    # ROS service call
    if __ROS2__:
        req = DrlPause.Request() 

        future = _ros2_drl_pause.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('drl_script_pause Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def drl_script_resume():
    # ROS service call
    if __ROS2__:
        req = DrlResume.Request()  

        future = _ros2_drl_resume.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('drl_script_resume Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    return ret

def get_drl_state():
    # ROS service call
    if __ROS2__:
        req = GetDrlState.Request()  

        future = _ros2_get_drl_state.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('get_drl_state Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = result.drl_state 

    return ret


########################################################################################################################################
########################################################################################################################################
########################################################################################################################################
class CDsrRobot:
    def __init__(self, robot_id='dsr01', robot_model='m1013'):
        global g_node    
        self._robot_id = robot_id
        self._robot_model = robot_model
        print("CDsrRobot(): self._robot_id = %s, self._robot_model = %s" %(self._robot_id, self._robot_model))

        self._srv_name_prefix   = '' #'/' + self._robot_id + self._robot_model #ROS2


        ############### connect to dsr_control2 (ros service) ####################################################################### 
        #rospy.wait_for_service(self._srv_name_prefix +"/motion/move_joint")
        
        # system Operations
        self._ros2_set_robot_mode             = g_node.create_client(SetRobotMode, self._srv_name_prefix +"/system/set_robot_mode") ; self.req_SetRobotMode = SetRobotMode.Request()
        self._ros2_get_robot_mode             = g_node.create_client(GetRobotMode, self._srv_name_prefix +"/system/get_robot_mode") ; self.req_GetRobotMode = GetRobotMode.Request()
        self._ros2_set_robot_system           = g_node.create_client(SetRobotSystem, self._srv_name_prefix +"/system/set_robot_system") ; self.req_SetRobotSystem = SetRobotSystem.Request()
        self._ros2_get_robot_system           = g_node.create_client(GetRobotSystem, self._srv_name_prefix +"/system/get_robot_system") ; self.req_GetRobotSystem = GetRobotSystem.Request()
        self._ros2_get_robot_state            = g_node.create_client(GetRobotState, self._srv_name_prefix +"/system/get_robot_state") ; self.req_GetRobotState = GetRobotState.Request()
        self._ros2_set_robot_speed_mode       = g_node.create_client(SetRobotSpeedMode, self._srv_name_prefix +"/system/set_robot_speed_mode") ; self.req_SetRobotSpeedMode = SetRobotSpeedMode.Request()
        self._ros2_get_robot_speed_mode       = g_node.create_client(GetRobotSpeedMode, self._srv_name_prefix +"/system/get_robot_speed_mode") ; self.req_GetRobotSpeedMode = GetRobotSpeedMode.Request()
        self._ros2_set_safe_stop_reset_type   = g_node.create_client(SetSafeStopResetType, self._srv_name_prefix +"/system/set_safe_stop_reset_type") ; self.req_SetSafeStopResetType = SetSafeStopResetType.Request()
        self._ros2_get_last_alarm             = g_node.create_client(GetLastAlarm, self._srv_name_prefix +"/system/get_last_alarm") ; self.req_GetLastAlarm = GetLastAlarm.Request()
        self._ros2_get_current_pose           = g_node.create_client(GetCurrentPose, self._srv_name_prefix +"/system/get_current_pose") ; self.req_GetCurrentPose = GetCurrentPose.Request()

    
        #  motion Operations
        self._ros2_movej                      = g_node.create_client(MoveJoint,              self._srv_name_prefix +"/motion/move_joint") ; self.req_MoveJoint = MoveJoint.Request()
        self._ros2_movel                      = g_node.create_client(MoveLine,               self._srv_name_prefix +"/motion/move_line") ; self.req_MoveLine = MoveLine.Request()
        self._ros2_movejx                     = g_node.create_client(MoveJointx,             self._srv_name_prefix +"/motion/move_jointx") ; self.req_MoveJointx = MoveJointx.Request()
        self._ros2_movec                      = g_node.create_client(MoveCircle,             self._srv_name_prefix +"/motion/move_circle") ; self.req_MoveCircle = MoveCircle.Request()
        self._ros2_movesj                     = g_node.create_client(MoveSplineJoint,        self._srv_name_prefix +"/motion/move_spline_joint") ; self.req_MoveSplineJoint = MoveSplineJoint.Request() 
        self._ros2_movesx                     = g_node.create_client(MoveSplineTask,         self._srv_name_prefix +"/motion/move_spline_task") ; self.req_MoveSplineTask = MoveSplineTask.Request()
        self._ros2_moveb                      = g_node.create_client(MoveBlending,           self._srv_name_prefix +"/motion/move_blending") ; self.req_MoveBlending = MoveBlending.Request()
        self._ros2_move_spiral                = g_node.create_client(MoveSpiral,             self._srv_name_prefix +"/motion/move_spiral") ; self.req_MoveSpiral = MoveSpiral.Request()
        self._ros2_move_periodic              = g_node.create_client(MovePeriodic,           self._srv_name_prefix +"/motion/move_periodic") ; self.req_MovePeriodic = MovePeriodic.Request()
        self._ros2_move_wait                  = g_node.create_client(MoveWait,               self._srv_name_prefix +"/motion/move_wait") ; self.req_MoveWait = MoveWait.Request()
        self._ros2_jog                        = g_node.create_client(Jog,                    self._srv_name_prefix +"/motion/jog") ; self.req_Jog = Jog.Request()
        self._ros2_jog_multi                  = g_node.create_client(JogMulti,               self._srv_name_prefix +"/motion/jog_multi") ; self.req_JogMulti = JogMulti.Request()
        self._ros2_trans                      = g_node.create_client(Trans,                  self._srv_name_prefix +"/motion/trans") ; self.req_Trans = Trans.Request()

        self._ros2_fkin                       = g_node.create_client(Fkin,                   self._srv_name_prefix +"/motion/fkin") ; self.req_Fkin = Fkin.Request()
        self._ros2_ikin                       = g_node.create_client(Ikin,                   self._srv_name_prefix +"/motion/ikin") ; self.req_Ikin = Ikin.Request()
        self._ros2_set_ref_coord              = g_node.create_client(SetRefCoord,            self._srv_name_prefix +"/motion/set_ref_coord") ; self.req_SetRefCoord = SetRefCoord.Request()
        self._ros2_move_home                  = g_node.create_client(MoveHome,               self._srv_name_prefix +"/motion/move_home") ; self.req_MoveHome = MoveHome.Request()
        self._ros2_check_motion               = g_node.create_client(CheckMotion,            self._srv_name_prefix +"/motion/check_motion") ; self.req_CheckMotion = CheckMotion.Request()
        self._ros2_change_operation_speed     = g_node.create_client(ChangeOperationSpeed,   self._srv_name_prefix +"/motion/change_operation_speed") ; self.req_ChangeOperationSpeed = ChangeOperationSpeed.Request()
        self._ros2_enable_alter_motion        = g_node.create_client(EnableAlterMotion,      self._srv_name_prefix +"/motion/enable_alter_motion") ; self.req_EnableAlterMotion = EnableAlterMotion.Request()
        self._ros2_alter_motion               = g_node.create_client(AlterMotion,            self._srv_name_prefix +"/motion/alter_motion") ; self.req_AlterMotion = AlterMotion.Request()
        self._ros2_disable_alter_motion       = g_node.create_client(DisableAlterMotion,     self._srv_name_prefix +"/motion/disable_alter_motion") ; self.req_DisableAlterMotion = DisableAlterMotion.Request()
        self._ros2_set_singularity_handling   = g_node.create_client(SetSingularityHandling, self._srv_name_prefix +"/motion/set_singularity_handling") ; self.req_SetSingularityHandling = SetSingularityHandling.Request()


        # Auxiliary Control Operations
        self._ros2_get_control_mode               = g_node.create_client(GetControlMode,     self._srv_name_prefix +"/aux_control/get_control_mode") ; self.req_GetControlMode = GetControlMode.Request()
        self._ros2_get_control_space              = g_node.create_client(GetControlSpace,    self._srv_name_prefix +"/aux_control/get_control_space") ; self.req_GetControlSpace = GetControlSpace.Request()
    
        self._ros2_get_current_posj               = g_node.create_client(GetCurrentPosj,     self._srv_name_prefix +"/aux_control/get_current_posj") ; self.req_GetCurrentPosj = GetCurrentPosj.Request()
        self._ros2_get_current_velj               = g_node.create_client(GetCurrentVelj,     self._srv_name_prefix +"/aux_control/get_current_velj") ; self.req_GetCurrentVelj = GetCurrentVelj.Request()
        self._ros2_get_desired_posj               = g_node.create_client(GetDesiredPosj,     self._srv_name_prefix +"/aux_control/get_desired_posj") ; self.req_GetDesiredPosj = GetDesiredPosj.Request()
        self._ros2_get_desired_velj               = g_node.create_client(GetDesiredVelj,     self._srv_name_prefix +"/aux_control/get_desired_velj") ; self.req_GetDesiredVelj = GetDesiredVelj.Request()
    
        self._ros2_get_current_posx               = g_node.create_client(GetCurrentPosx,     self._srv_name_prefix +"/aux_control/get_current_posx") ; self.req_GetCurrentPosx = GetCurrentPosx.Request()
        self._ros2_get_current_velx               = g_node.create_client(GetCurrentVelx,     self._srv_name_prefix +"/aux_control/get_current_velx") ; self.req_GetCurrentVelx = GetCurrentVelx.Request()
        self._ros2_get_desired_posx               = g_node.create_client(GetDesiredPosx,     self._srv_name_prefix +"/aux_control/get_desired_posx") ; self.req_GetDesiredPosx = GetDesiredPosx.Request()
        self._ros2_get_desired_velx               = g_node.create_client(GetDesiredVelx,     self._srv_name_prefix +"/aux_control/get_desired_velx") ; self.req_GetDesiredVelx = GetDesiredVelx.Request()
     
        self._ros2_get_current_tool_flange_posx   = g_node.create_client(GetCurrentToolFlangePosx,   self._srv_name_prefix +"/aux_control/get_current_tool_flange_posx") ; self.req_GetCurrentToolFlangePosx = GetCurrentToolFlangePosx.Request()
     
        self._ros2_get_current_solution_space     = g_node.create_client(GetCurrentSolutionSpace,    self._srv_name_prefix +"/aux_control/get_current_solution_space") ; self.req_GetCurrentSolutionSpace = GetCurrentSolutionSpace.Request()
        self._ros2_get_current_rotm               = g_node.create_client(GetCurrentRotm,             self._srv_name_prefix +"/aux_control/get_current_rotm") ; self.req_GetCurrentRotm = GetCurrentRotm.Request()
        self._ros2_get_joint_torque               = g_node.create_client(GetJointTorque,             self._srv_name_prefix +"/aux_control/get_joint_torque") ; self.req_GetJointTorque = GetJointTorque.Request()
        self._ros2_get_external_torque            = g_node.create_client(GetExternalTorque,          self._srv_name_prefix +"/aux_control/get_external_torque") ; self.req_GetExternalTorque = GetExternalTorque.Request()
        self._ros2_get_tool_force                 = g_node.create_client(GetToolForce,               self._srv_name_prefix +"/aux_control/get_tool_force") ; self.req_GetToolForce = GetToolForce.Request()
        self._ros2_get_solution_space             = g_node.create_client(GetSolutionSpace,           self._srv_name_prefix +"/aux_control/get_solution_space") ; self.req_GetSolutionSpace = GetSolutionSpace.Request()
        self._ros2_get_orientation_error          = g_node.create_client(GetOrientationError,        self._srv_name_prefix +"/aux_control/get_orientation_error") ; self.req_GetOrientationError = GetOrientationError.Request()

        # Force/Stiffness Control & others Operations
        self._ros2_get_workpiece_weight        = g_node.create_client(GetWorkpieceWeight,            self._srv_name_prefix +"/force/get_workpiece_weight") ; self.req_GetWorkpieceWeight = GetWorkpieceWeight.Request()
        self._ros2_reset_workpiece_weight      = g_node.create_client(ResetWorkpieceWeight,          self._srv_name_prefix +"/force/reset_workpiece_weight") ; self.req_ResetWorkpieceWeight = ResetWorkpieceWeight.Request()

        self._ros2_parallel_axis1              = g_node.create_client(ParallelAxis1,                 self._srv_name_prefix +"/force/parallel_axis1") ; self.req_ParallelAxis1 = ParallelAxis1.Request()
        self._ros2_parallel_axis2              = g_node.create_client(ParallelAxis2,                 self._srv_name_prefix +"/force/parallel_axis2") ; self.req_ParallelAxis2 = ParallelAxis2.Request()
        self._ros2_align_axis1                 = g_node.create_client(AlignAxis1,                    self._srv_name_prefix +"/force/align_axis1") ; self.req_AlignAxis1 = AlignAxis1.Request()
        self._ros2_align_axis2                 = g_node.create_client(AlignAxis2,                    self._srv_name_prefix +"/force/align_axis2") ; self.req_AlignAxis2 = AlignAxis2.Request()
        self._ros2_is_done_bolt_tightening     = g_node.create_client(IsDoneBoltTightening,          self._srv_name_prefix +"/force/is_done_bolt_tightening") ; self.req_IsDoneBoltTightening = IsDoneBoltTightening.Request()
        self._ros2_release_compliance_ctrl     = g_node.create_client(ReleaseComplianceCtrl,         self._srv_name_prefix +"/force/release_compliance_ctrl") ; self.req_ReleaseComplianceCtrl = ReleaseComplianceCtrl.Request()
        self._ros2_task_compliance_ctrl        = g_node.create_client(TaskComplianceCtrl,            self._srv_name_prefix +"/force/task_compliance_ctrl") ; self.req_TaskComplianceCtrl = TaskComplianceCtrl.Request()
        self._ros2_set_stiffnessx              = g_node.create_client(SetStiffnessx,                 self._srv_name_prefix +"/force/set_stiffnessx") ; self.req_SetStiffnessx = SetStiffnessx.Request()
        self._ros2_calc_coord                  = g_node.create_client(CalcCoord,                     self._srv_name_prefix +"/force/calc_coord") ; self.req_CalcCoord = CalcCoord.Request()
        self._ros2_set_user_cart_coord1        = g_node.create_client(SetUserCartCoord1,             self._srv_name_prefix +"/force/set_user_cart_coord1") ; self.req_SetUserCartCoord1 = SetUserCartCoord1.Request()
        self._ros2_set_user_cart_coord2        = g_node.create_client(SetUserCartCoord2,             self._srv_name_prefix +"/force/set_user_cart_coord2") ; self.req_SetUserCartCoord2 = SetUserCartCoord2.Request()
        self._ros2_set_user_cart_coord3        = g_node.create_client(SetUserCartCoord3,             self._srv_name_prefix +"/force/set_user_cart_coord3") ; self.req_SetUserCartCoord3 = SetUserCartCoord3.Request()
        self._ros2_overwrite_user_cart_coord   = g_node.create_client(OverwriteUserCartCoord,        self._srv_name_prefix +"/force/overwrite_user_cart_coord") ; self.req_OverwriteUserCartCoord = OverwriteUserCartCoord.Request()
        self._ros2_get_user_cart_coord         = g_node.create_client(GetUserCartCoord,              self._srv_name_prefix +"/force/get_user_cart_coord") ; self.req_GetUserCartCoord = GetUserCartCoord.Request()
        self._ros2_set_desired_force           = g_node.create_client(SetDesiredForce,               self._srv_name_prefix +"/force/set_desired_force") ; self.req_SetDesiredForce = SetDesiredForce.Request()
        self._ros2_release_force               = g_node.create_client(ReleaseForce,                  self._srv_name_prefix +"/force/release_force") ; self.req_ReleaseForce = ReleaseForce.Request()
        self._ros2_check_position_condition    = g_node.create_client(CheckPositionCondition,        self._srv_name_prefix +"/force/check_position_condition") ; self.req_CheckPositionCondition = CheckPositionCondition.Request()
        self._ros2_check_force_condition       = g_node.create_client(CheckForceCondition,           self._srv_name_prefix +"/force/check_force_condition") ; self.req_CheckForceCondition = CheckForceCondition.Request()
        self._ros2_check_orientation_condition1= g_node.create_client(CheckOrientationCondition1,    self._srv_name_prefix +"/force/check_orientation_condition1") ; self.req_CheckOrientationCondition1 = CheckOrientationCondition1.Request()
        self._ros2_check_orientation_condition2= g_node.create_client(CheckOrientationCondition2,    self._srv_name_prefix +"/force/check_orientation_condition2") ; self.req_CheckOrientationCondition2 = CheckOrientationCondition2.Request()
        self._ros2_coord_transform             = g_node.create_client(CoordTransform,                self._srv_name_prefix +"/force/coord_transform") ; self.req_CoordTransform = CoordTransform.Request()

        #  GPIO Operations
        self._ros2_set_digital_output         = g_node.create_client(SetCtrlBoxDigitalOutput,        self._srv_name_prefix +"io/set_ctrl_box_digital_output") ; self.req_SetCtrlBoxDigitalOutput = SetCtrlBoxDigitalOutput.Request()
        self._ros2_get_digital_input          = g_node.create_client(GetCtrlBoxDigitalInput,         self._srv_name_prefix +"io/get_ctrl_box_digital_input") ; self.req_GetCtrlBoxDigitalInput = GetCtrlBoxDigitalInput.Request()
        self._ros2_set_tool_digital_output    = g_node.create_client(SetToolDigitalOutput,           self._srv_name_prefix +"io/set_tool_digital_output") ; self.req_SetToolDigitalOutput= SetToolDigitalOutput.Request()
        self._ros2_get_tool_digital_input     = g_node.create_client(GetToolDigitalInput,            self._srv_name_prefix +"io/get_tool_digital_input") ; self.req_GetToolDigitalInput = GetToolDigitalInput.Request()
        self._ros2_set_analog_output          = g_node.create_client(SetCtrlBoxAnalogOutput,         self._srv_name_prefix +"io/set_ctrl_box_analog_output") ; self.req_SetCtrlBoxAnalogOutput = SetCtrlBoxAnalogOutput.Request()
        self._ros2_get_analog_input           = g_node.create_client(GetCtrlBoxAnalogInput,          self._srv_name_prefix +"io/get_ctrl_box_analog_input") ; self.req_GetCtrlBoxAnalogInput = GetCtrlBoxAnalogInput.Request()
        self._ros2_set_mode_analog_output     = g_node.create_client(SetCtrlBoxAnalogOutputType,     self._srv_name_prefix +"io/set_ctrl_box_analog_output_type") ; self.req_SetCtrlBoxAnalogOutputType = SetCtrlBoxAnalogOutputType.Request()  
        self._ros2_set_mode_analog_input      = g_node.create_client(SetCtrlBoxAnalogInputType,      self._srv_name_prefix +"io/set_ctrl_box_analog_input_type") ; self.req_SetCtrlBoxAnalogInputType = SetCtrlBoxAnalogInputType.Request()   
        self._ros2_get_digital_output         = g_node.create_client(GetCtrlBoxDigitalOutput,        self._srv_name_prefix +"io/get_ctrl_box_digital_output") ; self.req_GetCtrlBoxDigitalOutput = GetCtrlBoxDigitalOutput.Request()
        self._ros2_get_tool_digital_output    = g_node.create_client(GetToolDigitalOutput,           self._srv_name_prefix +"io/get_tool_digital_output") ; self.req_GetToolDigitalOutput = GetToolDigitalOutput.Request()

        #  Modbus Operations, 
        self._ros2_set_modbus_output          = g_node.create_client(SetModbusOutput,                self._srv_name_prefix +"/modbus/set_modbus_output") ; self.req_SetModbusOutput = SetModbusOutput.Request()
        self._ros2_get_modbus_input           = g_node.create_client(GetModbusInput,                 self._srv_name_prefix +"/modbus/get_modbus_input") ; self.req_GetModbusInput = GetModbusInput.Request()
        self._ros2_add_modbus_signal          = g_node.create_client(ConfigCreateModbus,             self._srv_name_prefix +"/modbus/config_create_modbus") ; self.req_ConfigCreateModbus = ConfigCreateModbus.Request()
        self._ros2_del_modbus_signal          = g_node.create_client(ConfigDeleteModbus,             self._srv_name_prefix +"/modbus/config_delete_modbus") ; self.req_ConfigDeleteModbus = ConfigDeleteModbus.Request()

        # TCP Operations, 
        self._ros2_set_current_tcp            = g_node.create_client(SetCurrentTcp,                  self._srv_name_prefix +"/tcp/set_current_tcp") ; self.req_SetCurrentTcp = SetCurrentTcp.Request()
        self._ros2_get_current_tcp            = g_node.create_client(GetCurrentTcp,                  self._srv_name_prefix +"/tcp/get_current_tcp") ; self.req_GetCurrentTcp = GetCurrentTcp.Request()
        self._ros2_config_create_tcp          = g_node.create_client(ConfigCreateTcp,                self._srv_name_prefix +"/tcp/config_create_tcp") ; self.req_ConfigCreateTcp = ConfigCreateTcp.Request()
        self._ros2_config_delete_tcp          = g_node.create_client(ConfigDeleteTcp,                self._srv_name_prefix +"/tcp/config_delete_tcp") ; self.req_ConfigDeleteTcp = ConfigDeleteTcp.Request()
 
        # Tool Operations, 
        self._ros2_set_current_tool           = g_node.create_client(SetCurrentTool,                 self._srv_name_prefix +"/tool/set_current_tool") ; self.req_SetCurrentTool = SetCurrentTool.Request()
        self._ros2_get_current_tool           = g_node.create_client(GetCurrentTool,                 self._srv_name_prefix +"/tool/get_current_tool") ; self.req_GetCurrentTool = GetCurrentTool.Request()
        self._ros2_config_create_tool         = g_node.create_client(ConfigCreateTool,               self._srv_name_prefix +"/tool/config_create_tool") ; self.req_ConfigCreateTool = ConfigCreateTool.Request()
        self._ros2_config_delete_tool         = g_node.create_client(ConfigDeleteTool,               self._srv_name_prefix +"/tool/config_delete_tool") ; self.req_ConfigDeleteTool = ConfigDeleteTool.Request()
        self._ros2_set_tool_shape             = g_node.create_client(SetToolShape,                   self._srv_name_prefix +"/tool/set_tool_shape") ; self.req_SetToolShape = SetToolShape.Request()

        # DRL Operations, 
        self._ros2_drl_pause                  = g_node.create_client(DrlPause,                       self._srv_name_prefix +"/drl/drl_pause") ; self.req_DrlPause = DrlPause.Request()
        self._ros2_drl_resume                 = g_node.create_client(DrlResume,                      self._srv_name_prefix +"/drl/drl_resume") ; self.req_DrlResume = DrlResume.Request()
        self._ros2_drl_start                  = g_node.create_client(DrlStart,                       self._srv_name_prefix +"/drl/drl_start") ; self.req_DrlStart = DrlStart.Request()
        self._ros2_drl_stop                   = g_node.create_client(DrlStop,                        self._srv_name_prefix +"/drl/drl_stop") ; self.req_DrlStop = DrlStop.Request()
        self._ros2_get_drl_state              = g_node.create_client(GetDrlState,                    self._srv_name_prefix +"/drl/get_drl_state") ; self.req_GetDrlState = GetDrlState.Request()

        ########################################################################################################################################

        self._g_blend_state = False
        self._g_blend_radius = 0.0

        self._g_velj = [0.0] * DR_VELJ_DT_LEN
        self._g_accj = [0.0] * DR_ACCJ_DT_LEN

        self._g_velx = [0.0] * DR_VELX_DT_LEN
        self._g_velx[0]= 0.0
        self._g_velx[1]= DR_COND_NONE

        self._g_accx = [0.0] * DR_ACCX_DT_LEN
        self._g_accx[0]= 0.0
        self._g_accx[1]= DR_COND_NONE

        self._g_coord = DR_BASE

        self._g_analog_output_mode_ch1 = -1
        self._g_analog_output_mode_ch2 = -1

    ##### SYSTEM ##############################################################################################################################

    def set_robot_mode(self, robot_mode):
        if type(robot_mode) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : robot_mode")

        # ROS service call
        if __ROS2__:
            #srv = self._ros_set_robot_mode(robot_mode)
            req = self.req_SetRobotMode
            req.robot_mode = robot_mode

            #ret = 0 if (srv.success == True) else -1
            print("self._ros2_set_robot_mode.name ")
            future = self._ros2_set_robot_mode.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_robot_mode Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        return ret

    def get_robot_mode(self):

        # ROS2 service call
        if __ROS2__:
            req = self.req_GetRobotMode

            #ret = 0 if (srv.success == True) else -1
            future = self._ros2_get_robot_mode.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_robot_mode Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.robot_mode            
        return ret

    def set_robot_system(self, robot_system):
        if type(robot_system) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : robot_system")
        # ROS2 service call
        if __ROS2__:
            req = self.req_SetRobotSystem
            req.robot_system = robot_system

            future = self._ros2_set_robot_system.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_robot_system Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1   
                    g_node.get_logger().info('set_robot_system Service call failed')
                else:        
                    ret = 0 if (result.success == True) else -1                
        return ret


    def get_robot_system(self):
        # ROS2 service call
        if __ROS2__:
            req = self.req_GetRobotSystem

            future = self._ros2_get_robot_system.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_robot_system Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.robot_system            
        return ret

    def get_robot_state(self):
        # ROS2 service call
        if __ROS2__:
            req = self.req_GetRobotState

            future = self._ros2_get_robot_state.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_robot_state Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.robot_state            
        return ret

    def set_robot_speed_mode(self, speed_mode):
        if type(speed_mode) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed_mode")

        # ROS2 service call
        if __ROS2__:
            req = self.req_SetRobotSpeedMode
            req.speed_mode = speed_mode

            future = self._ros2_set_robot_speed_mode.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_robot_speed_mode Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        return ret

    def get_robot_speed_mode(self):
        # ROS2 service call
        if __ROS2__:
            req = self.req_GetRobotSpeedMode

            future = self._ros2_get_robot_speed_mode.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_robot_speed_mode Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.speed_mode            
        return ret

    def set_safe_stop_reset_type(self, reset_type):
        if type(reset_type) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : reset_type")

        # ROS2 service call
        if __ROS2__:
            req = self.req_SetSafeStopResetType
            req.reset_type = reset_type

            future = self._ros2_set_safe_stop_reset_type.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_safe_stop_reset_type Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        return ret


    def get_current_pose(self, space_type):
        if type(space_type) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : space_type")

        if __ROS2__:
            req = self.req_GetCurrentPose
            req.space_type = space_type

            future = self._ros2_get_current_pose.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_current_pose Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.pos            
        return ret

    def get_current_solution_space(self):
        # ROS2 service call
        if __ROS2__:
            req = self.req_GetCurrentSolutionSpace

            future = self._ros2_get_current_solution_space.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_current_solution_space Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.sol_space            
        return ret

    def get_last_alarm(self):
         #ROS2 service call
        if __ROS2__:
            req = self.req_GetLastAlarm

            future = self._ros2_get_last_alarm.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_last_alarm Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.log_alarm            
        return ret

##### Auxiliary Control ##############################################################################################################################
    def get_control_mode(self):
        if __ROS2__:
            req = self.req_GetControlMode  

            future = self._ros2_get_control_mode.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_control_mode Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.control_mode            
        return ret

    def get_control_space(self):
        if __ROS2__:
            req = self.req_GetControlSpace  

            future = self._ros2_get_control_space.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_control_space Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.space            
        return ret

    def get_current_posj(self):
        if __ROS2__:
            req = self.req_GetCurrentPosj  

            future = self._ros2_get_current_posj.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_current_posj Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:     
                    pos = list(result.pos)  # Convert tuple to list   
                    # set posj
                    cur_pos = posj(pos)
                    ret = cur_pos           
        return ret

    def get_current_velj(self):
        if __ROS2__:
            req = self.req_GetCurrentVelj  

            future = self._ros2_get_current_velj.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_current_velj Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = list(result.joint_speed)  # Convert tuple to list            
        return ret

    def get_desired_posj(self):
        if __ROS2__:
            req = self.req_GetDesiredPosj  

            future = self._ros2_get_desired_posj.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_desired_posj Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    pos = list(result.pos)  # Convert tuple to list            
                    ret = posj(pos)
        return ret

    def get_desired_velj(self):
        if __ROS2__:
            req = self.req_GetDesiredVelj  

            future = self._ros2_get_desired_velj.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_desired_velj Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = list(result.joint_vel)  # Convert tuple to list           
        return ret

    def get_current_posx(self, ref=None):

        # ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref != DR_BASE and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        if __ROS2__:
            req = self.req_GetCurrentPosx  
            req.ref = _ref

            future = self._ros2_get_current_posx.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_current_posx Service call failed %r' % (e,))
            else:
                if result == None:
                    return None, None    
                else:        
                    posx_info = _ros_Float64MultiArrayTolist(result.task_pos_info) # Convert Float64MultiArray to list
                    pos = []
                    for i in range(POINT_COUNT):
                        pos.append(posx_info[0][i])
                        sol = int(round( posx_info[0][6] ))
                        conv_posx = posx(pos)

                        return conv_posx, sol
        return 0

    def get_current_tool_flange_posx(self, ref=None):

        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        if __ROS2__:
            req =  self.req_GetCurrentToolFlangePosx  
            req.ref = _ref

            future = self._ros2_get_current_tool_flange_posx.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_current_tool_flange_posx Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.pos            

                    pos = list(result.pos)  # Convert tuple to list
                    # set posx type
                    ret = posx(pos)
        return ret

    def get_current_velx(self, ref=None):
        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        if __ROS2__:
            req =  self.req_GetCurrentVelx  
            req.ref = _ref

            future = self._ros2_get_current_velx.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_current_velx Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.vel            

        return ret

    def get_desired_posx(self, ref=None):
        # ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref != DR_BASE and _ref != DR_TOOL and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        if __ROS2__:
            req =  self.req_GetDesiredPosx  
            req.ref = _ref

            future = self._ros2_get_desired_posx.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_desired_posx Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    pos = list(result.pos)  # Convert tuple to list
                    conv_posx = posx(pos)
                    ret = conv_posx
        return ret

    def get_desired_velx(self, ref=None):
        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        if __ROS2__:
            req =  self.req_GetDesiredVelx  
            req.ref = _ref

            future = self._ros2_get_desired_velx.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_desired_velx Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = list(result.vel)  # Convert tuple to list            
        return ret

    def get_current_solution_space(self):
        if __ROS2__:
            req =  self.req_GetCurrentSolutionSpace  

            future = self._ros2_get_current_solution_space.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_current_solution_space Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.sol_space            
        return ret

    def get_current_rotm(self, ref=None):

        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        if __ROS2__:
            req =  self.req_GetCurrentRotm  
            req.ref = _ref

            future = self._ros2_get_current_rotm.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_current_rotm Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = _ros_Float64MultiArrayTolist(result.rot_matrix) # Convert Float64MultiArray to list 

        return ret

    def get_joint_torque(self):
        if __ROS2__:
            req =  self.req_GetJointTorque  

            future = self._ros2_get_joint_torque.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_joint_torque Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                     ret = list(result.jts)  # Convert tuple to list  
        return ret

    def get_external_torque(self):
        if __ROS2__:
            req =  self.req_GetExternalTorque  

            future = self._ros2_get_external_torque.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_external_torque Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = list(result.ext_torque)  # Convert tuple to list  
        return ret

    def get_tool_force(self, ref=None):

        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        if __ROS2__:
            req =  self.req_GetToolForce  
            req.ref = _ref

            future = self._ros2_get_tool_force.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_tool_force Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = list(result.tool_force)  # Convert tuple to list  
        return ret

    def get_solution_space(self, pos):
        _pos = get_posj(pos)

        if __ROS2__:
            req =  self.req_GetSolutionSpace  
            req.pos = [float(x) for x in _pos]

            future = self._ros2_get_solution_space.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_solution_space Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.sol_space            
        return ret

    def get_orientation_error(self, xd, xc, axis):
        # xd, xc
        _xd = get_posx(xd)
        _xc = get_posx(xc)

        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

        if (axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z and \
            axis != DR_AXIS_A and axis != DR_AXIS_B and axis != DR_AXIS_C):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

        if __ROS2__:
            req =  self.req_GetOrientationError  
            req.xd = [float(x) for x in _xd]
            req.xc = [float(x) for x in _xc]
            req.axis = axis

            future = self._ros2_get_orientation_error.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_orientation_error Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.ori_error            
        return ret

##### MOTION ##############################################################################################################################

    def trans(self, pos, delta, ref=None, ref_out=DR_BASE):
        # pos, delta
        _pos = get_posx(pos)
        _delta = get_posx(delta)
        # ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref
        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")
        if _ref != DR_BASE and _ref != DR_TOOL and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        # check ref_out
        if type(ref_out) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref_out")
        if ref_out != DR_BASE and ref_out != DR_WORLD and (ref_out < DR_TC_USER_MIN or ref_out > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref_out({0})".format(ref_out))
        # ROS service call
        if __ROS2__: 
            req = self.req_Trans
            req.pos     = [float(x) for x in _pos]
            req.delta   = [float(x) for x in _delta]
            req.ref     = _ref
            req.ref_out = ref_out            

            future = self._ros2_trans.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('trans Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.trans_pos            
        return ret

    def fkin(self, pos, ref=None):

        _pos = get_posj(pos)

        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        # ROS service call
        if __ROS2__: 
            req = self.req_Fkin
            req.pos = [float(x) for x in _pos]
            req.ref = _ref

            future = self._ros2_fkin.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('fkin Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.conv_posx            
        return ret

    def ikin(self, pos, sol_space, ref=None):

        _pos = get_posx(pos)

        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        # sol_space
        if type(sol_space) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : sol_space")

        if sol_space < DR_SOL_MIN or sol_space > DR_SOL_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : sol_space({0})".format(sol_space))

        # ROS service call
        if __ROS2__: 
            req = self.req_Ikin
            req.pos       = [float(x) for x in _pos]
            req.sol_space = sol_space        
            req.ref       = _ref

            future = self._ros2_ikin.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('ikin Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.conv_posj            
        return ret

    def set_ref_coord(self, coord):
        # coord
        if type(coord) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : coord")

        if coord != DR_BASE and coord != DR_TOOL and coord != DR_WORLD and not (DR_TC_USER_MIN <= coord <= DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : coord({0})".format(coord))

        # ROS service call
        if __ROS2__: 
            req = self.req_SetRefCoord
            req.coord = coord

            future = self._ros2_set_ref_coord.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_ref_coord Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                

        # set global accx
        global _g_coord

        _g_coord = coord

        #print_result("{0} = set_ref_coord(coord:{1})".format(ret, coord))
        return ret

    def movej(self, pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = self._movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=0)
        return ret
    def amovej(self, pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = self._movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=1)
        return ret
    def _movej(self, pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
        # _pos
        _pos = get_posj(pos)
    
        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velj
        else:
            if type(temp) == int or type(temp) == float:
                _vel = [temp] * DR_VELJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")
    
        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        for item in _vel:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accj
        else:
            if type(temp) == int or type(temp) == float:
                _acc = [temp] * DR_ACCJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")
    
        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        for item in _acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0
    
        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")
        
        if _time < 0 and _time != DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")
    
        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_joint(_vel, _acc, _time)
    
        # _radius
        _radius = get_param(radius, r)
        if _radius == None:
            #for ros global _g_blend_state
    
            if self._g_blend_state == True:
                _radius = self._g_blend_radius
            else:
                _radius = 0.0
    
        if type(_radius) != int and type(_radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")
    
        if _radius< 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")
    
        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    
        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")
    
        # ra
        if type(ra) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")
    
        if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")
    
        # _async
        if 1 == _async:
            _radius = 0   
    
        # ROS service call
        if __ROS2__: 
            #SSS srv = self._ros_movej(_pos, _vel[0], _acc[0], _time, mod, _radius, ra, _async)
            req = self.req_MoveJoint
            #for i in range(6):
            #    req.pos[i]      = pos[i] 
            req.pos         = [float(x) for x in _pos]
            req.vel         = float(_vel[0])
            req.acc         = float(_acc[0])
            req.time        = float(_time)
            req.mode        = int(mod)
            req.radius      = float(_radius)
            req.blend_type  = int(ra)
            req.sync_type   = int(_async)
            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_movej.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('movej Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:   
            ret = PythonMgr.py_movej(_pos, _vel, _acc, _time, _radius, mod, ra, _async)
        return ret

    def movejx(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
        ret = _movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, _async=0)
        return ret 
    def amovejx(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
        ret = _movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, _async=1)
        return ret 
    def _movejx(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None, _async=0):
        # _pos
        _pos = get_posx(pos)

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velj
        else:
            if type(temp) == int or type(temp) == float:
                _vel = [temp] * DR_VELJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accj
        else:
            if type(temp) == int or type(temp) == float:
                _acc = [temp] * DR_ACCJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

        if _time < 0 and _time != DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_joint(_vel, _acc, _time)

        # _radius
        _radius = get_param(radius, r)
        if _radius == None:
            if self._g_blend_state == True:
                _radius = self._g_blend_radius
            else:
                _radius = 0.0

        if type(_radius) != int and type(_radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

        if _radius < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

        # _ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        # ra
        if type(ra) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type, ra")

        if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

        # sol
        if type(sol) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : sol")

        if sol < DR_SOL_MIN or sol > DR_SOL_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : sol")

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__: 
            #srv = _ros_movejx(_pos, _vel[0], _acc[0], _time, mod, _ref, _radius, ra, sol, _async)   
            req = self.req_MoveJointx
            req.pos         = [float(x) for x in _pos]
            req.vel         = float(_vel[0])
            req.acc         = float(_acc[0])
            req.time        = float(_time)
            req.mode        = int(mod)
            req.ref         = int(_ref)
            req.radius      = float(_radius)
            req.blend_type  = int(ra)
            req.sol         = int(sol)
            req.sync_type   = int(_async)

            #ret = srv.success
            #ret = 0 if (srv.success == True) else -1
            future = self._ros2_movejx.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('movejx Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:    
            ret = PythonMgr.py_movejx(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, sol, _async)
            print_ext_result("{0} = PythonMgr.py_movejx(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref{6}, mod{7}, ra:{8}, sol:{9}, async:{10})" \
                             .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, sol, _async))   
        return ret


    def movel(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = _movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, _async=0)
        return ret
    def amovel(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = _movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, _async=1)
        return ret
    def _movel(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
        # _pos
        _pos = get_normal_pos(pos, def_type=posx)

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                #_vel = [temp] * DR_VELX_DT_LEN 
                _vel = [0] * DR_VELX_DT_LEN     
                _vel[0] = temp
                _vel[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        #print("_vel[0]={0}, _vel[1]={1}".format(_vel[0],_vel[1]))

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                #_acc = [temp] * DR_ACCX_DT_LEN 
                _acc = [0] * DR_ACCX_DT_LEN     
                _acc[0] = temp
                _acc[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        #print("_acc[0]={0}, _acc[1]={1}".format(_acc[0],_acc[1]))

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

        if _time < 0 and _time != DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_task(_vel, _acc, _time)

        # _radius
        _radius = get_param(radius, r)
        if _radius == None:
            if self._g_blend_state == True:
                _radius = self._g_blend_radius
            else:
                _radius = 0.0

        if type(_radius) != int and type(_radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

        if _radius < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

        # ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        # ra
        if type(ra) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

        if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

        # qcommand
        if type(_pos) == posx:
            qcommand = 0
        else:
            qcommand = 1

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__: 
            req = self.req_MoveLine

            req.pos         = [float(x) for x in _pos]
            req.vel         = [float(x) for x in _vel]
            req.acc         = [float(x) for x in _acc]
            req.time        = float(_time)
            req.radius      = float(_radius)
            req.ref         = int(_ref)
            req.mode        = int(mod)
            req.blend_type  = int(ra)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_movel.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('movel Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:    
            ret = PythonMgr.py_movel(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, qcommand, _async)
            print_ext_result("{0} = PythonMgr.py_movel(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref:{6}, mod:{7}, ra:{8}, qcommand:{9}, async:{10})" \
                             .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, qcommand, _async))
        return ret

    def movec(self, pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
        ret = _movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, _async=0)
        return ret
    def amovec(self, pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
        ret = _movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, _async=1)
        return ret
    def _movec(self, pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None, _async=0):

        # _pos1, _pos2

        _pos1 = get_normal_pos(pos1, def_type=posx)
        _pos2 = get_normal_pos(pos2, def_type=posx)

        if type(_pos1) != type(_pos2):
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos1, ps2")

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
                _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
                _vel[0] = temp
                _vel[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
                _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
                _acc[0] = temp
                _acc[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

        if _time < 0 and _time != DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_task(_vel, _acc, _time)

        # _radius
        _radius = get_param(radius, r)
        if _radius == None:
            if self._g_blend_state == True:
                _radius = self._g_blend_radius
            else:
                _radius = 0.0

        if type(_radius) != int and type(_radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

        if _radius < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

        # _ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        # _angle
        temp = get_param(angle, an)
        if temp == None:
            _angle = [0, 0]
        else:
            if type(temp) == int or type(temp) == float:
                _angle = [temp, 0]
            elif type(temp) == list and len(temp) == DR_ANGLE_DT_LEN:
                _angle = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : angle, an")

        if is_number(_angle) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : angle, an")

        for item in _angle:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : angle, an")

        # ra
        if type(ra) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

        if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

        # qcommand
        if type(_pos1) == posx:
            qcommand = 0
        else:
            qcommand = 1

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__:
            # make multi pos
            _circle_pos = _ros_listToFloat64MultiArray([_pos1, _pos2])
            #_circle_pos = _ros_listToFloat64MultiArray([ [float(x) for x in _pos1], [float(x) for x in _pos2] ])
            #print(_circle_pos)

            #srv = _ros_movec(_circle_pos, _vel, _acc, _time, mod, _ref, _angle[0], _angle[1], _radius, ra, _async) 
            req = self.req_MoveCircle
            req.pos         = _circle_pos               #??? 검증필요!
            req.vel         = [float(x) for x in _vel]
            req.acc         = [float(x) for x in _acc]
            req.time        = float(_time)
            req.radius      = float(_radius)
            req.ref         = int(_ref)
            req.mode        = int(mod)
            req.angle1      = float(_angle[0])
            req.angle2      = float(_angle[1])
            req.blend_type  = int(ra)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            
            future = self._ros2_movec.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('movec Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:   
            ret = PythonMgr.py_movec(_pos1, _pos2, _vel, _acc, _time, _radius, _ref, mod, _angle, ra, qcommand, _async)
            print_ext_result("{0} = PythonMgr.py_movec(pos1:{1}, pos2:{2}, vel:{3}, acc:{4}, time:{5}, radius:{6}, ref:{7}, mod:{8}, angle:{9}, ra:{10}, qcommand:{11}, async:{12})" \
                             .format(ret, _pos1, _pos2, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, dr_form(_angle), ra, qcommand, _async))
        return ret


    def movesj(self, pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = _movesj(pos_list, vel, acc, time, mod, v, a, t, _async=0)
        return ret
    def amovesj(self, pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = _movesj(pos_list, vel, acc, time, mod, v, a, t, _async=1)
        return ret
    def _movesj(self, pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None, _async=0):
        # pos_list
        if type(pos_list) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list")

        for item in pos_list:
            if type(item) != posj:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list [item]")

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velj
        else:
            if type(temp) == int or type(temp) == float:
                _vel = [temp] * DR_VELJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accj
        else:
            if type(temp) == int or type(temp) == float:
                _acc = [temp] * DR_ACCJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

        if _time < 0 and _time != DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_joint(_vel, _acc, _time)

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__:
            # make multi pos
            _spline_posj = _ros_listToFloat64MultiArray(pos_list)
            #srv = _ros_movesj(_spline_posj, len(_spline_posj), _vel, _acc, _time, mod, _async)

            req = self.req_MoveSplineJoint

            req.pos         = _spline_posj               #??? 검증필요!
            req.pos_cnt     = len(_spline_posj)

            if type(_vel) == list:
                _vel = float(_vel[0]) 
            if type(_acc) == list:
                _acc = float(_acc[0]) 
            req.vel         = _vel #[float(x) for x in _vel]
            req.acc         = _acc #[float(x) for x in _acc]

            req.time        = float(_time)
            req.mode        = int(mod)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_movesj.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('movesj Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:    
            ret = PythonMgr.py_movesj(pos_list, _vel, _acc, _time, mod, _async)
            print_ext_result("{0} = PythonMgr.py_movesj(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, mod:{5} async:{6})" \
                             .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, mod, _async))
        return ret



    def movesx(self, pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
        ret = _movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, _async=0)
        return ret
    def amovesx(self, pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
        ret = _movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, _async=1)
        return ret
    def _movesx(self, pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None, _async=0):
        # pos_list
        if type(pos_list) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list")

        for item in pos_list:
            if type(item) != posx:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list [item]")

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
                _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
                _vel[0] = temp
                _vel[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
                _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
                _acc[0] = temp
                _acc[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

        if _time < 0 and _time != DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_task(_vel, _acc, _time)

        # _ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        # vel_opt
        if type(vel_opt) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel_opt")

        if vel_opt != DR_MVS_VEL_NONE and vel_opt != DR_MVS_VEL_CONST:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel_opt")

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__:
            # make multi pos
            _spline_posx = _ros_listToFloat64MultiArray(pos_list)
            #srv = _ros_movesx(_spline_posx, len(_spline_posx), _vel, _acc, _time, mod, _ref, vel_opt, _async)
            req = self.req_MoveSplineTask
            req.pos         = _spline_posx               #??? 검증필요!
            req.pos_cnt     = len(_spline_posx)
            req.vel         = [float(x) for x in _vel]
            req.acc         = [float(x) for x in _acc]
            req.time        = float(_time)
            req.ref         = int(_ref)
            req.mode        = int(mod)
            req.opt         = int(vel_opt)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_movesx.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('movesx Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:
            ret = PythonMgr.py_movesx(pos_list, _vel, _acc, _time, _ref, mod, vel_opt, _async)
            print_ext_result("{0} = PythonMgr.py_movesx(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, vel_opt:{7}, async:{8})" \
                            .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, vel_opt, _async))
        return ret

    def moveb(self, seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = _moveb(seg_list, vel, acc, ref, time, mod, v, a, t, _async=0)
        return ret
    def amoveb(self, seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = _moveb(seg_list, vel, acc, ref, time, mod, v, a, t, _async=1)
        return ret
    def _moveb(self, seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None, _async=0):

        # seg_list
        if type(seg_list) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_list")

        if len(seg_list) == 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : seg_list")

        _seg_list = []
        if __ROS2__:
            for seg in seg_list:
                _seg_list.append(seg.to_list())
        else:
            for seg in seg_list:
                if type(seg) != posb:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_list(item)")
                else:
                    _seg_list.append(seg.to_list())

        if __ROS2__:
            print(len(_seg_list))
            _ros_seg_list = []
            _tmp_list = []

            for s in range(0, len(_seg_list)):
                #print(s)
                for elemnt in range(0, len(_seg_list[s])):
                    if _seg_list[s][elemnt] == None:
                        _seg_list[s][elemnt] = [0.0]*POINT_COUNT

                # make [pos1] + [pos2] + [type] + [radius]
                _tmp_list = _seg_list[s][1] + _seg_list[s][2] + [_seg_list[s][0]] + [_seg_list[s][3]]
                _ros_seg_list.append(_tmp_list)

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
                _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
                _vel[0] = temp
                _vel[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
                _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
                _acc[0] = temp
                _acc[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_task(_vel, _acc, _time)

        # ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS Service call
        if __ROS2__:
            seg = _ros_listToFloat64MultiArray(_ros_seg_list)
            #srv = _ros_moveb(seg, len(_ros_seg_list), _vel, _acc, _time, mod, _ref, _async)    
            req = self.req_MoveBlending
            req.segment     = seg               #??? 검증필요!
            req.pos_cnt     = len(_ros_seg_list)
            req.vel         = [float(x) for x in _vel]
            req.acc         = [float(x) for x in _acc]
            req.time        = float(_time)
            req.ref         = int(_ref)
            req.mode        = int(mod)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_moveb.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('moveb Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:
            ret = PythonMgr.py_moveb(_seg_list, _vel, _acc, _time, _ref, mod, _async)
            print_ext_result("{0} = PythonMgr.py_moveb(seg_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, async:{7})" \
                             .format(ret, dr_form(_seg_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, _async))
        return ret


    def move_spiral(self, rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
        ret = _move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, _async=0)
        return ret
    def amove_spiral(self, rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
        ret = _move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, _async=1)
        return ret
    def _move_spiral(self, rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None, _async=0):
        # rev
        if type(rev) != int and type(rev) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : rev")

        if rev <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : rev (Ranges: rev > 0)")

        # rmax
        if type(rmax) != int and type(rmax) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : rmax")

        if rmax <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: rmax (Ranges: rmax > 0)")

        # lmax
        if type(lmax) != int and type(lmax) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : lmax")

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                _vel = [temp] * DR_VELX_DT_LEN
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                _acc = [temp] * DR_ACCX_DT_LEN
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

        if _time < 0 and _time != DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_joint(_vel, _acc, _time)

        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

        if axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis")

        # ref
        if type(ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if ref != DR_BASE and ref != DR_TOOL and (ref < DR_TC_USER_MIN or ref > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__:
            #srv = _ros_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
            req = self.req_MoveSpiral
            req.revolution  = float(rev)
            req.max_radius  = float(rmax)
            req.max_length  = float(lmax)
            req.vel         = [float(x) for x in _vel]
            req.acc         = [float(x) for x in _acc]
            req.time        = float(_time)
            req.task_axis   = int(axis)
            req.ref         = int(ref)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_move_spiral.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('move_spiral Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        else:    
            ret = PythonMgr.py_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
            print_ext_result("{0} = PythonMgr.py_move_spiral(rev:{1}, rmax:{2}, lmax:{3}, vel:{4}, acc:{5}, time:{6}, axis:{7}, ref:{8}, async:{9})" \
                             .format(ret, dr_form(rev), dr_form(rmax), dr_form(lmax), dr_form(_vel), dr_form(_acc), _time, axis, ref, _async))    
        return ret

    def move_periodic(self, amp, period, atime=None, repeat=None, ref=DR_TOOL):
        ret = _move_periodic(amp, period, atime, repeat, ref, _async=0)
        return ret
    def amove_periodic(self, amp, period, atime=None, repeat=None, ref=DR_TOOL):
        ret = _move_periodic(amp, period, atime, repeat, ref, _async=1)
        return ret
    def _move_periodic(self, amp, period, atime=None, repeat=None, ref=DR_TOOL, _async=0):
        _amp = [POINT_COUNT]
        _period = [POINT_COUNT]
        _atime =0.0
        _repeat =0.0
        _ref=0 

        # amp : float[6] 
        if type(amp) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : amp")
        if len(amp) != POINT_COUNT:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : amp")
        _amp =amp

        # period : float or float[6] 
        if (type(period) != int) and (type(period) != float) and (type(period) != list):
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : period")

        if (type(period) == int) or (type(period) == float):
            _period = [period] * POINT_COUNT
        else: #list �� ��� 
            if len(period) != POINT_COUNT:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : period")
            _period = period

        # atime 
        if atime == None:
            _atime = 0.0
        else:
            if (type(atime) != int) and (type(atime) != float):
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : atime")
            if atime < 0.0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : atime")
            _atime = atime

        # repeat 
        if repeat == None:
            _repeat = 1
        else:
            if (type(repeat) != int) and (type(repeat) != float): 
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : repeat")
            if repeat < 0.0: 
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : repeat")
            _repeat = repeat

        # ref
        if ref == None:
            #_ref = _g_coord
            _ref = DR_TOOL
        else:
            if type(ref) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

            if ref < DR_BASE or ref > DR_TC_USER_MAX:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")
            _ref = ref

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__:
            #srv = _ros_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)    
            req = self.req_MovePeriodic
            req.amp         = [float(x) for x in _amp]
            req.periodic    = [float(x) for x in _period]
            req.acc         = float(_atime)
            req.repeat      = int(_repeat)
            req.ref         = int(_ref)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_move_periodic.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('move_periodic Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        else:
            ret = PythonMgr.py_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)
            print_ext_result("{0} = PythonMgr.move_periodic(amp:{1}, period:{2}, atime:{3}, repeat{4}, ref:{5}, async:{6})" \
                             .format(ret, dr_form(_amp), dr_form(_period), _atime, _repeat, _ref, _async))
        return ret

    def move_home(self, target=None):

        # target
        if target == None:
            _target = DR_HOME_TARGET_MECHANIC
        else:
            _target = target

        if __ROS2__:
            req = self.req_MoveHome    
            req.target = _target

            future = self._ros2_move_home.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('move_home Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return 0
        

    def mwait(self, time=0):
        ret = _move_wait(time)
        return ret
    def _move_wait(self, time):
        if type(time) != int and type(time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

        if time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time")

        # ROS service call
        if __ROS2__:
            req = self.req_MoveWait  #ROS 에서는 time 인자를 사용하지 않음. 

            future = self._ros2_move_wait.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('mwait Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret

    def check_motion(self):

        if __ROS2__:
            req = self.req_CheckMotion  

            future = self._ros2_check_motion.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('check_motion Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.status            
        return ret

    def change_operation_speed(self, speed):
        if type(speed) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed")

        if speed < DR_OP_SPEED_MIN or speed > DR_OP_SPEED_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : speed({0})".format(speed))

        if __ROS2__:
            req = self.req_ChangeOperationSpeed 
            req.speed = speed

            future = self._ros2_change_operation_speed.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('change_operation_speed Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret

    def enable_alter_motion(self, n, mode, ref=None, limit_dPOS=None, limit_dPOS_per=None):
        # n
        if type(n) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : n")
        if n < 0:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : (n>=0)")
        _n = n #*20

        # mode
        if mode < DR_DPOS or mode > DR_DVEL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mode({0})".format(mode))
        _mode = mode

        # ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref != DR_BASE and _ref != DR_TOOL and _ref != DR_WORLD and (_ref < DR_TC_USER_MIN or _ref > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        if(None==limit_dPOS):
            _limit_dPOS = [DR_COND_NONE, DR_COND_NONE]
        else:
            _limit_dPOS = limit_dPOS

        if(None==limit_dPOS_per):
            _limit_dPOS_per = [DR_COND_NONE, DR_COND_NONE]
        else:
            _limit_dPOS_per = limit_dPOS_per

        if __ROS2__:
            req = self.req_EnableAlterMotion  
            req.n              = _n 
            req.mode           = _mode
            req.ref            = _ref
            req.limit_dpos     = [float(x) for x in _limit_dPOS] 
            req.limit_dpos_per = [float(x) for x in _limit_dPOS_per]

            future = self._ros2_enable_alter_motion.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('enable_alter_motion Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret

    def alter_motion(self, dposx):

        # _dposx
        if (type(dposx) == posx):
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : dposx")

        _dposx = get_normal_pos(dposx, def_type=posx)

        if __ROS2__:
            req = self.req_AlterMotion  
            req.pos = [float(x) for x in _dposx]

            future = self._ros2_alter_motion.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('alter_motion Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret

    def disable_alter_motion(self):

        if __ROS2__:
            req = self.req_DisableAlterMotion  

            future = self._ros2_disable_alter_motion.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('disable_alter_motion Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret

    def set_singular_handling(self, mode = DR_AVOID):
        if type(mode) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mode")

        if mode < DR_AVOID or mode > DR_VAR_VEL:    #DR_TASK_STOP
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mode({0})".format(mode))

        if __ROS2__:
            req = self.req_SetSingularityHandling 
            req.mode = mode

            future = self._ros2_set_singularity_handling.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_singular_handling Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret

    def set_singularity_handling(self, mode = DR_AVOID):
        ret = set_singular_handling(mode)
        return ret

    def jog(self, jog_axis, ref=0, speed=0):
        if type(jog_axis) != int and type(jog_axis) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : jog_axis")
        
        if type(ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if type(speed) != int and type(speed) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed")
        # ROS service call
        if __ROS2__:
            req = self.req_Jog
            req.jog_axis       = jog_axis                            
            req.move_reference = ref
            req.speed          = float(speed)

            future = self._ros2_jog.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('jog Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret

    def jog_multi(self, jog_axis_list, ref=0, speed=0):
        if type(jog_axis_list) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : jog_axis_list")
        
        if type(ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if type(speed) != int and type(speed) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : speed")
        # ROS service call
        if __ROS2__:
            req = self.req_JogMulti
            req.jog_axis       = [float(x) for x in jog_axis_list]     
            req.move_reference = ref 
            req.speed          = float(speed) 

            future = self._ros2_jog_multi.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('jog_multi Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret


##### force/stiffness Control #########################################################################################################################

    def get_workpiece_weight(self):

        print("================================> get_workpiece_weight()")
        if __ROS2__:
            req = self.req_GetWorkpieceWeight 

            future = self._ros2_get_workpiece_weight.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_workpiece_weight Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.weight            
        return ret


    def reset_workpiece_weight(self):

        if __ROS2__:
            req = self.req_ResetWorkpieceWeight  

            future = self._ros2_reset_workpiece_weight.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('reset_workpiece_weight Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret

    def parallel_axis(self, *args, **kargs):
        len_args = len(args)
        len_kargs = len(kargs)
        _nType = 0

        # check kargs.key
        for key in kargs.keys():
            if key != "x1" and key != "x2" and key != "x3" and key != "vect" and key != "axis" and key != "ref":
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : x1, x2, x3, vect, axis, ref")

        # get kargs
        _x1 = get_kargs(kargs, "x1")
        _x2 = get_kargs(kargs, "x2")
        _x3 = get_kargs(kargs, "x3")
        _vect = get_kargs(kargs, "vect")
        _axis = get_kargs(kargs, "axis")
        _ref = get_kargs(kargs, "ref")

        # check parameter combination
        if len_args == 0:
            if len_kargs == 2:
                if _vect == None or _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_kargs == 3:
                _nType = 3
                if _vect == None or _axis == None or _ref == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_kargs == 4:
                if _x1 == None or _x2 == None or _x3 == None or _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_kargs == 5:
                _nType = 2
                if _x1 == None or _x2 == None or _x3 == None or _axis == None or _ref == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        elif len_args == 1:
            if len_kargs != 1:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

            _vect = args[0]

            if _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        elif len_args == 2:
            if len_kargs == 1:  # +ref
                _nType = 3
                _vect = args[0]
                _axis = args[1]
            else:
                if len_kargs != 0:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            _vect = args[0]
            _axis = args[1]

        elif len_args == 3:
            if len_kargs == 0:  # +ref
                _nType = 3
                _vect = args[0]
                _axis = args[1]
                _ref  = args[2]
            else:
                if len_kargs != 1:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]

                if _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        elif len_args == 4:
            if len_kargs == 1:  # + ref
                _nType = 2
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _axis = args[3]
            else:
                if len_kargs != 0:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _axis = args[3]

        elif len_args == 5:
            if len_kargs == 0:  # + ref
                _nType = 2
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _axis = args[3]
                _ref = args[4]
            else:
                if len_kargs != 0:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _axis = args[3]
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        # check parameter type, value
        if _x1 != None:
            _x1 = get_posx(_x1)
            _x2 = get_posx(_x2)
            _x3 = get_posx(_x3)
        else:
            if type(_vect) != list:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vect")
            if len(_vect) != DR_VECTOR_DT_LEN:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vect({0})".format(_vect))

        if type(_axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

        if _axis != DR_AXIS_X and _axis != DR_AXIS_Y and _axis != DR_AXIS_Z:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(_axis))

        if _nType == 2: # with ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
            if __ROS2__:
                req = self.req_ParallelAxis1  
                req.x1   = [float(x) for x in _x1]
                req.x2   = [float(x) for x in _x2]
                req.x3   = [float(x) for x in _x3]
                req.axis = _axis
                req.ref  = _ref

                future = self._ros2_parallel_axis1.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('parallel_axis Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = 0 if (result.success == True) else -1     

        elif _nType == 3: # with ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
            if __ROS2__:
                req = self.req_ParallelAxis2  
                req.vect = [float(x) for x in _vect] 
                req.axis = _axis           
                req.ref  = _ref

                future = self._ros2_parallel_axis2.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('parallel_axis Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = 0 if (result.success == True) else -1     

        else:   # 기존
            _ref = DR_BASE
            if _x1 != None:
                if __ROS2__:
                    req = self.req_ParallelAxis1  
                    req.x1   = [float(x) for x in _x1]
                    req.x2   = [float(x) for x in _x2]
                    req.x3   = [float(x) for x in _x3]
                    req.axis = _axis
                    req.ref  = _ref

                    future = self._ros2_parallel_axis1.call_async(req)
                    rclpy.spin_until_future_complete(g_node, future)

                    try:
                        result = future.result()
                    except Exception as e:
                        g_node.get_logger().info('parallel_axis Service call failed %r' % (e,))
                    else:
                        if result == None:
                            ret = -1    
                        else:        
                            ret = 0 if (result.success == True) else -1     


            else:
                if __ROS2__:
                    req = ParallelAxis2.Request()  
                    req._vect = vect
                    req._axis = axis           
                    req.ref   = _ref

                    future = self._ros2_parallel_axis2.call_async(req)
                    rclpy.spin_until_future_complete(g_node, future)

                    try:
                        result = future.result()
                    except Exception as e:
                        g_node.get_logger().info('parallel_axis Service call failed %r' % (e,))
                    else:
                        if result == None:
                            ret = -1    
                        else:        
                            ret = 0 if (result.success == True) else -1     

        return ret

    def align_axis(self, *args, **kargs):
        len_args = len(args)
        len_kargs = len(kargs)
        _nType = 0

        # check kargs.key
        for key in kargs.keys():
            if key != "x1" and key != "x2" and key != "x3" and key != "vect" and key != "pos" and key != "axis" and key != "ref":
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        # get kargs
        _x1 = get_kargs(kargs, "x1")
        _x2 = get_kargs(kargs, "x2")
        _x3 = get_kargs(kargs, "x3")
        _vect = get_kargs(kargs, "vect")
        _pos = get_kargs(kargs, "pos")
        _axis = get_kargs(kargs, "axis")
        _ref = get_kargs(kargs, "ref")

        # check parameter combination
        if len_args == 0:
            if len_kargs == 3:
                if _vect == None or _pos == None or _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_kargs == 5:
                if _x1 == None or _x2 == None or _x3 == None or _pos == None or _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_kargs == 6:
                _nType = 2
                if _x1 == None or _x2 == None or _x3 == None or _pos == None or _axis == None or _ref == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_kargs == 4:
                _nType = 3
                if _vect == None or _pos == None or _axis == None or _ref == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        elif len_args == 1:
            if len_kargs != 2:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

            _vect = args[0]

            if _pos == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            if _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        elif len_args == 2:
            if len_kargs != 1:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

            _vect = args[0]
            _pos = args[1]

            if _axis == None:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        elif len_args == 3:
            if len_kargs == 0:
                _vect = args[0]
                _pos = args[1]
                _axis = args[2]
            elif len_kargs == 1:
                _nType = 3
                _vect = args[0]
                _pos = args[1]
                _axis = args[2]
            elif len_kargs == 2:
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]

                if _pos == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                if _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        elif len_args == 4:
            if len_kargs == 0:
                _nType = 3
                _vect = args[0]
                _pos = args[1]
                _axis = args[2]
                _ref = args[3]
            else:
                if len_kargs != 1:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _pos = args[3]

                if _axis == None:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        elif len_args == 5:
            if len_kargs == 0:
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _pos = args[3]
                _axis = args[4]
            elif len_kargs == 1:    # + ref
                _nType = 2
                _x1 = args[0]
                _x2 = args[1]
                _x3 = args[2]
                _pos = args[3]
                _axis = args[4]

        elif len_args == 6:
            if len_kargs != 0:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            _nType = 2
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _pos = args[3]
            _axis = args[4]
            _ref  = args[5]

        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        # check parameter type, value
        if _x1 != None:
            _x1 = get_posx(_x1)
            _x2 = get_posx(_x2)
            _x3 = get_posx(_x3)
        else:
            if type(_vect) != list:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vect")
            if len(_vect) != DR_VECTOR_DT_LEN:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vect({0})".format(_vect))

        norm_pos = get_posx(_pos)

        if type(_axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")
        if _axis != DR_AXIS_X and _axis != DR_AXIS_Y and _axis != DR_AXIS_Z:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(_axis))

        if _nType == 2: # with ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

            if __ROS2__:
                req = self.req_AlignAxis1  
                req.x1       = [float(x) for x in _x1]
                req.x2       = [float(x) for x in _x2]
                req.x3       = [float(x) for x in _x3]
                req.pos      = [float(x) for x in norm_pos]
                req.axis     = _axis
                req.ref      = _ref

                future = self._ros2_align_axis1.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('align_axis Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = 0 if (result.success == True) else -1     


        elif _nType == 3: # with ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

            if __ROS2__:
                req = self.req_AlignAxis2  
                req.vect     = [float(x) for x in _vect]
                req.pos      = [float(x) for x in norm_pos]
                req.axis     = _axis
                req.ref      = _ref

                future = self._ros2_align_axis2.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('align_axis Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = 0 if (result.success == True) else -1     


        else:   # 기존
            _ref = DR_BASE
            if _x1 != None:
                if __ROS2__:
                    req = self.req_AlignAxis1  
                    req.x1       = [float(x) for x in _x1]
                    req.x2       = [float(x) for x in _x2]
                    req.x3       = [float(x) for x in _x3]
                    req.pos      = [float(x) for x in norm_pos]
                    req.axis     = _axis
                    req.ref      = _ref

                    future = self._ros2_align_axis1.call_async(req)
                    rclpy.spin_until_future_complete(g_node, future)

                    try:
                        result = future.result()
                    except Exception as e:
                        g_node.get_logger().info('align_axis Service call failed %r' % (e,))
                    else:
                        if result == None:
                            ret = -1    
                        else:        
                            ret = 0 if (result.success == True) else -1     

            else:

                if __ROS2__:
                    req = self.req_AlignAxis2  
                    req.vect     = [float(x) for x in _vect]
                    req.pos      = [float(x) for x in norm_pos]
                    req.axis     = _axis
                    req.ref      = _ref

                    future = _ros2_align_axis2.call_async(req)
                    rclpy.spin_until_future_complete(g_node, future)

                    try:
                        result = future.result()
                    except Exception as e:
                        g_node.get_logger().info('align_axis Service call failed %r' % (e,))
                    else:
                        if result == None:
                            ret = -1    
                        else:        
                            ret = 0 if (result.success == True) else -1     

        return ret

    def is_done_bolt_tightening(self, m=0, timeout=0, axis=None):
        # m
        if type(m) != int and type(m) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : m")

        # timeout
        if type(timeout) != int and type(timeout) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : timeout")

        if timeout < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : timeout({0})".format(timeout))

        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

        if axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

        if __ROS2__:
            req = self.req_IsDoneBoltTightening  
            req.m       = float(m)
            req.timeout = float(timeout)
            req.axis    = axis

            future = self._ros2_is_done_bolt_tightening.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)
            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('is_done_bolt_tightening Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def release_compliance_ctrl(self):

        if __ROS2__:
            req = self.req_ReleaseComplianceCtrl  

            future = self._ros2_release_compliance_ctrl.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('release_compliance_ctrl Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def task_compliance_ctrl(self, stx=[3000, 3000, 3000, 200, 200, 200], time=0):
        # st
        if type(stx) != list or len(stx) != 6:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : stx")

        if is_number(stx) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : stx({0})".format(stx))

        # _ref
        _ref = _g_coord

        # time
        if type(time) != int and type(time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

        if time != -1 and time < 0:     # -1 : motion default
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))

        if time > 1:
            _time = 1
        else:
            _time = time

        if __ROS2__:
            req = self.req_TaskComplianceCtrl  
            req.stx  = [float(x) for x in stx]
            req.ref  = _ref      
            req.time = float(_time)

            future = self._ros2_task_compliance_ctrl.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('task_compliance_ctrl Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret


    def set_stiffnessx(self, stx=[500, 500, 500, 100, 100, 100], time=0):
        # st
        if type(stx) != list or len(stx) != 6:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : stx")

        if is_number(stx) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : stx({0})".format(stx))

        # _ref
        _ref = _g_coord

        # time
        if type(time) != int and type(time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

        if time != -1 and time < 0:     # -1 : motion default
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))

        if time > 1:
            _time = 1
        else:
            _time = time

        if __ROS2__:
            req = self.req_SetStiffnessx  
            req.stx  = [float(x) for x in stx]
            req.ref  = _ref      
            req.time = float(_time)

            future = self._ros2_set_stiffnessx.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_stiffnessx Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret

    def calc_coord(self, *args, **kargs):

        #arg(x1, ref, mod)                    arg =0 , kargs =3    _nType=1
        #arg(x1, x2, ref, mod)                arg =0 , kargs =4    _nType=2
        #arg(x1, x2, x3, ref, mod)            arg =0 , kargs =5    _nType=3
        #arg(x1, x2, x3, x4, ref, mod)        arg =0 , kargs =6    _nType=4

        #arg(?, ref, mod)                     arg =1 , kargs =2    _nType=1
        #arg(?, ?, ref, mod)                  arg =2 , kargs =2    _nType=2
        #arg(?, ?, ?, ref, mod)               arg =3 , kargs =2    _nType=3
        #arg(?, ?, ?, ?, ref, mod)            arg =4 , kargs =2    _nType=4

        #arg(?, ?, ?)                         arg =3 , kargs =0    _nType=1
        #arg(?, ?, ?, ?)                      arg =4 , kargs =0    _nType=2
        #arg(?, ?, ?, ?, ?)                   arg =5 , kargs =0    _nType=3
        #arg(?, ?, ?, ?, ?, ?)                arg =6 , kargs =0    _nType=4

        len_args = len(args)
        len_kargs = len(kargs)
        _cnt_pos = 0

        # check kargs.key
        for key in kargs.keys():
            if key != "x1" and key != "x2" and key != "x3" and key != "x4" and key != "ref" and key != "mod":
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : x1, x2, x3, x4, ref, mod")

        # get kargs
        _x1 = get_kargs(kargs, "x1")
        _x2 = get_kargs(kargs, "x2")
        _x3 = get_kargs(kargs, "x3")
        _x4 = get_kargs(kargs, "x4")
        _ref = get_kargs(kargs, "ref")
        _mod = get_kargs(kargs, "mod")

        if(_x1 == None):
            _x1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if(_x2 == None):
            _x2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if(_x3 == None):
            _x3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if(_x4 == None):
            _x4 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        if len_args == 0 and len_kargs ==3:
            _cnt_pos = 1
            print("x1={}, ref={}, mod={}".format(_x1, _ref, _mod))
        elif len_args == 0 and len_kargs ==4:
            _cnt_pos = 2
            print("x1={}, x2={}, ref={}, mod={}".format(_x1, _x2, _ref, _mod))
        elif len_args == 0 and len_kargs ==5:
            _cnt_pos = 3
            print("x1={}, x2={}, x3={}, ref={}, mod={}".format(_x1, _x2, _x3, _ref, _mod))
        elif len_args == 0 and len_kargs ==6:
            _cnt_pos = 4
            print("x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))

        elif len_args == 1 and len_kargs ==2:
            _cnt_pos = 1
            _x1 = args[0]
            print("x1={}, ref={}, mod={}".format(_x1, _ref, _mod))
        elif len_args == 2 and len_kargs ==2:
            _cnt_pos = 2
            _x1 = args[0]
            _x2 = args[1]
            print("x1={}, x2={}, ref={}, mod={}".format(_x1, _x2, _ref, _mod))
        elif len_args == 3 and len_kargs ==2:
            _cnt_pos = 3
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            print("x1={}, x2={}, x3={}, ref={}, mod={}".format(_x1, _x2, _x3, _ref, _mod))
        elif len_args == 4 and len_kargs ==2:
            _cnt_pos = 4
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _x4 = args[3]
            print("x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))

        elif len_args == 3 and len_kargs ==0:
            _cnt_pos = 1
            _x1 = args[0]
            _ref =args[1]
            _mod =args[2]
            print("x1={}, ref={}, mod={}".format(_x1, _ref, _mod))
        elif len_args == 4 and len_kargs ==0:
            _cnt_pos = 2
            _x1 = args[0]
            _x2 = args[1]
            _ref =args[2]
            _mod =args[3]
            print("x1={}, x2={}, ref={}, mod={}".format(_x1, _x2, _ref, _mod))
        elif len_args == 5 and len_kargs ==0:
            _cnt_pos = 3
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _ref =args[3]
            _mod =args[4]
            print("x1={}, x2={}, x3={}, ref={}, mod={}".format(_x1, _x2, _x3, _ref, _mod))
        elif len_args == 6 and len_kargs ==0:
            _cnt_pos = 4
            _x1 = args[0]
            _x2 = args[1]
            _x3 = args[2]
            _x4 = args[3]
            _ref =args[4]
            _mod =args[5]
            print("x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid arguments")

        print("FINAL : x1={}, x2={}, x3={}, x4={}, ref={}, mod={}".format(_x1, _x2, _x3, _x4, _ref, _mod))

        # check parameter type, value
        if _x1 != None: 
            _x1 = get_posx(_x1)
        if _x2 != None: 
            _x2 = get_posx(_x2)
        if _x3 != None: 
            _x3 = get_posx(_x3)
        if _x4 != None: 
            _x4 = get_posx(_x4)
        if _ref !=None:
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
        if _mod != None:
            if _mod != 0 and _mod != 1:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0})".format(_mod))

        if __ROS2__:
            req = self.req_CalcCoord  
            req.input_pos_cnt = _cnt_pos
            req.x1      = [float(x) for x in _x1]
            req.x2      = [float(x) for x in _x2]
            req.x3      = [float(x) for x in _x3]
            req.x4      = [float(x) for x in _x4]
            req.ref     = _ref
            req.mod     = _mod

            future = self._ros2_calc_coord.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('calc_coord Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    pos = list(result.conv_posx)  # Convert tuple to list 
                    conv_posx = posx(pos)
                    ret = pos

        return ret

    def set_user_cart_coord(self, *args, **kargs):
        len_args = len(args)
        len_kargs = len(kargs)
        _nType =0  

        # check kargs.key
        for key in kargs.keys():
            if key != "x1" and key != "x2" and key != "x3" and key != "pos" and key != "u1" and key != "v1" and key != "v1" and key != "ref":
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : x1, x2, x3, pos, u1, v1, ref")

        # get kargs
        _x1 = get_kargs(kargs, "x1")
        _x2 = get_kargs(kargs, "x2")
        _x3 = get_kargs(kargs, "x3")
        _pos= get_kargs(kargs, "pos")
        _u1 = get_kargs(kargs, "u1")
        _v1 = get_kargs(kargs, "v1")
        _ref = get_kargs(kargs, "ref")

        # check parameter combination
        #set_user_cart_coord(x1=?, x2=?, x3=?, pos=?)   arg =0 , kargs =4    _nType=0
        #set_user_cart_coord(u1=?, v1=?, pos=?)         arg =0 , kargs =3 *  _nType=1
        #set_user_cart_coord(x1=?, x2=?, x3=?)          arg =0 , kargs =3 *  _nType=2

        #set_user_cart_coord(x1=?, x2=?, x3=?, ?)       arg =1 , kargs =3    _nType=0
        #set_user_cart_coord(u1=?, v1=?, ?)             arg =1 , kargs =2    _nType=1

        #set_user_cart_coord(?, ?, pos=?)               arg =2 , kargs =1    _nType=1

        #set_user_cart_coord(?, ?, ?, pos=?)            arg =3 , kargs =1    _nType=0
        #set_user_cart_coord(?, ?, ?)                   arg =3 , kargs =0 *  _nType=1
        #set_user_cart_coord(?, ?, ?)                   arg =3 , kargs =0 *  _nType=2  

        #set_user_cart_coord(?, ?, ?, ?)                arg =4 , kargs =0    _nType=0

    #----- 신규 추가 명령 2019/11/27 ----------------------------------------------------------------------------------------------------------------
        #set_user_cart_coord(pos=?, ref=?)         arg =0 , kargs =2    _nType=5
        #set_user_cart_coord(pos=?, ?)             arg =1 , kargs =1 *  _nType=5 #python syntax error : positional argument follows keyword argument
        #set_user_cart_coord(?, ref=?)             arg =1 , kargs =1 *  _nType=5
        #set_user_cart_coord(?, ?)                 arg =2 , kargs =0    _nType=5
    #------------------------------------------------------------------------------------------------------------------------------------------------

        if len_args == 0 and len_kargs ==2:
            print("new commnad  len_args == 0 and len_kargs ==2")
            #_pos
            #_ref
            print("_pos={},_ref={}".format(_pos, _ref))
            _nType = 5
        elif len_args == 1 and len_kargs ==1:
            print("new commnad  len_args == 1 and len_kargs ==1")
            _pos = args[0]
            #_ref
            print("_pos={},_ref={}".format(_pos, _ref))
            _nType = 5
        elif len_args == 2 and len_kargs ==0:
            print("new commnad  len_args == 2 and len_kargs ==0")
            _pos = args[0]
            _ref = args[1]
            print("_pos={},_ref={}".format(_pos, _ref))
            _nType = 5
        else:
            if len_args == 0:
                if len_kargs == 5:
                    if _x1 == None or _x2 == None or _x3 == None or _pos == None or _ref == None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    _nType = 3
                elif len_kargs == 4:
                    if _ref != None:
                        _nType = 4
                    else:
                        if _x1 == None or _x2 == None or _x3 == None or _pos == None:
                            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                        _nType = 0
                elif len_kargs == 3:
                    if _u1 != None: 
                        if _v1 == None or _pos == None:
                            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                        _nType = 1
                    elif _x1 != None:
                        if _x2 == None or _x3 == None:
                            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                        _nType = 2
                    else: 
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_args == 1:
                if len_kargs == 3:
                    if _x1 == None or _x2 == None or _x3 == None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    _nType = 0
                    _pos= args[0]
                elif len_kargs == 2:
                    if _u1 == None or _v1 == None:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    _nType = 1
                    _pos= args[0]
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_args == 2:
                if len_kargs == 1:
                    if _pos != None: 
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                    else:
                        _nType = 1
                        _u1 = args[0]
                        _v1 = args[1]
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_args == 3:
                if len_kargs == 1:
                    if _ref != None:
                        _nType = 4
                        _u1 = args[0]
                        _v1 = args[1]
                        _pos = args[2]
                    else:
                        if _pos != None:
                            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                        else:
                            _nType = 0
                            _x1 = args[0]
                            _x2 = args[1]
                            _x3 = args[2]
                elif len_kargs == 0:
                    if len(args[0]) == 3:
                        _nType = 1
                        _u1 = args[0]
                        _v1 = args[1]
                        _pos= args[2]
                    elif len(args[0]) == 6:
                        _nType = 2
                        _x1 = args[0]
                        _x2 = args[1]
                        _x3 = args[2]
                    else:
                        raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_args == 4:
                if len_kargs == 0:
                    _nType = 0
                    _x1 = args[0]
                    _x2 = args[1]
                    _x3 = args[2]
                    _pos= args[3]
                elif len_kargs == 1:
                    _nType = 3
                    _x1 = args[0]
                    _x2 = args[1]
                    _x3 = args[2]
                    _pos= args[3]
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            elif len_args == 5:
                if len_kargs == 0:
                    _nType = 0
                    _x1 = args[0]
                    _x2 = args[1]
                    _x3 = args[2]
                    _pos= args[3]
                    _ref= args[4]
                else:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        # check parameter type, value
        if _x1 != None: 
            _x1 = get_posx(_x1)
        if _x2 != None: 
            _x2 = get_posx(_x2)
        if _x3 != None: 
            _x3 = get_posx(_x3)
        if _pos != None: 
            _pos = get_posx(_pos)
        if _u1 !=None:
            if type(_u1) != list:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : u1")
            if len(_u1) != DR_VECTOR_U1_LEN:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : u1({0})".format(_u1))
        if _v1 !=None:
            if type(_v1) != list:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : v1")
            if len(_v1) != DR_VECTOR_V1_LEN:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : v1({0})".format(_v1))

        if _nType == 0:
            _ref = DR_BASE
            if __ROS2__:
                req = self.req_SetUserCartCoord2  
                req.x1  = [float(x) for x in _x1]
                req.x2  = [float(x) for x in _x2]
                req.x3  = [float(x) for x in _x3]
                req.pos = [float(x) for x in _pos]
                req.ref = _ref 

                future = self._ros2_set_user_cart_coord2.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('set_user_cart_coord Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = result.id            


        elif _nType == 1:
            _ref = DR_BASE
            if __ROS2__:
                req = SetUserCartCoord3.Request()  
                req.u1  = [float(x) for x in _u1]
                req.v1  = [float(x) for x in _v1]
                req.pos = [float(x) for x in _pos]
                req.ref = _ref 

                future = self._ros2_set_user_cart_coord3.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('set_user_cart_coord Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = result.id            

        elif _nType == 2:   #현재 미사용
            #_ref = DR_BASE
            #if __:
            #    req = set_user_cart_coord?(_x1, _x2, _x3, _ref)  
            #    ret = srv.id
            #else:
            #    ret = PythonMgr.py_set_user_cart_coord_ex2(_x1, _x2, _x3, _ref)
            #    #print_ext_result("{0} = PythonMgr.py_set_user_cart_coord_ex2(x1:{1}, x2:{2}, x3:{3}, ref:{4})".format(ret, _x1, _x2, _x3, _ref))
            pass
        elif _nType == 3:   # with ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
            if __ROS2__:
                req = self.req_SetUserCartCoord2  
                req.x1 = [float(x) for x in _x1]
                req.x2 = [float(x) for x in _x2]
                req.x3 = [float(x) for x in _x3]
                req.pos = [float(x) for x in _pos]
                req.ref = _ref 

                future = self._ros2_set_user_cart_coord2.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('set_user_cart_coord Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = result.id            

        elif _nType == 4:   # with ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
            if __ROS2__:
                req = SetUserCartCoord3.Request()  
                req.u1  = [float(x) for x in _u1]
                req.v1  = [float(x) for x in _v1]
                req.pos = [float(x) for x in _pos]
                req.ref = _ref 


                future = self._ros2_set_user_cart_coord3.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('set_user_cart_coord Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = result.id            

        elif _nType == 5:   # (pos, ref) 20191127 신규 추가 
            #print("new command !!!!!!!!!!!!!!!!")
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))
            if __ROS2__:
                req = self.req_SetUserCartCoord1  
                req.pos = [float(x) for x in _pos]
                req.ref = _ref 

                future = self._ros2_set_user_cart_coord1.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('set_user_cart_coord Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = result.id            

        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid argument list")

        return ret

    def overwrite_user_cart_coord(self, id, pos, ref=None, gol=None):

        # id
        _id = id

        # pos
        _pos = get_posx(pos)

        # ref
        if ref == None:
            _ref = DR_BASE
        else:
            _ref = ref
            if _ref != DR_BASE and _ref != DR_WORLD:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        # gol : #1 RCT 요청에 의하여 gol 무조건 0으로 변경 2019/12/09
        #if gol == None:
        #    _gol = 0
        #else:
        #    _gol = gol
        #    if _gol != 0 and _gol != 1:
        #        raise DR_Error(DR_ERROR_VALUE, "Invalid value : gol")
        _gol = 0

        if __ROS2__:
            #req = overwrite_user_cart_coord(_id, _pos, _ref, _gol)  
            req = self.req_OverwriteUserCartCoord
            req.id  = _id
            req.pos = [float(x) for x in _pos]
            req.ref = _ref 

            future = self._ros2_overwrite_user_cart_coord.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('overwrite_user_cart_coord Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.id            

        return ret

    def get_user_cart_coord(self, id):

        print("==============================> get_user_cart_coord(id)")
        if __ROS2__:
            req = self.req_GetUserCartCoord  
            req.id  = id

            future = self._ros2_get_user_cart_coord.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_user_cart_coord Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1, -1    
                else:        
                    pos = list(result.conv_posx)  # Convert tuple to list 
                    ref = result.ref
                    conv_posx = posx(pos)
                    print("==============================> get_user_cart_coord(id) 555 conv_posx={}, ref={}".format(conv_posx, ref))
                    return conv_posx, ref
        return 0

    def set_desired_force(self, fd=[0, 0, 0, 0, 0, 0], dir=[0, 0, 0, 0, 0, 0], time=0, mod=DR_FC_MOD_ABS):
        # df
        if type(fd) != list or len(fd) != 6:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : fd")

        if is_number(fd) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : fd({0})".format(fd))

        # dir
        if type(dir) != list or len(dir) != 6:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : dir")

        if is_number(dir) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : dir({0})".format(dir))

        # _ref
        _ref = _g_coord

        # time
        if type(time) != int and type(time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

        if time != -1 and time < 0:     # -1 : motion default
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))

        if time > 1:
            _time = 1
        else:
            _time = time

        # _mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
        _mod = mod

        if __ROS2__:
            req = self.req_SetDesiredForce  
            req.fd   = [float(x) for x in fd]
            req.dir  = dir
            req.ref  = _ref
            req.time = float(_time)
            req.mod  = _mod

            future = self._ros2_set_desired_force.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_desired_force Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret


    def release_force(self, time=0):
        if type(time) != int and type(time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time")

        if time != -1 and time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time({0})".format(time))

        if time > 1:
            _time = 1
        else:
            _time = time

        if __ROS2__:
            req = self.req_ReleaseForce 
            req.time = float(_time)

            future = self._ros2_release_force.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('release_force Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret


    #def check_position_condition(axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None):
    def check_position_condition(self, axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None, mod= DR_MV_MOD_ABS, pos=None):
        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

        if (axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

        # min
        if type(min) != int and type(min) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")

        # max
        if type(max) != int and type(max) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")

        if min == DR_COND_NONE and max == DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min,max))

        # min < max check
        if min != DR_COND_NONE and max != DR_COND_NONE: #min, max 값이 둘다 설정된 경우에만 검사 실시 2017/08/23
            if min > max:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min,max))

        # ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0})".format(mod))

        # _pos : DR_MV_MOD_REL 인 경우에는 반드시 pos 가 필요하고, otherwise 불필요
        if(mod == DR_MV_MOD_REL):
            if(pos==None):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : need to pos")
            else:
                _pos = get_normal_pos(pos, def_type=posx)
        else:
            if(pos!=None):
                _pos = get_normal_pos(pos, def_type=posx)
            else:
                pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                _pos = get_normal_pos(pos, def_type=posx)

        # check axis
        if (axis == DR_AXIS_A or axis == DR_AXIS_B or axis == DR_AXIS_C):
            _ref = DR_TOOL

        if __ROS2__:
            req = self.req_CheckPositionCondition  
            req.axis = axis
            req.min  = float(min)
            req.max  = float(max)
            req.ref  = _ref
            req.mode = mod
            req.pos = [float(x) for x in _pos]

            future = self._ros2_check_position_condition.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('check_position_condition Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret



    def check_force_condition(self, axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None):
        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

        if (axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z and \
            axis != DR_AXIS_A and axis != DR_AXIS_B and axis != DR_AXIS_C):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

        # min
        if type(min) != int and type(min) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")

        # max
        if type(max) != int and type(max) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")

        if min == DR_COND_NONE and max == DR_COND_NONE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))

        # min check : min 설정되어 있는데 0보다 작은 경우 에러 처리 2017/12/07
        if min != DR_COND_NONE:
            if min < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value: min (Ranges: min({0}) >= 0)".format(min))

        # max check : max 설정되어 있는데 0보다 작은 경우 에러 처리 2017/12/07
        if max != DR_COND_NONE:
            if max < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value: max (Ranges: max({0}) >= 0)".format(max))

        # min < max check
        if min != DR_COND_NONE and max != DR_COND_NONE: #min, max 값이 둘다 설정된 경우에만 검사 실시 2017/08/23
            if min > max:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))

        # ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

        # check axis
        if (axis == DR_AXIS_A or axis == DR_AXIS_B or axis == DR_AXIS_C):
            _ref = DR_TOOL

        if __ROS2__:
            req = self.req_CheckForceCondition  
            req.axis = axis
            req.min  = float(min)
            req.max  = float(max)
            req.ref  = _ref

            future = self._ros2_check_force_condition.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('check_force_condition Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret


    def check_orientation_condition(self, axis, min=None, max=None, ref=None, mod = None, pos=None):
        _cmd_type = 0

        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

        if (axis != DR_AXIS_A and axis != DR_AXIS_B and axis != DR_AXIS_C):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis({0})".format(axis))

        # min, max check type
        if(min != None):
            if type(min) == posx:
                _cmd_type = 0
            elif type(min) == posj:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : min is not posj")
            elif type(min) == list and len(min) == POINT_COUNT:
                _cmd_type = 0
            elif type(min) == int or type(min) == float:
                _cmd_type = 1
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")
        if(max != None):
            if type(max) == posx:
                _cmd_type = 0
            elif type(max) == posj:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : max is not posj")
            elif type(max) == list and len(max) == POINT_COUNT:
                _cmd_type = 0
            elif type(max) == int or type(max) == float:
                _cmd_type = 1
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")

        if _cmd_type == 0:

            # _pos_min
            if(min != None):
                _pos_min = get_normal_pos(min, def_type=posx)
            else:
                _pos_min = [DR_COND_NONE]*POINT_COUNT

            # _pos_max
            if(max != None):
                _pos_max = get_normal_pos(max, def_type=posx)
            else:
                _pos_max = [DR_COND_NONE]*POINT_COUNT

            # _ref
            if ref == None:
                _ref = _g_coord
            else:
                _ref = ref

            # check ref
            if type(_ref) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

            if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

            # mod
            if mod == None:
                mod = DR_MV_MOD_ABS
            else:
                if type(mod) != int:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod is not integer")

                if mod != DR_MV_MOD_ABS:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0}) is not DR_MV_MOD_ABS".format(mod))

            print("_cmd_type={0}, axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5}".format(_cmd_type, axis, _pos_min, _pos_max, _ref, mod) )

        elif _cmd_type == 1:
            # min
            if min == None:
                min = DR_COND_NONE
            else:
                if type(min) != int and type(min) != float:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid type : min")
            # max
            if max == None:
                max = DR_COND_NONE
            else:
                if type(max) != int and type(max) != float:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid type : max")

            if min == DR_COND_NONE and max == DR_COND_NONE:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))

            # min < max check
            if min != DR_COND_NONE and max != DR_COND_NONE: #min, max 값이 둘다 설정된 경우에만 검사 실시 2017/08/23
                if min > max:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value: min({0}), max({1})".format(min, max))

            # _ref
            if ref == None:
                _ref = _g_coord
            else:
                _ref = ref

            # check ref
            if type(_ref) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

            if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref({0})".format(_ref))

            # mod
            if mod == None:
                mod = DR_MV_MOD_REL
            else:
                if type(mod) != int:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod is not integer")

                if mod != DR_MV_MOD_REL:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod({0}) is not DR_MV_MOD_REL".format(mod))

            # _pos
            if(pos==None):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : need to pos")
            else:
                _pos = get_normal_pos(pos, def_type=posx)

            #print("_cmd_type={0}, axis:{1}, min:{2}, max:{3}, ref:{4}, mod:{5}, pos:{6}".format(_cmd_type, axis, min, max, _ref, mod, _pos) )

        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid arguments")


        if _cmd_type == 0:
            if __ROS2__:
                req = self.req_CheckOrientationCondition1
                req.axis    = axis
                req.min = [float(x) for x in _pos_min]
                req.max = [float(x) for x in _pos_max]
                req.ref     = _ref
                req.mode    = mod

                future = self._ros2_check_orientation_condition1.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('check_orientation_condition Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = 0 if (result.success == True) else -1     
        else:
            if __ROS2__:
                req = self.CheckOrientationCondition2  
                req.axis = axis
                req.min  = float(min)
                req.max  = float(max)
                req.ref  = _ref
                req.mode = mod
                req.pos = [float(x) for x in _pos]

                future = self._ros2_check_orientation_condition2.call_async(req)
                rclpy.spin_until_future_complete(g_node, future)

                try:
                    result = future.result()
                except Exception as e:
                    g_node.get_logger().info('check_orientation_condition Service call failed %r' % (e,))
                else:
                    if result == None:
                        ret = -1    
                    else:        
                        ret = 0 if (result.success == True) else -1     

        return ret

    def coord_transform(self, pose_in, ref_in=DR_BASE, ref_out=None):

        # _pos
        _pos = get_normal_pos(pose_in, def_type=posx)

        # ref_in
        if type(ref_in) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref_in")

        # ref_out
        if type(ref_out) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref_out")

        if __ROS2__:
            req = self.req_CoordTransform  
            req.pos_in  = [float(x) for x in _pos]
            req.ref_in  = ref_in
            req.ref_out = ref_out               
    
            future = self._ros2_coord_transform.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('coord_transform Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    pos = list(result.conv_posx)  # Convert tuple to list 
                    trans_posx = posx(pos)
                    ret = trans_posx
        return ret

##### I/O #########################################################################################################################
    def get_digital_input(self, index):

        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

        if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

        if __ROS2__:
            req = self.req_GetCtrlBoxDigitalInput
            req.index  = index

            future = self._ros2_get_digital_input.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_digital_input Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.value 

        return ret

    def get_analog_input(self, ch):
        if type(ch) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")

        if ch < 1 or ch > 2:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")

        if __ROS2__:
            req = self.req_GetCtrlBoxAnalogInput
            req.channel  = ch

            future = self._ros2_get_analog_input.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_analog_input Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.value 

        return ret

    def get_tool_digital_input(self, index):
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

        if index < DR_TDIO_MIN_INDEX or index > DR_TDIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

        if __ROS2__:
            req = self.req_GetToolDigitalInput
            req.index  = index

            future = self._ros2_get_tool_digital_input.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_tool_digital_input Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.value 

        return ret

    def set_digital_output(self, index, val=None):
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

        if val != None:
            if type(val) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")
            
            if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
            
            if val != ON and val != OFF:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        else: # # val ���ڰ� ������ simple style
            if (index < (-DR_DIO_MAX_INDEX)) or (index > DR_DIO_MAX_INDEX) or (index==0): # -16~+16
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
            else:
                if index < 0:
                    index = index*(-1) 
                    val = OFF
                else:
                    index = index
                    val = ON

        if __ROS2__:
            req = self.req_SetCtrlBoxDigitalOutput 
            req.index = index
            req.value = val

            future = self._ros2_set_digital_output.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_digital_output Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def get_digital_output(self, index):
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

        if index < DR_DIO_MIN_INDEX or index > DR_DIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

        if __ROS2__:
            req = self.req_GetCtrlBoxDigitalOutput
            req.index = index

            future = self._ros2_get_digital_output.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_digital_output Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.value 

        return ret


    def set_analog_output(self, ch, val):
        global _g_analog_output_mode_ch1
        global _g_analog_output_mode_ch2

        if type(ch) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")

        if type(val) != int and type(val) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")

        if ch < 1 or ch > 2:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")

        #if val < 0 or val > 20.0:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        if ch == 1:
            if _g_analog_output_mode_ch1 == DR_ANALOG_CURRENT:
                if val < 4 or val > 20.0:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
            elif _g_analog_output_mode_ch1 == DR_ANALOG_VOLTAGE:
                if val < 0 or val > 10.0:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
            else: 
                raise DR_Error(DR_ERROR_VALUE, "Analog output mode(ch1) is not set")
        if ch == 2:
            if _g_analog_output_mode_ch2 == DR_ANALOG_CURRENT:
                if val < 4 or val > 20.0:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
            elif _g_analog_output_mode_ch2 == DR_ANALOG_VOLTAGE:
                if val < 0 or val > 10.0:
                    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
            else: 
                raise DR_Error(DR_ERROR_VALUE, "Analog output mode(ch2) is not set")

        if __ROS2__:
            req = self.req_SetCtrlBoxAnalogOutput 
            req.channel = ch
            req.value   = float(val)

            future = self._ros2_set_analog_output.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_analog_output Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def set_mode_analog_output(self, ch, mod):
        global _g_analog_output_mode_ch1
        global _g_analog_output_mode_ch2

        if type(ch) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")

        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if ch < 1 or ch > 2:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")

        if mod != DR_ANALOG_CURRENT and mod !=DR_ANALOG_VOLTAGE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        if ch == 1: 
            _g_analog_output_mode_ch1 = mod
        if ch == 2: 
            _g_analog_output_mode_ch2 = mod

        if __ROS2__:
            req = self.req_SetCtrlBoxAnalogOutputType
            req.channel  = ch
            req.mode = mod

            future = self._ros2_set_mode_analog_output.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_mode_analog_output Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def set_mode_analog_input(self, ch, mod):
        if type(ch) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ch")

        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if ch < 1 or ch > 2:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ch")

        if mod != DR_ANALOG_CURRENT and mod !=DR_ANALOG_VOLTAGE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        if __ROS2__:
            req = self.req_SetCtrlBoxAnalogInputType
            req.channel  = ch
            req.mode = mod

            future = self._ros2_set_mode_analog_input.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_mode_analog_input Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret


    def set_tool_digital_output(self, index, val=None):
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

        if val != None:
            if type(val) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")

            if index < DR_TDIO_MIN_INDEX or index > DR_TDIO_MAX_INDEX:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

            if val != ON and val != OFF:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
        else: # val ���ڰ� ������ simple style
            if (index < (-DR_TDIO_MAX_INDEX)) or (index > DR_TDIO_MAX_INDEX) or (index==0): # -6~+6
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")
            else:
                if index < 0:
                    index = index*(-1) 
                    val = OFF
                else:
                    index = index
                    val = ON

        if __ROS2__:
            req = self.req_SetToolDigitalOutput   
            req.index = index
            req.value = val

            future = self._ros2_set_tool_digital_output.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_tool_digital_output Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def get_tool_digital_output(self, index):
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

        if index < DR_TDIO_MIN_INDEX or index > DR_TDIO_MAX_INDEX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index")

        if __ROS2__:
            req = self.req_GetToolDigitalOutput
            req.index = index


            future = self._ros2_get_tool_digital_output.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_tool_digital_output Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.value 

        return ret


##### Modbus #########################################################################################################################

    def add_modbus_signal(self, ip, port, name, reg_type, index, value=0, slaveid=255):
        # ip
        if type(ip) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ip")

        #try:
        #    ipaddress.ip_address(ip)
        #except:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : ip")

        # port
        if type(port) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : port")

        if port <= 0 or port > 65535:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : port (Ranges: 1 ~ 65535)")

        # name
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # reg_type
        if type(reg_type) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : reg_type")

        if reg_type != DR_MODBUS_DIG_INPUT and reg_type != DR_MODBUS_DIG_OUTPUT and reg_type != DR_MODBUS_REG_INPUT and reg_type != DR_MODBUS_REG_OUTPUT:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : reg_type (Ranges : 0 ~ 3")

        # index
        if type(index) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : index")

        if index < 0 or index > 65535:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : index (Ranges: 0 ~ 65535)")

        # value
        if type(value) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : value")

        if value < 0 or value > 65535:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : value (Ranges: 0 ~ 65535)")

        # check value
        if reg_type == DR_MODBUS_DIG_OUTPUT or reg_type == DR_MODBUS_REG_OUTPUT:
            _value = value
        else:
            _value = 0

        # slaveid
        if type(slaveid) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : slaveid")
        if slaveid < 0 or slaveid > 255:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : slaveid (Ranges: 0 or 1~247 or 255)")
        elif slaveid > 247 and slaveid < 255:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : slaveid (Ranges: 0 or 1~247 or 255)")

        # ROS service call
        if __ROS2__:
            req = self.req_ConfigCreateModbus
            req.name     = name
            req.ip       = ip
            req.port     = port
            req.reg_type = reg_type
            req.index    = index
            req.value    = _value
            req.slave_id = slaveid

            future = self._ros2_add_modbus_signal.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('add_modbus_signal Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def del_modbus_signal(self, name):
        # name
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS2__:
            req = self.req_ConfigDeleteModbus 
            req.name = name

            future = self._ros2_del_modbus_signal.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('del_modbus_signal Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def set_modbus_output(self, iobus, val):

        if type(iobus) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : iobus")

        if type(val) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : val")

        # not check value (register : 2byte...)
        # if val != ON and val != OFF:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")
    
        # ROS service call
        if __ROS2__:
            req = self.req_SetModbusOutput 
            req.name  = iobus
            req.value = val

            future = self._ros2_set_modbus_output.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_modbus_output Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def get_modbus_input(self, iobus):
        if type(iobus) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : iobus")

        # ROS service call
        if __ROS2__:
            req = self.req_GetModbusInput
            req.name  = iobus

            future = self._ros2_get_modbus_input.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_modbus_input Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.value 

        return ret

##### TCP/TOOL #########################################################################################################################
    def set_tcp(self, name):

        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # not check value (register : 2byte...)
        # if val != ON and val != OFF:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")

        # ROS service call
        if __ROS2__:
            req = self.req_SetCurrentTcp
            req.name = name

            future = self._ros2_set_current_tcp.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_tcp Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def get_tcp(self):
        # ROS service call
        if __ROS2__:
            req = self.req_GetCurrentTcp        

            future = self._ros2_get_current_tcp.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_tcp Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.info 

        return ret

    def set_tool(self, name):

        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # not check value (register : 2byte...)
        # if val != ON and val != OFF:
        #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : val")

        # ROS service call
        if __ROS2__:
            req = self.req_SetCurrentTool    
            req.name = name

            future = self._ros2_set_current_tool.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_tool Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret


    def get_tool(self):
        # ROS service call
        if __ROS2__:
            req = self.req_GetCurrentTool 

            future = self._ros2_get_current_tool.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_tool Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.info 

        return ret

    def set_tool_shape(self, name):

        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        if __ROS2__:
            req = self.req_SetToolShape  
            req.name = name

            future = self._ros2_set_tool_shape.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_tool_shape Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def add_tcp(self, name, pos):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS2__:
            req = self.req_ConfigCreateTcp  
            req.name = name
            req.pos = [float(x) for x in pos]

            future = self._ros2_config_create_tcp.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('add_tcp Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def del_tcp(self, name):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS2__:
            req = self.req_ConfigDeleteTcp  
            req.name = name

            future = self._ros2_config_delete_tcp.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('del_tcp Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def add_tool(self, name, weight, cog, inertia):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS2__:
            req = self.req_ConfigCreateTool 
            req.name    = name
            req.weight  = float(weight)
            req.cog     = [float(x) for x in cog]
            req.inertia = [float(x) for x in inertia]        

            future = self._ros2_config_create_tool.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('add_tool Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

    def del_tool(self, name):
        if type(name) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS2__:
            req = self.req_ConfigDeleteTool  
            req.name    = name

            future = self._ros2_config_delete_tool.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('del_tool Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        return ret

##### DRL #########################################################################################################################

    def drl_script_run(self, robotSystem, code):
        if type(code) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : name")

        # ROS service call
        if __ROS2__:
            req = self.req_DrlStart
            req.robot_system = robotSystem
            req.code         = code

            future = self._ros2_drl_start.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('drl_script_run Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret


    def drl_script_stop(self, stop_mode):
        if type(stop_mode) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : stop_mode")
        print("drl_script_stop")
        # ROS service call
        if __ROS2__:
            req = self.req_DrlStop  
            req.stop_mode = stop_mode

            future = self._ros2_drl_stop.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('drl_script_stop Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret

    def drl_script_pause(self):
        # ROS service call
        if __ROS2__:
            req = self.req_DrlPause 

            future = self._ros2_drl_pause.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('drl_script_pause Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret

    def drl_script_resume(self):
        # ROS service call
        if __ROS2__:
            req = self.req_DrlResume  

            future = self._ros2_drl_resume.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('drl_script_resume Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        return ret

    def get_drl_state(self):
        # ROS service call
        if __ROS2__:
            req = self.req_GetDrlState  

            future = self._ros2_get_drl_state.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('get_drl_state Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = result.drl_state 

        return ret