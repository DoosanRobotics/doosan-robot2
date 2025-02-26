'''
/*********************************************************************
 *
 * Doosan Robot Framework Constant
 * Author: Lee Jeong-Woo(jeongwoo1.lee@doosan.com)
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
'''

import sys
sys.dont_write_bytecode = True

#
# Robot configuration
#
NUM_JOINT                             = 6
NUMBER_OF_JOINT                       = 6  
NUM_TASK                              = 6
NUM_FLANGE_IO                         = 6
NUM_BUTTON                            = 5
                                      
#                                     
# string                              
#                                                                          
MAX_STRING_SIZE                       = 256
MAX_SYMBOL_SIZE                       = 32
                                      
#                                     
# I/O configuration                   
#                                     
NUM_DIGITAL                           = 16
MAX_DIGITAL_BURST_SIZE                = 16
NUM_ANALOG                            = 2
NUM_SWITCH                            = 3
NUM_SAFETY_IN                         = 2
NUM_ENCORDER                          = 2
NUM_POWER_OUT                         = 1
MAX_MODBUS_TOTAL_REGISTERS            = 100                                      
MAX_MOVEB_POINT                       = 50
MAX_SPLINE_POINT                      = 100
                                      
#                                     
# robot state                         
#                                     
STATE_INITIALIZING                    = 0
STATE_STANDBY                         = 1
STATE_MOVING                          = 2
STATE_SAFE_OFF                        = 3
STATE_TEACHING                        = 4
STATE_SAFE_STOP                       = 5
STATE_EMERGENCY_STOP                  = 6
STATE_HOMMING                         = 7
STATE_RECOVERY                        = 8
STATE_SAFE_STOP2                      = 9
STATE_SAFE_OFF2                       = 10
STATE_RESERVED1                       = 11
STATE_RESERVED2                       = 12
STATE_RESERVED3                       = 13
STATE_RESERVED4                       = 14
STATE_NOT_READY                       = 15
                                      
#                                     
# robot control                       
#                                     
CONTROL_INIT_CONFIG                   = 0
CONTROL_ENABLE_OPERATION              = 1
CONTROL_RESET_SAFET_STOP              = 2
CONTROL_RESET_SAFET_OFF               = 3
CONTROL_SERVO_ON = CONTROL_RESET_SAFET_OFF
CONTROL_RECOVERY_SAFE_STOP            = 4
CONTROL_RECOVERY_SAFE_OFF             = 5
CONTROL_RECOVERY_BACKDRIVE            = 6
CONTROL_RESET_RECOVERY                = 7
                                      
#                                     
# speed mode                          
SPEED_NORMAL_MODE                     = 0
SPEED_REDUCED_MODE                    = 1
                                      
#                                     
# robot system                        
#                                     
ROBOT_SYSTEM_REAL                     = 0 
ROBOT_SYSTEM_VIRTUAL                  = 1
                                      
#                                     
# robot mode                          
#                                     
ROBOT_MODE_MANUAL                     = 0
ROBOT_MODE_AUTONOMOUS                 = 1
ROBOT_MODE_MEASURE                    = 2
                                      
#                                     
# get robot space    
#                                     
ROBOT_SPACE_JOINT                     = 0
ROBOT_SPACE_TASK                      = 1
                                      
#                                     
# jog axis                            
#                                     
JOG_AXIS_JOINT_1                      = 0
JOG_AXIS_JOINT_2                      = 1
JOG_AXIS_JOINT_3                      = 2
JOG_AXIS_JOINT_4                      = 3
JOG_AXIS_JOINT_5                      = 4
JOG_AXIS_JOINT_6                      = 5
JOG_AXIS_TASK_X                       = 6
JOG_AXIS_TASK_Y                       = 7
JOG_AXIS_TASK_Z                       = 8
JOG_AXIS_TASK_RX                      = 9
JOG_AXIS_TASK_RY                      = 10
JOG_AXIS_TASK_RZ                      = 11
                                      
#                                     
# motion command axis (joint)         
#                                     
JOINT_AXIS_1                          = 0
JOINT_AXIS_2                          = 1
JOINT_AXIS_3                          = 2
JOINT_AXIS_4                          = 3
JOINT_AXIS_5                          = 4
JOINT_AXIS_6                          = 5
                                      
#                                     
# motion command axis (task)          
#                                     
TASK_AXIS_X                           = 0
TASK_AXIS_Y                           = 1
TASK_AXIS_Z                           = 2
                                      
#
# reference coordinate
COORDINATE_SYSTEM_BASE                = 0
COORDINATE_SYSTEM_TOOL                = 1
COORDINATE_SYSTEM_WORLD               = 2
COORDINATE_SYSTEM_USER_MIN            = 101
COORDINATE_SYSTEM_USER_MAX            = 200

#                                     
# move command reference type         
MOVE_REFERENCE_BASE                   = COORDINATE_SYSTEM_BASE
MOVE_REFERENCE_TOOL                   = COORDINATE_SYSTEM_TOOL
MOVE_REFERENCE_WORLD                  = COORDINATE_SYSTEM_WORLD
MOVE_REFERENCE_USER_MIN               = COORDINATE_SYSTEM_USER_MIN
MOVE_REFERENCE_USER_MAX               = COORDINATE_SYSTEM_USER_MAX
                                      
#                                     
# move command mode type              
#                                     
MOVE_MODE_ABSOLUTE                    = 0
MOVE_MODE_RELATIVE                    = 1
                                      
#                                     
# mode of path              
#                                     
PATH_MODE_DPOS                        = 0
PATH_MODE_DVEL                        = 1 

#                                     
# blending speed type                 
#                                     
BLENDING_SPEED_TYPE_DUPLICATE         = 0
BLENDING_SPEED_TYPE_OVERRIDE          = 1
                                      
#                                     
# reset safety stop type              
#                                     
SAFE_STOP_RESET_TYPE_DEFAULT          = 0
SAFE_STOP_RESET_TYPE_PROGRAM_STOP     = SAFE_STOP_RESET_TYPE_DEFAULT
SAFE_STOP_RESET_TYPE_PROGRAM_RESUME   = 1
                                      
#                                     
# acces control management            
#                                     
MANAGE_ACCESS_CONTROL_FORCE_REQUEST   = 0
MANAGE_ACCESS_CONTROL_REQUEST         = 1
MANAGE_ACCESS_CONTROL_RESPONSE_YES    = 2
MANAGE_ACCESS_CONTROL_RESPONSE_NO     = 3
                                      
#                                     
# access control state                
#                                     
MONITORING_ACCESS_CONTROL_REQUEST     = 0
MONITORING_ACCESS_CONTROL_DENY        = 1
MONITORING_ACCESS_CONTROL_GRANT       = 2
MONITORING_ACCESS_CONTROL_LOSS        = 3
                                      
#                                     
# motion stop type                    
#                                     
STOP_TYPE_QUICK_STO                   = 0
STOP_TYPE_QUICK                       = 1
STOP_TYPE_SLOW                        = 2
STOP_TYPE_HOLD                        = 3
STOP_TYPE_EMERGENCY = STOP_TYPE_HOLD
                                      
#                                     
# spline velocity option              
#                                     
SPLINE_VELOCITY_OPTION_DEFAULT        = 0
SPLINE_VELOCITY_OPTION_CONST          = 1
                                      
                                      
#                                     
# gpio index                          
#                                     
GPIO_CTRLBOX_DIGITAL_INDEX_1          = 0
GPIO_CTRLBOX_DIGITAL_INDEX_2          = 1
GPIO_CTRLBOX_DIGITAL_INDEX_3          = 2
GPIO_CTRLBOX_DIGITAL_INDEX_4          = 3
GPIO_CTRLBOX_DIGITAL_INDEX_5          = 4
GPIO_CTRLBOX_DIGITAL_INDEX_6          = 5
GPIO_CTRLBOX_DIGITAL_INDEX_7          = 6
GPIO_CTRLBOX_DIGITAL_INDEX_8          = 7
GPIO_CTRLBOX_DIGITAL_INDEX_9          = 8
GPIO_CTRLBOX_DIGITAL_INDEX_10         = 9
GPIO_CTRLBOX_DIGITAL_INDEX_11         = 10
GPIO_CTRLBOX_DIGITAL_INDEX_12         = 11
GPIO_CTRLBOX_DIGITAL_INDEX_13         = 12
GPIO_CTRLBOX_DIGITAL_INDEX_14         = 13
GPIO_CTRLBOX_DIGITAL_INDEX_15         = 14
GPIO_CTRLBOX_DIGITAL_INDEX_16         = 15
                                      
GPIO_CTRLBOX_ANALOG_INDEX_1           = 0
GPIO_CTRLBOX_ANALOG_INDEX_2           = 1
                                      
GPIO_TOOL_DIGITAL_INDEX_1             = 0
GPIO_TOOL_DIGITAL_INDEX_2             = 1
GPIO_TOOL_DIGITAL_INDEX_3             = 2
GPIO_TOOL_DIGITAL_INDEX_4             = 3
GPIO_TOOL_DIGITAL_INDEX_5             = 4
GPIO_TOOL_DIGITAL_INDEX_6             = 5

#
# program running state
#
DRL_PROGRAM_STATE_PLAY                = 0
DRL_PROGRAM_STATE_STOP                = 1
DRL_PROGRAM_STATE_HOLD                = 2
                                     
# analog I/O type                    
#                                    
GPIO_ANALOG_TYPE_CURRENT              = 0
GPIO_ANALOG_TYPE_VOLTAGE              = 1
                                     
# modbus registertype
#
MODBUS_REGISTER_TYPE_DISCRETE_INPUTS  = 0
MODBUS_REGISTER_TYPE_COILS            = 1
MODBUS_REGISTER_TYPE_INPUT_REGISTER   = 2
MODBUS_REGISTER_TYPE_HOLDING_REGISTER = 3 

#
# program response
#
PROGRAM_STOP_CAUSE_NORMAL             = 0
PROGRAM_STOP_CAUSE_FORCE              = 1
PROGRAM_STOP_CAUSE_ERROR              = 2
                                    
#                                   
# MoveB blending type               
#                                   
MOVEB_BLENDING_TYPE_LINE              = 0
MOVEB_BLENDING_TYPE_CIRLCE            = 1

#
# force command axis type
#
FORCE_AXIS_X                          = 0
FORCE_AXIS_Y                          = 1  
FORCE_AXIS_Z                          = 2 
FORCE_AXIS_A                          = 10
FORCE_AXIS_B                          = 11  
FORCE_AXIS_C                          = 12

#
# force control mode type
#
FORCE_MODE_ABSOLUTE                   = 0
FORCE_MODE_RELATIVE                   = 1  

#                                   
# log level                         
#                              
LOG_LEVEL_SYSINFO                     = 1
LOG_LEVEL_SYSWARN                     = 2
LOG_LEVEL_SYSERROR                    = 3

#
# log group
#
LOG_GROUP_SYSTEMFMK                   = 1
LOG_GROUP_MOTIONLIB                   = 2
LOG_GROUP_SMARTTP                     = 3
LOG_GROUP_INVERTER                    = 4
LOG_GROUP_SAFETYCONTROLLER            = 5


MESSAGE_LEVEL_INFO                    = 0
MESSAGE_LEVEL_WARN                    = 1 
MESSAGE_LEVEL_ALARM                   = 2


#
# log code(eLOG_GROUP_SYSTEMFMK)
#

# eLOG_GROUP_SYSTEMFMK(SYSTEM)

OPERATION_FRAMEWORK_START                               = 1000
OPERATION_FRAMEWORK_START_FAILED                        = 1001
OPERATION_FRAMEWORK_STOP                                = 1002

COMMAND_SYSTEM_SHUTDOWN                                 = 1011
COMMAND_SYSTEM_RESTART                                  = 1012
COMMAND_SET_TIME                                        = 1013
COMMAND_SET_IPADDRESS                                   = 1014

OPERATION_CONFIG_LOAD_FAIL                              = 1017
OPERATION_SYSTEM_STATE_CHANGED                          = 1018
OPERATION_3STATESWITCH_RELEASED                         = 1019
OPERATION_HOMMING_COMPLETED                             = 1020
OPERATION_NOT_EXIT_SAME_TCPNAME                         = 1021
OPERATION_NOT_EXIT_SAME_TOOLNAME                        = 1022
OPERATION_EXTERNAL_STO_SINNAL_INPUTED                   = 1023
OPERATION_EXTERNAL_EMG_SINNAL_INPUTED                   = 1024
OPERATION_EXTERNAL_PRS_SINNAL_INPUTED                   = 1025
OPERATION_EXTERNAL_REDUCED_SPEED_MODE_SINNAL_INPUTED    = 1026
OPERATION_COLLISION_DETECTION_STOP_FAIL                 = 1027
OPERATION_COLLISION_DETECTION_RESTART                   = 1028
OPERATION_RECOVERY_MODE_FORCE_START                     = 1029
OPERATION_CONFIG_FILE_NAME_EMPTY                        = 1030
OPERATION_TOOL_NAME_EXCEED_MAX_NUMBER                   = 1031
OPERATION_TCP_NAME_EXCCED_MAX_NUMBER                    = 1032
OPERATION_CONFIG_TOOL_FAIL                              = 1033
OPERATION_CONFIG_TCP_FAIL                               = 1034
OPERATION_EXTERNAL_DIRECT_TEACH_MODE_SINNAL_INPUTED     = 1035
OPERATION_EXTERNAL_MANUAL_GUIDE_STOP_SINNAL_INPUTED     = 1036
OPERATION_MASTERING_PROCESS_NEEDED                      = 1037
OPERATION_SYSTEM_IDLE_OFF                               = 1038

#eLOG_GROUP_SYSTEMFMK(INTERPRET)

COMMAND_RUN_PROGRAM                                     = 2002

COMMAND_STOP_PROGRAM                                    = 2004
COMMAND_PAUSE_PROGRAM                                   = 2005
COMMAND_RESUME_PROGRAM                                  = 2006

OPERATION_PROGRAM_NORMAL_STOP                           = 2007
OPERATION_PROGRAM_FORCED_STOP                           = 2008
OPERATION_PROGRAM_FORCED_ERROR_STOP                     = 2009

OPERATION_PROGRAM_SW_ERROR                              = 2010
OPERATION_PROGRAM_INTERNAL_ERROR                        = 2011
OPERATION_PROGRAM_INIT_ERROR                            = 2012
OPERATION_PROGRAM_EMPTY_SCRIPT                          = 2013

OPERATION_PROGRAM_SYNTAX_EXCEPTION                      = 2014
OPERATION_PROGRAM_SYNTAX_ERROR                          = 2015

OPERATION_PROGRAM_RUNTIME_TYPE_ERROR                    = 2016
OPERATION_PROGRAM_RUNTIME_VALUE_ERROR                   = 2017
OPERATION_PROGRAM_RUNTIME_RUNTIME_ERROR                 = 2018
OPERATION_PROGRAM_RUNTIME_EXCEPTION                     = 2019

#eLOG_GROUP_SYSTEMFMK(ETHETNET)

OPERATION_SERVER_START                                  = 3000
OPERATION_SERVER_STOP                                   = 3001
OPERATION_CLIENT_CONNECTED                              = 3002
OPERATION_CLIENT_DISCONNECTED                           = 3003
OPERATION_CLIENT_AUTHENTIFICATION_FAIL                  = 3004
OPERATION_UNKNOWN_CLIENT_CONNECTED                      = 3005
OPERATION_SEND_QUEUE_OVERFLOW                           = 3006
OPERATION_SEND_COMMAND_FAIL                             = 3007


#eLog_GROUP_SYSTEMFMK(ETHERCAT)

OPERATION_ECAT_MASTER_INIT_FAILED                       = 4000
OPERATION_ECAT_INVALID_SLAVE_TYPE                       = 4001
OPERATION_ECAT_JOBTASK_INIT_FAILED                      = 4002
OPERATION_ECAT_INVALID_ENI_FILE                         = 4003
OPERATION_ECAT_DC_NOT_SUPPORTED                         = 4004
OPERATION_ECAT_DC_INIT_FAILED                           = 4005
OPERATION_ECAT_REG_NOTIFY_CALLBACK_FAILED               = 4006
OPERATION_ECAT_INVALID_VENDOR_PRODUCT_CODE              = 4007
OPERATION_ECAT_CHANGE_INIT_STAT_FAILED                  = 4008
OPERATION_ECAT_CHANGE_PREOP_STAT_FAILED                 = 4009
OPERATION_ECAT_SLAVE_CONFIG_FAILED                      = 4010
OPERATION_ECAT_CONFIG_PDO_FAILED                        = 4011
OPERATION_ECAT_CHANGE_SAFEOP_STAT_FAILED                = 4012
OPERATION_ECAT_GET_DCM_STATE_FAILED                     = 4013
OPERATION_ECAT_CHANGE_OP_STAT_FAILED                    = 4014
OPERATION_ECAT_MASTER_STOP_FAILED                       = 4015
OPERATION_ECAT_UNREG_NOTIFY_CALLBACK_FAILED             = 4016
OPERATION_ECAT_MASTER_RELEASE_FAILED                    = 4017
OPERATION_ECAT_CHANGE_BOOTSTRAP_FAILED                  = 4018
OPERATION_ECAT_OPEN_UPDATE_FILE_FAILED                  = 4019
OPERATION_ECAT_INVERTER_UPDATE_FAILED                   = 4020
OPERATION_ECAT_INVALID_UPDATE_FILE                      = 4021
OPERATION_ECAT_JOBTASK_RECV_RXFRAME_FAILED              = 4022
OPERATION_ECAT_JOBTASK_SEND_CYCFRAME_FAILED             = 4023
OPERATION_ECAT_JOBTASK_DC_SYNC_FAILED                   = 4034
OPERATION_ECAT_JOBTASK_SEND_ACYCFRAME_FAILED            = 4025
OPERATION_ECAT_NOTIFY_ERROR                             = 4026

#eLOG_GROUP_SYSTEMFMK(SERIAL)

OPERATION_SERIAL_CONNECT_SUCCESS                        = 5000
OPERATION_SERIAL_CONNECT_FAIL                           = 5001 
OPERATION_SERIAL_DISCONNECT                             = 5002
OPERATION_READ_SERIAL_FAIL                              = 5003
OPERATION_READ_SERIAL_TIMEOUT = OPERATION_READ_SERIAL_FAIL
OPERATION_WRITE_SERIAL_FAIL                             = 5004
OPERATION_READ_CRC_ERROR                                = 5005
OPERATION_UNSUPPORTED_PROTOCOL                          = 5006

OPERATION_OUTOFRANGE_GPIO                               = 5013
OPERATION_READ_SEQUENCE_ERROOR                          = 5014

#eLOG_GROUP_SYSTEMFMK(MODBUS)

OPERATION_MODBUS_CONNECT_SUCCESS                        = 6000
OPERATION_MODBUS_CONNECT_FAIL                           = 6001

OPERATION_EXIST_SAME_REGADDRESS                         = 6003
OPERATION_ADDVARIABLE_FAIL                              = 6004 

OPERATION_NOT_EXIT_SAME_NAME                            = 6006

OPERATION_DELVARIABLE_FAIL                              = 6008
OPERATION_SETVARIABLE_FAIL                              = 6009
OPERATION_MODBUS_SLAVE_EXCEED_MAX_NUMBER                = 6010
OPERATION_MODBUS_REGISTER_EXCEED_MAX_NUMBER             = 6011
OPERATION_MODBUS_LOAD_MODULE_FAIL                       = 6012
OPERATION_GETVARIABLE_FAIL                              = 6013

#
# log code(eLOG_GROUP_MOTIONLIB)
#

RC_ERROR_NO_ERROR                                       = 0

# InputError
RC_ERROR_INPUT_TOOL_WEIGHT                              = 1101
RC_ERROR_INPUT_TCP                                      = 1102
RC_ERROR_INPUT_JNT_LIMIT_SET                            = 1103
RC_ERROR_INPUT_TCP_CONSTRAINED_MOVE_RXRYRZ              = 1104

RC_ERROR_INPUT_NOT_ENOUGH_VIA_POINT_NUM                 = 1201
RC_ERROR_INPUT_UNEXPECTED_NEW_MTN_CMD                   = 1202
RC_ERROR_INPUT_UNEXPECTED_NEW_MTN_CMD_BLENDING          = 1203
RC_ERROR_INPUT_CIRC_MTN_CMD                             = 1204
RC_ERROR_INPUT_BLENDING_RAD                             = 1205
RC_ERROR_INPUT_OUT_OF_WORKSPACE_CMD                     = 1206
RC_ERROR_INPUT_OUT_OF_USER_JOINT_LIMIT                  = 1207
RC_ERROR_INPUT_OUT_OF_USER_JVEL_LIMIT                   = 1208
RC_ERROR_INPUT_OUT_OF_USER_TVEL_LIMIT                   = 1209
RC_ERROR_INPUT_UNEXPECTED_TARGET_AXIS                   = 1210
RC_ERROR_INPUT_UNEXPECTED_TARGET_TIME                   = 1211
RC_ERROR_INPUT_OUT_OF_TARGET_TIME_LIMIT                 = 1212
RC_ERROR_INPUT_OUT_OF_TARGET_TIME_LIMIT_INFO            = 1214

RC_ERROR_INPUT_PARALLEL_AXIS                            = 1301
RC_ERROR_INPUT_ALIGN_AXIS                               = 1302

RC_ERROR_INPUT_INCORRECT_POSITION_MAX_MIN               = 1401
RC_ERROR_INPUT_INCORRECT_FORCE_MAX_MIN                  = 1402

RC_ERROR_EST_INSTALL_POSE                               = 1501
RC_ERROR_INPUT_INVALID_PROTECTZONE                      = 1502
RC_ERROR_INPUT_INVALID_COLLISION_MUTE_ZONE              = 1503
RC_ERROR_INPUT_INVALID_TOOL_ANGLE_LIMIT_ZONE            = 1504
RC_ERROR_INPUT_INVALID_TOOL_SHAPE                       = 1505
RC_ERROR_INPUT_INVALID_ROBOT_SHAPE                      = 1506
RC_ERROR_INPUT_INVALID_SAFE_ZONE                        = 1507

RC_ERROR_INPUT_VIRTUAL_FENCE_SETTING                    = 1601
RC_ERROR_LIMIT_MAX_TCP_POSITON_VF                       = 1602
RC_ERROR_INPUT_MIN_VALUE_VF                             = 1603

RC_ERROR_DRCL_STATE_INVALID_EVENT                       = 1903
RC_ERROR_MATH_CALCULATION                               = 1904

RC_ERROR_LIMIT_MAX_UCPOSITON                            = 2501
RC_ERROR_LIMIT_MAX_UCVELOCITY                           = 2502
RC_ERROR_LIMIT_MAX_JTS                                  = 2503
RC_ERROR_LIMIT_MAX_POWER                                = 2504
RC_ERROR_LIMIT_MAX_FORCE                                = 2505
RC_ERROR_LIMIT_MAX_MOMENTUM                             = 2506
RC_ERROR_LIMIT_MAX_SPEED                                = 2507

RC_ERROR_MOVESX_CONSTANT_SPD_UNAVAILABLE                = 3213
RC_ERROR_JOG_JNT_LIMIT                                  = 3215
RC_ERROR_JOG_TSK_LIMIT                                  = 3216
RC_ERROR_JOG_SINGULARITY                                = 3217


RC_ERROR_APP_AUTOTOOLMEASURE_SINGULARITY                = 3301

RC_ERROR_MTN_COMPLIANCE_SINGULARITY                     = 3403

RC_ERROR_SYSTEM_JTS_VALUE_COLLISION                     = 3501
RC_ERROR_SYSTEM_JTS_VALUE_VF                            = 3502
RC_ERROR_SYSTEM_JTS_VALUE_COMP_CTRL                     = 3503
RC_ERROR_SYSTEM_JTS_VALUE_TEACHING_CTRL                 = 3504
RC_ERROR_SYSTEM_JTS_VALUE_AUTO_COMPENSATION             = 3505
RC_ERROR_SYSTEM_JTS_CAL                                 = 3507
RC_ERROR_MTN_SINGULARITY                                = 3509

RC_ERROR_CALC_INVERSE_KINEMATICS                        = 3905

RC_ERROR_SYSTEM_JTS_VALUE_BOOT                          = 4501
RC_ERROR_SYSTEM_JTS_VALUE                               = 4508

RC_ERROR_APP_FAIL_JTS_CALIBRATION                       = 6301
RC_ERROR_APP_FAIL_MEASURE_INSTALLPOSE                   = 6302
RC_ERROR_APP_FAIL_MEASURE_TOOL_INFO                     = 6303
RC_ERROR_APP_FAIL_MEASURE_TCP                           = 6304
RC_ERROR_INPUT_NOT_VALID_MTN_PARAM                      = 1218  #180117
RC_ERROR_MANUAL_GUIDING_SAFETY                          = 9301  #180212

RC_ERROR_FORCE_INPUT                                    = 9401

#
# log code(eLOG_GROUP_INVERTER)
#

# Inverter Fault Code
OPERATION_INVT_NO_ERROR                                 = 1000
OPERATION_INVT_UNKNOWN_ERROR                            = 1001
OPERATION_INVT_SDO_COMM_ERROR                           = 1002
OPERATION_INVT_OVER_CURRENT_ERROR                       = 1003 # 0x2310
OPERATION_INVT_OVER_VOLTAGE_ERROR                       = 1004 # 0x3210
OPERATION_INVT_UNDER_VOLTAGE_ERROR                      = 1005 # 0x3220
OPERATION_INVT_OVER_TEMPERATURE_ERROR                   = 1006 # 0x4210
OPERATION_INVT_UNDER_TEMPERATURE_ERROR                  = 1007 # 0x4220
OPERATION_INVT_SUPPLY_24V_LOW_VOLTAGE_ERROR             = 1008 # 0x5112
OPERATION_INVT_SUPPLY_5V_LOW_VOLTAGE_ERROR              = 1009 # 0x5113
OPERATION_INVT_SUPPLY_17V_LOW_VOLTAGE_ERROR             = 1010 # 0x5114
OPERATION_INVT_SUPPLY_24BRK_LOW_VOLTAGE_ERROR           = 1011 # 0x5115
OPERATION_INVT_SUPPLY_DSPIO_VOLTAGE_ERROR               = 1012 # 0x5116
OPERATION_INVT_SUPPLY_CORE_VOLTAGE_ERROR                = 1013 # 0x5117
OPERATION_INVT_CURRENT_OFFSET_ERROR                     = 1014 # 0x7280
OPERATION_INVT_INCREMENTAL_SENSOR_ERROR                 = 1015 # 0x7305
OPERATION_INVT_ABSENCODER_MULT_ERROR                    = 1016 # 0x7306
OPERATION_INVT_ABSENCODER_CRC_ERROR                     = 1017 # 0x7307
OPERATION_INVT_OVER_SPEED_ERROR                         = 1018 # 0x7310
OPERATION_INVT_POSTIION_LIMIT_ERROR                     = 1019 # 0x7320
OPERATION_INVT_HALL_SENSOR_ERROR                        = 1020 # 0x7388
OPERATION_INVT_HALL_ANGLE_DETECTION_ERROR               = 1021 # 0x7389
OPERATION_INVT_ETHERCAT_COMM_ERROR                      = 1022 # 0x8100
OPERATION_INVT_HEARTBEAT_ERROR                          = 1023 # 0x8130
OPERATION_INVT_DIFFICULT_START_UP_ERROR                 = 1024 # 0x8312
OPERATION_INVT_POSITION_FOLLOWING_ERROR                 = 1025 # 0x8611
OPERATION_INVT_POSITION_REFERENCE_LIMIT_ERROR           = 1026 # 0x8612
OPERATION_INVT_STO_ERROR                                = 1027 # 0x8a88
OPERATION_INVT_EXTERNAL_ERROR                           = 1028 # 0x9000
OPERATION_INVT_JOINT_TORQUE_SENSOR_LIMIT                = 1029 # 0xFF01
OPERATION_INVT_OVERLOAD_ERROR                           = 1030 # 0xFF02
OPERATION_INVT_HW_OVER_CURRENT_SHORT_ERROR              = 1031 # 0xFF05
OPERATION_INVT_BRAKE_ERROR                              = 1032 # 0xFF06
OPERATION_INVT_POSITION_SENSOR_INDEX_ERROR              = 1033 # 0xFF07
OPERATION_INVT_POSITION_SENSOR_PULSE_ERROR              = 1034 # 0xFF08
OPERATION_INVT_CONTROLBOARD_TYPE_ERROR                  = 1035 # 0xFF09
OPERATION_INVT_HARMONICDRIVE_TYPE_ERROR                 = 1036 # 0xFF0A
OPERATION_INVT_EEPROM_ERROR                             = 1037 # 0xFF0B
OPERATION_INVT_PRECHARGE_ERROR                          = 1038 # 0xFF0C
OPERATION_INVT_JOINT_TORQUE_SENSOR_LIMIT_1              = 1039 # 0xFF0D
OPERATION_INVT_JOINT_TORQUE_SENSOR_LIMIT_2              = 1040 # 0xFF0E
OPERATION_INVT_JOINT_TORQUE_SENSOR_COMPARE              = 1041 # 0xFF0F
OPERATION_INVT_JOINT_TORQUE_SENSOR_RAPID_CHANGE         = 1042 # 0xFF10
OPERATION_INVT_JOINT_TORQUE_SENSOR_NOT_CHANGE_1         = 1043 # 0xFF11
OPERATION_INVT_JOINT_TORQUE_SENSOR_NOT_CHANGE_2         = 1044 # 0xFF12
OPERATION_INVT_JOINT_TORQUE_SENSOR_COMPARE_SAME         = 1045 # 0xFF13
OPERATION_INVT_CURRENT_UNBALANCE_ERROR                  = 1046 # 0xFF14
OPERATION_INVT_BOARD_OVER_TEMPERATURE_ERROR             = 1047 # 0xFF15
OPERATION_INVT_BOARD_UNDER_TEMPERATURE_ERROR            = 1048 # 0xFF16
OPERATION_INVT_INCREMENTAL_ENCODER_NO_COUNT             = 1049 # 0xFF17
OPERATION_INVT_INC_ABS_POS_DIFFERENCE_ERROR             = 1050 # 0xFF18
OPERATION_INVT_INC_ABS_SPEED_DIFFERENCE_ERROR           = 1051 # 0xFF19
OPERATION_INVT_JTS_SEQ_NUNBER_ERROR                     = 1052 # 0xFF1A
OPERATION_INVT_JTS_CRC32_ERROR                          = 1053 # 0xFF1B


#
# log code(eLOG_GROUP_SAFETY_CONTROLLER)
#

SAFETY_CONTROLLER_ERROR_NUMBER_START                    = 7000
OPERATION_RS422_SEQUENCE_NUMBER_ERROR                   = 7001
OPERATION_RS422_CRC32_CHECK_ERROR                       = 7002
OPERATION_RS422_UNKNOWN_COMMAND                         = 7003
OPERATION_RS422_COMMUNICATION_ERROR                     = 7004
OPERATION_ETHERCAT_COMMUNICATION_ERROR                  = 7005
OPERATION_CROSS_CHECK_BUFFER_OVERFLOW_BETWEEN_CPUS      = 7006
OPERATION_HANDGUIDE_IS_NOT_ALLOWED                      = 7007
OPERATION_MOTION_CROSS_CHECK_ERROR_BETWEEN_CPUS         = 7008
OPERATION_MOTION_MONITORING_PERIOD_OVERUN               = 7009
OPERATION_EMG_INPUT_CROSS_CHECK_ERROR_BETWEEN_CPUS      = 7010
OPERATION_EMG_INPUT_BEYOND_VOLTAGE_RANGE                = 7011
OPERATION_PRS_INPUT_CROSS_CHECK_ERROR_BETWEEN_CPUS      = 7012
OPERATION_PRS_INPUT_BEYOND_VOLTAGE_RANGE                = 7013
OPERATION_SAFETY_OUT_CROSS_CHECK_ERROR_BETWEEN_CPUS     = 7014
OPERATION_SAFETY_OUT_FEEDBACK_DIFFENENT                 = 7015
OPERATION_SAFETY_INPUT_CROSS_CHECK_ERROR_BETWEEN_CPUS   = 7016
OPERATION_CROSS_CHECK_ERROR_SAFETY_STATE                = 7017
                                                       
OPERATION_INVERTER_TEMP_BEYOND_RANGE                    = 7020
OPERATION_INVERTER_DATA_SEQUENCE_NUMBER_ERROR           = 7021
OPERATION_INVERTER_DATA_CRC32_CHECK_ERROR               = 7022
OPERATION_INVERTER_JTS_SEQUENCE_NUMBER_ERROR            = 7023
OPERATION_INVERTER_JTS_CRC32_CHECK_ERROR                = 7024
OPERATION_MOTIONCONTROLLER_SEQUENCE_NUMBER_ERROR        = 7025
OPERATION_MOTIONCONTROLLER_CRC32_CHECK_ERROR            = 7026
                                                       
OPERATION_UNDEFINED_INSTRUCTION_EXCEPTION               = 7030
OPERATION_PREPATCH_ABORT_EXCEPTION                      = 7031
OPERATION_DATA_ABORT_EXCEPTION                          = 7032
                                                       
OPERATION_EXCHANGE_DATA_SEQUENCE_ERROR                  = 7040
OPERATION_EXCHANGE_DATA_CRC32_ERROR                     = 7041
OPERATION_EXCHANGE_CLEARCMD_SEQUENCE_ERROR              = 7042
OPERATION_EXCHANGE_CLEARCMD_CRC32_ERROR                 = 7043
OPERATION_EXCHANGE_UPDATEDATA_SEQUENCE_ERROR            = 7044
OPERATION_EXCHANGE_UPDATEDATA_CRC32_ERROR               = 7045
OPERATION_EXCHANGE_INTREQ_ERROR                         = 7046
OPERATION_EXCHANGE_INTREQ_SEQUENCE_ERROR                = 7047
OPERATION_EXCHANGE_INTREQ_CRC32_ERROR                   = 7048
OPERATION_EXCHANGE_RS422_COMPARE_ERROR                  = 7049
                                                       
OPERATION_SAFETY_FUNCTION_STO_VIOLATION                 = 7050
OPERATION_SAFETY_FUNCTION_SBC_VIOLATION                 = 7051
OPERATION_SAFETY_FUNCTION_SS1_VIOLATION                 = 7052
OPERATION_SAFETY_FUNCTION_SS2_VIOLATION                 = 7053
OPERATION_SAFETY_FUNCTION_EMG_VIOLATION                 = 7054
OPERATION_SAFETY_FUNCTION_PRS_VIOLATION                 = 7055
OPERATION_SAFETY_FUNCTION_SOS_VIOLATION                 = 7056
OPERATION_SAFETY_FUNCTION_JOINT_SLP_VIOLATION           = 7057
OPERATION_SAFETY_FUNCTION_JOINT_SLS_VIOLATION           = 7058
OPERATION_SAFETY_FUNCTION_JOINT_SLT_VIOLATION           = 7059
OPERATION_SAFETY_FUNCTION_COLLISION_VIOLATION           = 7060
OPERATION_SAFETY_FUNCTION_TCP_SLP_VIOLATION             = 7061
OPERATION_SAFETY_FUNCTION_TCP_ORIENTATION_VIOLATION     = 7062
OPERATION_SAFETY_FUNCTION_TCP_SLS_VIOLATION             = 7063
OPERATION_SAFETY_FUNCTION_TCP_FORCE_VIOLATION           = 7064
OPERATION_SAFETY_FUNCTION_MOMENTUM_VIOLATION            = 7065
OPERATION_SAFETY_FUNCTION_POWER_VIOLATION               = 7066
OPERATION_SAFETY_FUNCTION_SELF_BODY_COLLISION           = 7067
OPERATION_SAFETY_FUNCTION_PROTECTIVE_ZONE_VIOLATION     = 7068
OPERATION_SAFETY_FUNCTION_ENCODER_CMP_VIOLATION         = 7069
OPERATION_SAFETY_FUNCTION_JTS_CMP_VIOLATION             = 7070
                                                       
OPERATION_INVERTER_1_2V_BEYOND_RANGE                    = 7080
OPERATION_INVERTER_3_3V_BEYOND_RANGE                    = 7081
OPERATION_INVERTER_5V_BEYOND_RANGE                      = 7082
OPERATION_INVERTER_17V_BEYOND_RANGE                     = 7083
OPERATION_INVERTER_24BV_BEYOND_RANGE                    = 7084
OPERATION_INVERTER_24CV_BEYOND_RANGE                    = 7085
OPERATION_INVERTER_48V_BEYOND_RANGE                     = 7086
                                                       
OPERATION_DETECT_S24V_DEVIATION                         = 7090
OPERATION_DETECT_S6V_DEVIATION                          = 7091
OPERATION_DETECT_S5V_DEVIATION                          = 7092
OPERATION_DETECT_S3V3_DEVIATION                         = 7093
OPERATION_DETECT_S1V2_DEVIATION                         = 7094
OPERATION_DETECT_NS12V_DEVIATION                        = 7095
OPERATION_DETECT_NS5V_DEVIATION                         = 7096
OPERATION_DETECT_NS3V3_DEVIATION                        = 7097
OPERATION_DETECT_NS2V5_DEVIATION                        = 7098
OPERATION_DETECT_STO_RELAY_MALFUNCTION                  = 7099
OPERATION_DETECT_EMF_DATA_BUS_MALFUNCTION               = 7100
OPERATION_DETECT_EMF_ADDRESS_BUS_MALFUNCTION            = 7101
OPERATION_DETECT_SPI_1__BUS__MALFUNCTION                = 7102
OPERATION_DETECT_SPI_2__BUS__MALFUNCTION                = 7103
OPERATION_DETECT_SPI_3__BUS__MALFUNCTION                = 7104
OPERATION_DETECT_CPU_TEMP_BEYOUN_RANGE                  = 7105
OPERATION_DETECT_BOARD_TEMP_BEYOUN_RANGE                = 7106
OPERATION_DETECT_CPU_CLOCK_MALFUNCTION                  = 7107
OPERATION_DETECT_MOTOR_SOLIDSTATE_CIRCUIT_BRAKE_MALFUNCTION = 7108
OPERATION_DETECT_BREAK_SOLIDSTATE_CIRCUIT_BRAKE_MALFUNCTION = 7109
OPERATION_DETECT_MOTOR48V_OVERVOLTAGE                   = 7110
OPERATION_DETECT_MOTOR48V_UNDERVOLTAGE                  = 7111
OPERATION_DETECT_BRAKE24V_OVERVOLTAGE                   = 7112
OPERATION_DETECT_BRAKE24V_UNDERVOLTAGE                  = 7113
OPERATION_DETECT_PRECHARGE_UNDERVOLTAGE                 = 7114
OPERATION_DETECT_MOTOR_OVERCURRENT                      = 7115
OPERATION_DETECT_BRAKE_OVERCURRENT                      = 7116
OPERATION_DETECT_LOGIC_OVERCURRENT                      = 7117
OPERATION_DETECT_IO_OVERCURRENT                         = 7118
OPERATION_DETECT_MOTOR_FET_FALUT                        = 7119
OPERATION_DETECT_BRAKE_FET_FALUT                        = 7120
OPERATION_DETECT_PRECHARGE_OVERCURRENT                  = 7121
OPERATION_DETECT_STO_RELAY_FAULT                        = 7122
OPERATION_DETECT_DISCREPANCY_MOTOR_CURRENT_A            = 7123
OPERATION_DETECT_DISCREPANCY_MOTOR_CURRENT_B            = 7124
OPERATION_DETECT_DISCREPANCY_BRAKE_VOLTAGE_A            = 7125
OPERATION_DETECT_DISCREPANCY_BRAKE_VOLTAGE_B            = 7126
OPERATION_DETECT_DISCREPANCY_MOTOR_VOLTAGE_A            = 7127
OPERATION_DETECT_DISCREPANCY_MOTOR_VOLTAGE_B            = 7128
OPERATION_DETECT_DISCREPANCY_BRAKE_CURRENT              = 7129
OPERATION_DETECT_WATCHDOG_ERROR                         = 7130
OPERATION_DETECT_CPU_UNDERTEMPERATURE                   = 7131
OPERATION_DETECT_BOARD_UNDERTEMPERATURE                 = 7132
OPERATION_DETECT_CPU_OVERTEMPERATURE                    = 7133
OPERATION_DETECT_BOARD_OVERTEMPERATURE                  = 7134
OPERATION_DETECT_CPU_SRAM_ECC_FAULT                     = 7135
OPERATION_DETECT_CPU_FLASH_ECC_FAULT                    = 7136
OPERATION_DETECT_CPU_FLASH_CRC_FAULT                    = 7137
OPERATION_DETECT_JTSSENSOR_FAULT                        = 7138
OPERATION_DETECT_MOTION_CAL_CMP_FAIL                    = 7139
OPERATION_DETECT_PRECHARGE_FET_FAULT                    = 7140
OPERATION_DETECT_IO5V_DEVIATION                         = 7141
                                                       
OPERATION_PARAM_INVALID_SYSTEM_STATE                    = 7170
OPERATION_PARAM_INVALID_CURRENT_TOOL                    = 7171
OPERATION_PARAM_INVALID_CURRENT_TCP                     = 7172
OPERATION_PARAM_INVALID_GENERAL_RANGE                   = 7173
OPERATION_PARAM_INVALID_JOINT_RANGE                     = 7174
OPERATION_PARAM_INVALID_SAFETY_FUNCTION                 = 7175
OPERATION_PARAM_INVALID_INSTALL_POSE                    = 7176
OPERATION_PARAM_INVALID_SAFETY_IO_SETTING               = 7177
OPERATION_PARAM_INVALID_VIRTUAL_WALL                    = 7178
OPERATION_PARAM_INVALID_SAFE_ZONE                       = 7179
OPERATION_PARAM_INVALID_SAFE_ZONE_ENABLE                = 7180
OPERATION_PARAM_INVALID_PROTECTED_ZONE                  = 7181
OPERATION_PARAM_INVALID_COLLISION_MUTE_ZONE             = 7182
OPERATION_PARAM_INVALID_TOOL_ORIENTATION_LIMIT_ZONE     = 7183
OPERATION_PARAM_INVALID_TOOL_SHAPE                      = 7184
                                                       
OPERATION_DETECT_CPU_RESERVED1                          = 7240
OPERATION_DETECT_CPU_RESERVED2                          = 7241
OPERATION_DETECT_CPU_RESERVED3                          = 7242
OPERATION_DETECT_CPU_RESERVED4                          = 7243
OPERATION_DETECT_CPU_RESERVED5                          = 7244
OPERATION_DETECT_CPU_RESERVED6                          = 7245
OPERATION_DETECT_CPU_RESERVED7                          = 7246
OPERATION_DETECT_CPU_RESERVED8                          = 7247
OPERATION_DETECT_CPU_RESERVED9                          = 7248
