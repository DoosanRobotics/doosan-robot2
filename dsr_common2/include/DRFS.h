/*    ========================================================================
    =                   Doosan Robot Framework Structure                      =
    =                   Copyright (c) Doosan Robotics.                        =   
    =_______________________________________________________________________  =
    = Title             : Doosan Robot Framwork Structure                     =
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
#include "DRFC.h"

#ifdef __cplusplus
#include <string>
#include <list>
using namespace std;
#endif

#pragma pack(1)

#define INFRACORE_PATCH
#define _FUNC_E_SERIES
#define _FUNC_AUTO_ACCELERATION

typedef struct _SYSTEM_VERSION
{
    /* smarttp version */
    char                        _szSmartTp[MAX_SYMBOL_SIZE];
    /* controller version */
    char                        _szController[MAX_SYMBOL_SIZE];
    /* interpreter version */
    char                        _szInterpreter[MAX_SYMBOL_SIZE];
    /* inverter version */
    char                        _szInverter[MAX_SYMBOL_SIZE];
    /* SafetyBoard version */
    char                        _szSafetyBoard[MAX_SYMBOL_SIZE];
    /* robot serial number */
    char                        _szRobotSerial[MAX_SYMBOL_SIZE];
    /* robot model number*/
    char                        _szRobotModel[MAX_SYMBOL_SIZE];
    /* jts board version */
    char                        _szJTSBoard[MAX_SYMBOL_SIZE];
    /* flange board version */
    char                        _szFlangeBoard[MAX_SYMBOL_SIZE];

} SYSTEM_VERSION, *LPSYSTEM_VERSION;

typedef struct _ROBOT_MONITORING_JOINT 
{
    /* Position Actual Value in INC */
    float                       _fActualPos[NUM_JOINT];
    /* Position Actual Value in ABS */
    float                       _fActualAbs[NUM_JOINT];
    /* Velocity Actual Value */
    float                       _fActualVel[NUM_JOINT];
    /* Joint Error */
    float                       _fActualErr[NUM_JOINT];
    /* Target Position */
    float                       _fTargetPos[NUM_JOINT];
    /* Target Velocity */
    float                       _fTargetVel[NUM_JOINT];

} ROBOT_MONITORING_JOINT, *LPROBOT_MONITORING_JOINT;

typedef struct _ROBOT_MONITORING_TASK 
{
    /* Position Actual Value(0: tool, 1: flange) */
    float                       _fActualPos[2][NUMBER_OF_JOINT]; 
    /* Velocity Actual Value */
    float                       _fActualVel[NUMBER_OF_JOINT];
    /* Task Error */
    float                       _fActualErr[NUMBER_OF_JOINT];
    /* Target Position */
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* Target Velocity */
    float                       _fTargetVel[NUMBER_OF_JOINT];
    /* Solution Space */
    unsigned char               _iSolutionSpace;
    /* Rotation Matrix */
    float                       _fRotationMatrix[3][3];

} ROBOT_MONITORING_TASK, *LPROBOT_MONITORING_TASK;

typedef struct _ROBOT_MONITORING_WORLD
{
    /* world to base relation */
    float                       _fActualW2B[NUMBER_OF_JOINT];
    /* Position Actual Value  (0:tool, 1:flange) */
    float                       _fActualPos[2][NUMBER_OF_JOINT]; 
    /* Velocity Actual Value */
    float                       _fActualVel[NUMBER_OF_JOINT];
    /* External Task Force/Torque */
    float                       _fActualETT[NUMBER_OF_JOINT];    
    /* Target Position */
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* Target Velocity */
    float                       _fTargetVel[NUMBER_OF_JOINT];   
    /* Rotation Matrix */
    float                       _fRotationMatrix[3][3];

} ROBOT_MONITORING_WORLD, *LPROBOT_MONITORING_WORLD;

typedef struct _ROBOT_MONITORING_USER
{
    /* actual user coord no */
    unsigned char               _iActualUCN;
    /* base: 0 world: 2 */
    unsigned char               _iParent;
    /* Position Actual Value  (0:tool, 1:flange) */
    float                       _fActualPos[2][NUMBER_OF_JOINT]; 
    /* Velocity Actual Value */
    float                       _fActualVel[NUMBER_OF_JOINT];
    /* External Task Force/Torque */
    float                       _fActualETT[NUMBER_OF_JOINT];       
    /* Target Position */
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* Target Velocity */
    float                       _fTargetVel[NUMBER_OF_JOINT];
    /* Rotation Matrix */
    float                       _fRotationMatrix[3][3];

} ROBOT_MONITORING_USER, *LPROBOT_MONITORING_USER;


typedef struct _ROBOT_MONITORING_TORQUE 
{
    /* Dynamics Torque */
    float                       _fDynamicTor[NUM_JOINT];
    /* Joint Torque Sensor Value */
    float                       _fActualJTS[NUM_JOINT];
    /* External Joint Torque */
    float                       _fActualEJT[NUM_JOINT];
    /* External Task Force/Torque */
    float                       _fActualETT[NUM_JOINT];

} ROBOT_MONITORING_TORQUE, *LPROBOT_MONITORING_TORQUE;

typedef struct _ROBOT_MONITORING_STATE 
{
    /* position control: 0, torque control: 1*/
    unsigned char               _iActualMode;
    /*joint space: 1, task space: 2*/
    unsigned char               _iActualSpace;

} ROBOT_MONITORING_STATE, *LPROBOT_MONITORING_STATE;

typedef struct _ROBOT_MONITORING_DATA
{
    ROBOT_MONITORING_STATE      _tState;
    /* joint */
    ROBOT_MONITORING_JOINT      _tJoint;
    /* task */
    ROBOT_MONITORING_TASK       _tTask;
    /* torque */
    ROBOT_MONITORING_TORQUE     _tTorque;  

} MONITORING_CONTROL, *LPMONITORING_CONTROL;

typedef struct _ROBOT_MONITORING_DATA_EX
{
    ROBOT_MONITORING_STATE      _tState;
    /* joint */
    ROBOT_MONITORING_JOINT      _tJoint;
    /* task */
    ROBOT_MONITORING_TASK       _tTask;
    /* torque */
    ROBOT_MONITORING_TORQUE     _tTorque;
    /* world */
    ROBOT_MONITORING_WORLD      _tWorld;
    /*user */
    ROBOT_MONITORING_USER       _tUser;
} ROBOT_MONITORING_DATA_EX, *LPROBOT_MONITORING_DATA_EX;

typedef ROBOT_MONITORING_DATA_EX
    MONITORING_CONTROL_EX, *LPMONITORING_CONTROL_EX;

typedef struct _MONITORING_MISC
{
    /* inner clock counter */
    double                       _dSyncTime;
    /* Flange Digtal Input data */
    unsigned char                _iActualDI[NUM_FLANGE_IO];
    /* Flange Digtal output data */
    unsigned char                _iActualDO[NUM_FLANGE_IO];
    /* brake state */
    unsigned char                _iActualBK[NUM_JOINT];
    /* robot button state */
    unsigned int                 _iActualBT[NUM_BUTTON];
    /* motor input current */
    float                        _fActualMC[NUM_JOINT];
    /* motro current temperature */
    float                        _fActualMT[NUM_JOINT];
} MONITORING_MISC, *LPMONITORING_MISC;

typedef struct _MONITORING_COCKPIT
{
    /* digital button state */
    unsigned char               _iActualBS[NUM_BUTTON_EX];

} MONITORING_COCKPIT, *LPMONITORING_COCKPIT;

typedef struct _MONITORING_DATA
{
    MONITORING_CONTROL          _tCtrl;
    /* misc. */
    MONITORING_MISC             _tMisc;

} MONITORING_DATA, *LPMONITORING_DATA;

typedef struct _FLANGE_VERSION
{
	unsigned char BoardNo;
	unsigned short PacketType;
	unsigned char res;
	unsigned char iFlangeHwVer;
}FLANGE_VERSION, *LPFLANGE_VERSION;

typedef struct _MONITORING_FLANGE_IO_CONFIG
{
    float               _iActualAI[MAX_FLANGE_AI];
    /* X1 RS485/Analog input pin mux */
    unsigned char               _iX1Rs485FAIPinMux;
    /* X2 RS485/Analog input pin mux */
    unsigned char               _iX2Rs485FAIPinMux;
    /* X1 Digital output pin BJT type */
    unsigned char               _iX1DOBjtType;
    /* X2 Digital output pin BJT type */
    unsigned char               _iX2DOBjtType;
    /* Voltage output level */
    unsigned char               _iVoutLevel;
    /* X1 Analog input 0 current or voltage mode */
    unsigned char               _iFAI0Mode;
    /* X1 Analog input 1 current or voltage mode */
    unsigned char               _iFAI1Mode;
    /* X2 Analog input 2 current or voltage mode */
    unsigned char               _iFAI2Mode;
    /* X2 Analog input 3 current or voltage mode */
    unsigned char               _iFAI3Mode;
    /* Serial X1 Baudrate */
    unsigned char               _szX1Baudrate[7]; // "0115200"(ASCII)
    /* Serial X1 Data Length */
    unsigned char               _szX1DataLength;  // 0: 1bit, 7: 8bit
    /* Serial X1 Parity */
    unsigned char               _szX1Parity;      // 0x00 : Non-parity, 0x01 : Odd, 0x02 : Even
    /* Serial X1 Stop Bit */
    unsigned char               _szX1StopBit;     // 0x01 : One stop bit, 0x02 : Two Stop bit
    /* Serial X2 Baudrate */
    unsigned char               _szX2Baudrate[7]; // "0115200"(ASCII)
    /* Serial X2 Data Length */
    unsigned char               _szX2DataLength;  // 0: 1bit, 7: 8bit
    /* Serial X2 Parity */
    unsigned char               _szX2Parity;      // 0x00 : Non-parity, 0x01 : Odd, 0x02 : Even
    /* Serial X2 Stop Bit */
    unsigned char               _szX2StopBit;     // 0x01 : One stop bit, 0x02 : Two Stop bit
    /* Servo Safety Mode */
    unsigned char               _iServoSafetyMode;     // 0x01 : One stop bit, 0x02 : Two Stop bit
    /* Interrupt Safety Mode */
    unsigned char               _iInterruptSafetyMode;     // 0x01 : One stop bit, 0x02 : Two Stop bit
} MONITORING_FLANGE_IO_CONFIG, *LPMONITORING_FLANGE_IO_CONFIG;

typedef struct _ROBOT_MONITORING_SENSOR
{
    /* Force Torque Sensor Value */
    float                       _fActualFTS[NUMBER_OF_JOINT];
    /* current sensor */
    float                       _fActualCS[NUMBER_OF_JOINT];
    /* accelerate speed sensro */
    float                       _fActualACS[3];

} ROBOT_MONITORING_SENSOR, *LPROBOT_MONITORING_SENSOR;

typedef struct _ROBOT_MONITORING_AMODEL
{
    /* sensor */
    ROBOT_MONITORING_SENSOR     _tSensor;
    /* singularity */
    float                       _fSingularity;
} ROBOT_MONITORING_AMODEL, *LPROBOT_MONITORING_AMODEL;

typedef ROBOT_MONITORING_AMODEL
    MONITORING_AMODEL, *LPMONITORING_AMODEL;

// typedef struct _MONITORING_FORCECONTROL
// {
// /* digital button state */
//     unsigned char               _iActualBS[NUMBER_OF_BUTTON];
//     /* current sensor */
//     float                       _fActualCS[NUMBER_OF_JOINT];
//     /* singularity */
//     float                       _fSingularity;
//     /* Tool Coord External Task Force/Torque */
//     float                       _fToolActualETT[NUMBER_OF_JOINT];
//     /* ForceControlMode */
//     unsigned char               _iForceControlMode[NUMBER_OF_JOINT]; // 0:Compliance, 1:Force Control, 2:None
//     /* reference coordinate */
//     unsigned char               _iReferenceCoord;  // 0:Base, 1:Tool, 2:World, 101~120:User Coord
// } MONITORING_FORCECONTROL, *LPMONITORING_FORCECONTROL;

typedef struct _MONITORING_FORCECONTROL
{
/* digital button state */
    unsigned char               _iActualBS[NUMBER_OF_BUTTON];
    /* current sensor */
    float                       _fActualCS[NUMBER_OF_JOINT];
    /* singularity */
    float                       _fSingularity;
    /* Tool Coord External Task Force/Torque */
    float                       _fToolActualETT[NUMBER_OF_JOINT];
    /* ForceControlMode */
    unsigned char               _iForceControlMode[NUMBER_OF_JOINT]; // 0:Compliance, 1:Force Control, 2:None
    /* reference coordinate */
    unsigned char               _iReferenceCoord;  // 0:Base, 1:Tool, 2:World, 101~120:User Coord
#ifdef _FUNC_AUTO_ACCELERATION
    /* auto acceleration mode */
    unsigned char               _iAutoAccMode;
#endif
#if defined (_FUNC_E_SERIES)
    /* reducer temperature */
    float                       _fActualHDT[NUMBER_OF_JOINT];
#endif
    /* Singluar Handling Mode */
    unsigned char               _iSingularHandlingMode;
    /* isMoving (DRCL) */
    unsigned char               _isMoving;
    unsigned char               reserved[2];
} MONITORING_FORCECONTROL, *LPMONITORING_FORCECONTROL;

typedef struct _MONITORING_DATA_EX
{
    MONITORING_CONTROL_EX       _tCtrl;
    /* misc. */
    MONITORING_MISC             _tMisc;

    union {
        MONITORING_FORCECONTROL       _tCtrlEx;
        unsigned char               _szReserved1[120];
    }                               _tMiscEx;

    union {
        MONITORING_AMODEL       _tAModel;
        unsigned char           _szReserved[64];
    }                           _tModel;

    union {
        MONITORING_FLANGE_IO_CONFIG _tConfig;
        unsigned char           _szReserved[56];
    }                           _tFlangeIo;
} MONITORING_DATA_EX, *LPMONITORING_DATA_EX;

typedef struct _READ_CTRLIO_INPUT
{
    /* Digtal Input data */
    unsigned char               _iActualDI[NUM_DIGITAL];
#if 0
    /*  Analog Input type */
    unsigned char               _iActualAT[NUM_ANALOG];
#endif
    /* Analog Input data */
    float                       _fActualAI[NUM_ANALOG];
    /* switch input data */
    unsigned char               _iActualSW[NUM_SWITCH];
    /* Safety Input data */
    unsigned char               _iActualSI[NUM_SAFETY_IN];
    /* Encorder Input data */
    unsigned char               _iActualEI[NUM_ENCORDER];
    /* Encorder raw data */
    unsigned int                _iAcutualED[NUM_ENCORDER];

} READ_CTRLIO_INPUT, *LPREAD_CTRLIO_INPUT;

typedef struct _READ_CTRLIO_OUTPUT
{
    /* Digital Output data */
    unsigned char               _iTargetDO[NUM_DIGITAL];
    /* Analog Output data */
    float                       _fTargetAO[NUM_ANALOG];

} READ_CTRLIO_OUTPUT, *LPREAD_CTRLIO_OUTPUT;



typedef struct _READ_ENCODER_INPUT
{
    /* Encorder strove signal */
    unsigned char               _iActualES[NUM_ENCORDER];
    /* Encorder raw data */
    unsigned int                _iActualED[NUM_ENCORDER];
    /* Encorder Reset signal */
    unsigned char               _iActualER[NUM_ENCORDER];

} READ_ENCODER_INPUT, *LPREAD_ENCODER_INPUT;



typedef struct _READ_PROCESS_INPUT
{
    /* Digtal Input data */
    unsigned char               _iActualDI[4];

} READ_PROCESS_INPUT, *LPREAD_PROCESS_INPUT;

typedef struct _MONITORING_CTRLIO
{
    /* input data */
    READ_CTRLIO_INPUT           _tInput;
    /* output data */
    READ_CTRLIO_OUTPUT          _tOutput;
} MONITORING_CTRLIO, *LPMONITORING_CTRLIO;


typedef struct _READ_CTRLIO_INPUT_EX
{
    /* Digtal Input data */
    unsigned char               _iActualDI[NUM_DIGITAL];
    /* Analog Input data */
    float                       _fActualAI[NUM_ANALOG];
    /* switch input data */
    unsigned char               _iActualSW[NUM_SWITCH];
    /* Safety Input data */
    unsigned char               _iActualSI[NUM_SAFETY_IN];
    /*  Analog Input type */
    unsigned char               _iActualAT[NUM_ANALOG];
    
} READ_CTRLIO_INPUT_EX, *LPREAD_CTRLIO_INPUT_EX;
typedef struct _READ_CTRLIO_OUTPUT_EX
{
    /* Digital Output data */
    unsigned char               _iTargetDO[NUM_DIGITAL];
    /* Analog Output data */
    float                       _fTargetAO[NUM_ANALOG];
    /*  Analog Output type */
    unsigned char               _iTargetAT[NUM_ANALOG];

} READ_CTRLIO_OUTPUT_EX, *LPREAD_CTRLIO_OUTPUT_EX;
typedef struct _MONITORING_CTRLIO_EX
{
    /* input data */
    READ_CTRLIO_INPUT_EX        _tInput;
    /* output data */
    READ_CTRLIO_OUTPUT_EX       _tOutput;
    /* input encoder data*/
    READ_ENCODER_INPUT          _tEncoder;
    /* reserved data */
    unsigned char               _szReserved[24];
} MONITORING_CTRLIO_EX, *LPMONITORING_CTRLIO_EX;

typedef struct _READ_CTRLIO_INPUT_EX2
{
    /* Digtal Input data */
    unsigned char               _iActualDI[NUM_DIGITAL_V3];
    /* Analog Input data */
    float                       _fActualAI[NUM_ANALOG];
    /* switch input data */
    unsigned char               _iActualSW[NUM_SWITCH];
    /* Safety Input data */
    unsigned char               _iActualSI[NUM_SAFETY_IN_V3];
    /*  Analog Input type */
    unsigned char               _iActualAT[NUM_ANALOG];

} READ_CTRLIO_INPUT_EX2, *LPREAD_CTRLIO_INPUT_EX2;

typedef struct _READ_CTRLIO_OUTPUT_EX2
{
    /* Digital Output data */
    unsigned char               _iTargetDO[NUM_DIGITAL_V3];
    /* Analog Output data */
    float                       _fTargetAO[NUM_ANALOG];
    /*  Analog Output type */
    unsigned char               _iTargetAT[NUM_ANALOG];

} READ_CTRLIO_OUTPUT_EX2, *LPREAD_CTRLIO_OUTPUT_EX2;

typedef struct _MONITORING_CTRLIO_EX2
{
    /* input data */
    READ_CTRLIO_INPUT_EX2        _tInput;
    /* output data */
    READ_CTRLIO_OUTPUT_EX2       _tOutput;
    /* input encoder data*/
    READ_ENCODER_INPUT          _tEncoder;
    /* reserved data */
    unsigned char               _szReserved[24];
} MONITORING_CTRLIO_EX2, *LPMONITORING_CTRLIO_EX2;

typedef struct _MODBUS_REGISTER
{
    /* modbus i/o name */
    char                        _szSymbol[MAX_SYMBOL_SIZE];
    /* modbus i/o  value */
    unsigned short              _iValue;

} MODBUS_REGISTER, *LPMODBUS_REGISTER;

typedef struct _MONITORING_MODBUS
{
    /* modbus i/o count */
    unsigned short              _iRegCount;
    /* modbus i/o values */
    MODBUS_REGISTER             _tRegister[MAX_MODBUS_TOTAL_REGISTERS];

} MONITORING_MODBUS, *LPMONITORING_MODBUS;

typedef struct _LOG_ALARM
{
    /* message level */
    unsigned char               _iLevel;
    /* group no */
    unsigned char               _iGroup;
    /* message no */
    unsigned int                _iIndex;
    /* message param */
    char                        _szParam[3][MAX_STRING_SIZE];
} LOG_ALARM, *LPLOG_ALARM;

typedef LOG_ALARM MONITORING_ALARM;

typedef struct _MESSAGE_PROGRESS
{
    /* current step */
    unsigned char               _iCurrentCount;
    /*  total step  */
    unsigned char               _iTotalCount;
} MESSAGE_PROGRESS, *LPMESSAGE_PROGRESS;

typedef struct _MESSAGE_POPUP
{
    /* message string */
    char                        _szText[MAX_STRING_SIZE];
    /*  Message: 0, Warning: 1, Alarm: 2 */
    unsigned char               _iLevel;
    /*  resuem and stop : 0, ok : 1 */
    unsigned char               _iBtnType;

} MESSAGE_POPUP, *LPMESSAGE_POPUP;

typedef MESSAGE_POPUP MONITORING_POPUP;

typedef struct _MESSAGE_INPUT
{
    /* message string */
    char                        _szText[MAX_STRING_SIZE];
    /*  int: 0, float: 1, string: 2 , bool: 3*/
    unsigned char               _iType;
} MESSAGE_INPUT, *LPMESSAGE_INPUT;

typedef MESSAGE_INPUT MONITORING_INPUT;

typedef struct _CONTROL_BRAKE
{
    /* joint: 0~5 , all joint: 6 */
    unsigned char               _iTargetAxs;
    /*  on: 1 off: 0 */
    unsigned char               _bValue;
} CONTROL_BRAKE, *LPCONTROL_BRAKE;

typedef struct _MOVE_POSB 
{
    /*  q               */
    float                       _fTargetPos[2][NUM_TASK];      
    /* blending motion type (line: 0, circle: 1) */
    unsigned char               _iBlendType;    
    /* blending radius  */
    float                       _fBlendRad;

} MOVE_POSB, *LPMOVE_POSB;

typedef struct _ROBOT_POSE
{
    /* current pose */
    float                       _fPosition[NUM_JOINT];

} ROBOT_POSE, *LPROBOT_POSE;

typedef struct _ROBOT_VEL
{
    /* current velocity */
    float                       _fVelocity[NUM_JOINT];
} ROBOT_VEL, *LPROBOT_VEL;

typedef struct _ROBOT_FORCE
{
    /* current force */
    float                       _fForce[NUM_JOINT];
} ROBOT_FORCE, *LPROBOT_FORCE;

typedef struct _ROBOT_TASK_POSE
{
    /* target pose */
    float                       _fTargetPos[NUM_TASK];
    /*  solution space: 0 ~ 7 */
    unsigned char               _iTargetSol;

} ROBOT_TASK_POSE, *LPROBOT_TASK_POSE;

typedef struct _USER_COORDINATE
{
    unsigned char _iReqId;
    unsigned char _iTargetRef;
    float _fTargetPos[NUM_TASK];
} USER_COORDINATE, *LPUSER_COORDINATE;

typedef struct _MEASURE_TOOL_RESPONSE
{
    /* mass(kg) */
    float                       _fWeight;
    /* center of mass */
    float                       _fXYZ[3];
    
} MEASURE_TOOL_RESPONSE, *LPMEASURE_TOOL_RESPONSE;

typedef struct _CONFIG_TCP
{
    /* target pose */
    float                       _fTargetPos[NUM_JOINT];

} CONFIG_TCP, *LPCONFIG_TCP;

typedef struct _CONFIG_TOOL
{
    /* mass(kg) */
    float                       _fWeight;
    /* center of mass */
    float                       _fXYZ[3];
    /* inertia */
    float                       _fInertia[NUMBER_OF_JOINT];    

} CONFIG_TOOL, *LPCONFIG_TOOL;

typedef struct _MEASURE_TCP_RESPONSE
{
    /* mesure pose  */
    CONFIG_TCP                  _tTCP;
    /* mesure error */
    float                       _fError;

} MEASURE_TCP_RESPONSE, *LPMEASURE_TCP_RESPONSE;

typedef struct _FLANGE_SERIAL_DATA
{
    unsigned char              _iCommand;       // 0 : OPEN, 1: CLOSE
                                                // 2 : SEND, 3: RECV
    union {
        struct {
            /* Baudrate */
            unsigned char               _szBaudrate[7]; // "0115200"(ASCII)
            /* Data Length */
            unsigned char               _szDataLength;  // 0: 1bit, 7: 8bit
            /* Parity */
            unsigned char               _szParity;      // 0x00 : Non-parity, 0x01 : Odd, 0x02 : Even
            /* Stop Bit */
            unsigned char               _szStopBit;     // 0x01 : One stop bit, 0x02 : Two Stop bit

        } _tConfig;

        //unsigned char                   _szConfigData[10];

        struct {
            unsigned short             _iLength;

            unsigned char              _szValue[MAX_SYMBOL_SIZE];

        } _tValue;
    } _tData;
} FLANGE_SERIAL_DATA, *LPFLANGE_SERIAL_DATA;

typedef struct _FLANGE_SER_RXD_INFO
{
    /* size of serial data */
    short                               _iSize;      //max 256bytes
                                                     /* raw serial data */
    unsigned char                       _cRxd[256];

} FLANGE_SER_RXD_INFO, *LPFLANGE_SER_RXD_INFO;

typedef struct _FLANGE_SER_RXD_INFO_EX
{
    /* size of serial data */
    short                               _iSize;      //max 256bytes
    /* raw serial data */
    unsigned char                       _cRxd[256];
    /* target port # */
    unsigned char                       _portNum;
} FLANGE_SER_RXD_INFO_EX, *LPFLANGE_SER_RXD_INFO_EX;

typedef struct _READ_FLANGE_SERIAL
{
    // check ready to read
    unsigned char               _bRecvFlag;    // 0: non-receive, 1: received
} READ_FLANGE_SERIAL, *LPREAD_FLANGE_SERIAL;

typedef struct _READ_FLANGE_SERIAL_EX
{
    // check ready to read
    unsigned char               _bRecvFlag[2];    // 0: non-receive, 1: received
} READ_FLANGE_SERIAL_EX, *LPREAD_FLANGE_SERIAL_EX;

typedef struct _INVERSE_KINEMATIC_RESPONSE
{
    /* target pose */
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* status */
    int                         _iStatus;
} INVERSE_KINEMATIC_RESPONSE, *LPINVERSE_KINEMATIC_RESPONSE;

typedef struct _RT_INPUT_DATA_LIST
{
    /* External Force Torque */
    float                       _fExternalForceTorque[NUM_JOINT];
    /* External Digital Input */
    unsigned short              _iExternalDI;
    /* External Digital Output */
    unsigned short              _iExternalDO;
    /* external analog input(6 channel) */
    float                       _fExternalAnalogInput[6];
    /* external analog output(6 channel) */
    float                       _fExternalAnalogOutput[6];
    /* Reserved */
    unsigned char               _iReserved[256];
} RT_INPUT_DATA_LIST, *LPRT_INPUT_DATA_LIST;

typedef struct _RT_OUTPUT_DATA_LIST
{
    /* timestamp at the data of data acquisition */
    double                      time_stamp;
    /* actual joint position from incremental encoder at motor side(used for control) [deg] */
    float                       actual_joint_position[NUMBER_OF_JOINT];
    /* actual joint position from absolute encoder at link side (used for exact link position) [deg] */
    float                       actual_joint_position_abs[NUMBER_OF_JOINT];
    /* actual joint velocity from incremental encoder at motor side [deg/s] */
    float                       actual_joint_velocity[NUMBER_OF_JOINT];
    /* actual joint velocity from absolute encoder at link side [deg/s] */
    float                       actual_joint_velocity_abs[NUMBER_OF_JOINT];
    /* actual robot tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] */
    float                       actual_tcp_position[NUM_TASK];
    /* actual robot tcp velocity w.r.t. base coordinates [mm, deg/s] */
    float                       actual_tcp_velocity[NUMBER_OF_TASK];
    /* actual robot flange position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] */
    float                       actual_flange_position[NUMBER_OF_TASK];
    /* robot flange velocity w.r.t. base coordinates [mm, deg/s] */
    float                       actual_flange_velocity[NUMBER_OF_TASK];
    /* actual motor torque applying gear ratio = gear_ratio * current2torque_constant * motor current [Nm] */
    float                       actual_motor_torque[NUMBER_OF_JOINT];
    /* estimated joint torque by robot controller [Nm] */
    float                       actual_joint_torque[NUMBER_OF_JOINT];
    /* calibrated joint torque sensor data [Nm] */
    float                       raw_joint_torque[NUMBER_OF_JOINT];
    /* calibrated force torque sensor data w.r.t. flange coordinates [N, Nm] */
    float                       raw_force_torque[NUMBER_OF_JOINT];
    /* estimated external joint torque [Nm] */
    float                       external_joint_torque[NUMBER_OF_JOINT];
    /* estimated tcp force w.r.t. base coordinates [N, Nm] */
    float                       external_tcp_force[NUMBER_OF_TASK];
    /* target joint position [deg] */
    float                       target_joint_position[NUMBER_OF_JOINT];
    /* target joint velocity [deg/s] */
    float                       target_joint_velocity[NUMBER_OF_JOINT];
    /* target joint acceleration [deg/s^2] */
    float                       target_joint_acceleration[NUMBER_OF_JOINT];
    /* target motor torque [Nm] */
    float                       target_motor_torque[NUMBER_OF_JOINT];
    /* target tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] */
    float                       target_tcp_position[NUMBER_OF_TASK];
    /* target tcp velocity w.r.t. base coordinates [mm, deg/s] */
    float                       target_tcp_velocity[NUMBER_OF_TASK];
    /* jacobian matrix=J(q) w.r.t. base coordinates */
    float                       jacobian_matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT];
    /* gravity torque=g(q) [Nm] */
    float                       gravity_torque[NUMBER_OF_JOINT];
    /* coriolis matrix=C(q,q_dot)  */
    float                       coriolis_matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT];
    /* mass matrix=M(q) */
    float                       mass_matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT];
    /* robot configuration */
    unsigned short              solution_space;
    /* minimum singular value */
    float                       singularity;
    /* current operation speed rate(1~100 %) */
    float                       operation_speed_rate;
    /* joint temperature(celsius) */
    float                       joint_temperature[NUMBER_OF_JOINT];
    /* controller digital input(16 channel) */
    unsigned short              controller_digital_input;
    /* controller digital output(16 channel) */
    unsigned short              controller_digital_output;
    /* controller analog input type(2 channel) */
    unsigned char               controller_analog_input_type[2];
    /* controller analog input(2 channel) */
    float                       controller_analog_input[2];
    /* controller analog output type(2 channel) */
    unsigned char               controller_analog_output_type[2];
    /* controller analog output(2 channel) */
    float                       controller_analog_output[2];
    /* flange digital input(A-Series: 2 channel, M/H-Series: 6 channel) */
    unsigned char               flange_digital_input;
    /* flange digital output(A-Series: 2 channel, M/H-Series: 6 channel) */
    unsigned char               flange_digital_output;
    /* flange analog input(A-Series: 2 channel, M/H-Series: 4 channel) */
    float                       flange_analog_input[4];
    /* strobe count(increased by 1 when detecting setting edge) */
    unsigned char               external_encoder_strobe_count[2];
    /* external encoder count */
    unsigned int                external_encoder_count[2];
    /* final goal joint position (reserved) */
    float                       goal_joint_position[NUMBER_OF_JOINT];
    /* final goal tcp position (reserved) */
    float                       goal_tcp_position[NUMBER_OF_TASK];
    /* ROBOT_MODE_MANUAL(0), ROBOT_MODE_AUTONOMOUS(1), SAFETY_MODE_RECOVERY(2)*/
    /* Refer to 'SAFETY_MODE' struct*/
    unsigned char               robot_mode;
    /* STATE_INITIALIZING(0), STATE_STANDBY(1), STATE_MOVING(2), STATE_SAFE_OFF(3), STATE_TEACHING(4), STATE_SAFE_STOP(5), STATE_EMERGENCY_STOP, STATE_HOMMING, STATE_RECOVERY, STATE_SAFE_STOP2, STATE_SAFE_OFF2, */
    /* Refer to 'OPERATION_STATE' struct in DRCF 'ROBOT_STATE' struct in DRFL.*/
    unsigned char               robot_state;
    /* position control mode, torque mode */
    /* Refer to 'SERVO_MODE' struct*/
    unsigned short              control_mode;
    /* Reserved */
    unsigned char               reserved[256];
    
} RT_OUTPUT_DATA_LIST, *LPRT_OUTPUT_DATA_LIST;
/*
typedef struct _UPDATE_SW_MODULE_RESPONSE{
    unsigned char               _bStatus;
    char                        _szModuleInform[2048];
} UPDATE_SW_MODULE_RESPONSE, *LPUPDATE_SW_MODULE_RESPONSE;
*/
typedef struct _JOINT_RANGE
{
    // max velocity
    float                       _fMaxVelocity[NUMBER_OF_JOINT];
    // max range
    float                       _fMaxRange[NUMBER_OF_JOINT];
    // min range
    float                       _fMinRange[NUMBER_OF_JOINT];
} JOINT_RANGE, *LPJOINT_RANGE;

typedef struct _CONFIG_JOINT_RANGE
{
    /* noraml mode */
    JOINT_RANGE                 _Normal;
    /* reduced mode */
    JOINT_RANGE                 _Reduced;

} CONFIG_JOINT_RANGE, *LPCONFIG_JOINT_RANGE;

typedef struct _GENERAL_RANGE
{
    // max force
    float                       _fMaxForce;
    // max power
    float                       _fMaxPower;
    // max speed
    float                       _fMaxSpeed;
    // max momenturm
    float                       _fMaxMomentum;
} GENERAL_RANGE, *LPGENERAL_RANGE;

typedef struct _CONFIG_GENERAL_RANGE
{
    /* noraml mode */
    GENERAL_RANGE               _Normal;
    /* reduced mode */
    GENERAL_RANGE               _Reduced;

} CONFIG_GENERAL_RANGE, *LPCONFIG_GENERAL_RANGE;

typedef struct _POINT_2D
{
    float                         _fXPos;
    float                         _fYPos;
} POINT_2D, *LPPOINT_2D;

typedef struct _POINT_3D
{
    float           _fXPos;
    float           _fYPos;
    float           _fZPos;
} POINT_3D, *LPPOINT_3D;

typedef struct _LINE
{
    POINT_2D            _tFromPoint;
    POINT_2D            _tToPoint;
} LINE, *LPLINE;

typedef union _CONFIG_SAFETY_FUNCTION
{
    /* SF05. Emergency Stop */
    /* SF06. Proctective Stop */
    /* SF07. StadnStill Monitoring */
    /* SF08. Joint Angle Monitoring */
    /* SF09. Joint Speed Monitoring */
    /* SF10. Joint Torque Monitoring*/
    /* SF11. Collisoin Detection */
    /* SF12. TCP Posiiont Monitoring */
    /* SF13. TCP Orientation Monitoring */
    /* SF14. TCP Speed Monitoring */
    /* SF15. TCP Force Monitoring */
    /* SF16. Momentum Monitoring */
    /* SF17. Power Mon. */

    struct {
        /* Standalone Workspace */
        unsigned char           _iStdWorkSpace:4;
        /* Collaborative Workspace */
        unsigned char           _iColWorkSpace:4;
    } _tStopCode[SAFETY_FUNC_LAST];

    unsigned char               _iStopCode[SAFETY_FUNC_LAST];

} CONFIG_SAFETY_FUNCTION, *LPCONFIG_SAFETY_FUNCTION;

typedef struct _CONFIG_INSTALL_POSE
{
    /* robot gradinet on ground  */
    float                       _fGradient;
    /* robot rotation angle */
    float                       _fRotation;

} CONFIG_INSTALL_POSE, *LPCONFIG_INSTALL_POSE;

typedef struct _CONFIG_SAFETY_IO
{
    /* Safety I/O */
    unsigned char               _iIO[TYPE_LAST][NUM_SAFETY];

} CONFIG_SAFETY_IO, *LPCONFIG_SAFETY_IO;

typedef struct _CONFIG_SAFETY_IO_EX
{
    /* Safety I/O */
    unsigned char               _iIO[TYPE_LAST][NUM_SAFETY * 2];
    ///* trigger level */
    //unsigned char               _bLevel[TYPE_LAST][NUM_SAFETY];

} CONFIG_SAFETY_IO_EX, *LPCONFIG_SAFETY_IO_EX;

typedef struct _CONFIG_SAFETY_IO_OP
{
    /* Safety I/O */
    unsigned char               _iIO[TYPE_LAST][16];
    /* TBSFT Input Option */
    unsigned char               _iTBI_Op;
    /* Reserved */
    unsigned char               _iReserved;
    /* Safety I/O Option */
    unsigned char               _iIO_Op[TYPE_LAST][16];

} CONFIG_SAFETY_IO_OP, *LPCONFIG_SAFETY_IO_OP;

typedef union _VIRTUAL_FENCE_OBJECT
{
    struct _CUBE {
        float                   _fXLoLimit;
        float                   _fXUpLimit;
        float                   _fYLoLimit;
        float                   _fYUpLimit;
        float                   _fZLoLimit;
        float                   _fZUpLimit;
    } _tCube;

    struct _POLYGON {
        unsigned char           _iLineCount;
        LINE                    _tLine[6];
        float                   _fZLoLimit;
        float                   _fZUpLimit;
    } _tPolygon;

    struct _CYLINDER {
        float                   _fRadius;
        float                   _fZLoLimit;
        float                   _fZUpLimit;
    } _tCylinder;

    unsigned char               _iBuffer[110];
} VIRTUAL_FENCE_OBJECT, *LPVIRTUAL_FENCE_OBJECT;

typedef struct _CONFIG_VIRTUAL_FENCE
{
    unsigned char               _iTargetRef;
    /* fence type - cube: 0, polygon: 1, cylinder: 2 */
    unsigned char               _iFenceType;
    /* fence objec */
    VIRTUAL_FENCE_OBJECT        _tFenceObject;
} CONFIG_VIRTUAL_FENCE, *LPCONFIG_VIRTUAL_FENCE;

typedef struct _CONFIG_SAFE_ZONE
{
    unsigned char               _iTargetRef;
    LINE                        _tLine[2];
    POINT_2D                    _tPoint[3];
} CONFIG_SAFE_ZONE, *LPCONFIG_SAFE_ZONE;

typedef struct _ENABLE_SAFE_ZONE
{
    /* region enable */
    unsigned char                 _iRegion[3];
} ENABLE_SAFE_ZONE, *LPENABLE_SAFE_ZONE;

typedef struct _SAFETY_OBJECT_SPHERE
{
    /* radius */
    float                   _fRadius;
    /* vertex: center */
    POINT_3D                _tTargetPos;
} SAFETY_OBJECT_SPHERE, *LPSAFETY_OBJECT_SPHERE;

typedef struct _SAFETY_OBJECT_CAPSULE
{
    /* radius */
    float                   _fRadius;
    /* vertex: low(0), high(1) */
    POINT_3D                _tTargetPos[2];

} SAFETY_OBJECT_CAPSULE, *LPSAFETY_OBJECT_CAPSULE;

typedef struct _SAFETY_OBJECT_CUBE
{
    /*  vertex: low(0), high(1)  */
    POINT_3D                _tTargetPos[2];

} SAFETY_OBJECT_CUBE, *LPSAFETY_OBJECT_CUBE;

typedef struct _SAFETY_OBJECT_OBB
{
    /* vetext: std(0), x-end(1), y-end(2), z-end(3) */
    POINT_3D                _tTargetPos[4];

} SAFETY_OBJECT_OBB, *LPSAFETY_OBJECT_OBB;

typedef struct _SAFETY_OBJECT_POLYPRISM
{
    unsigned char           _iPointCount;
    /* x,y coordinates of polygon vertices */
    POINT_2D                _tPoint[10];
    /* min z */
    float                   _fZLoLimit;
    /* max z */
    float                   _fZUpLimit;

} SAFETY_OBJECT_POLYPRISM, *LPSAFETY_OBJECT_POLYPRISM;

typedef union _SAFETY_OBJECT_DATA
{
    SAFETY_OBJECT_SPHERE        _tSphere;
    SAFETY_OBJECT_CAPSULE       _tCapsule;
    SAFETY_OBJECT_CUBE          _tCube;
    SAFETY_OBJECT_OBB           _tOBB;
    SAFETY_OBJECT_POLYPRISM     _tPolyPrism;
    unsigned char               _iBuffer[100];
} SAFETY_OBJECT_DATA, *LPSAFETY_OBJECT_DATA;

typedef struct _SAFETY_OBJECT
{
    unsigned char               _iTargetRef;
    /*
        geometry object type :
        0(Sphere), 1(Capsule), 2(Cube), 3(Oriented Box), 4(Polygon-Prism)
    */
    unsigned char                   _iObjectType;
    /* geometry object  data */
    SAFETY_OBJECT_DATA              _tObject;

} SAFETY_OBJECT, *LPSAFETY_OBJECT;

typedef struct _CONFIG_PROTECTED_ZONE
{
    /* validity of safety object: 0(invalid), 1(valid) */
    unsigned char                   _iValidity[10];
    /* safety object */
    SAFETY_OBJECT                   _tZone[10];

} CONFIG_PROTECTED_ZONE, *LPCONFIG_PROTECTED_ZONE;

typedef struct _CONFIG_COLLISION_MUTE_ZONE_PROPERTY
{
    char                            _szIdentifier[32];
    unsigned char                   _iOnOff;
#ifdef INFRACORE_PATCH        
    unsigned char                   _iSafetyIO;
#endif
    float                           _fSensitivity;
    SAFETY_OBJECT                   _tZone;
} CONFIG_COLLISION_MUTE_ZONE_PROPERTY, *LPCONFIG_COLLISION_MUTE_ZONE_PROPERTY;

typedef struct _CONFIG_COLLISION_MUTE_ZONE
{
    /* validity of safety object: 0(invalid), 1(valid) */
    unsigned char                   _iValidity[10];
    /* safety object */
    CONFIG_COLLISION_MUTE_ZONE_PROPERTY _tProperty[10];

} CONFIG_COLLISION_MUTE_ZONE, *LPCONFIG_COLLISION_MUTE_ZONE;

typedef struct _SAFETY_TOOL_ORIENTATION_LIMIT
{
    /* direction */
    POINT_3D                        _tTargetDir;
    /* angle (degree) */
    float                           _fTargetAng;

} SAFETY_TOOL_ORIENTATION_LIMIT, *LPSAFETY_TOOL_ORIENTATION_LIMIT;

typedef struct _CONFIG_TOOL_ORIENTATION_LIMIT_ZONE
{
    /* validity of safety object: 0(invalid), 1(valid) */
    unsigned char                   _iValidity[10];
    /* safety object */
    SAFETY_OBJECT                   _tZone[10];
    /* orientation limit for each zone */
    SAFETY_TOOL_ORIENTATION_LIMIT   _tLimit[10];

} CONFIG_TOOL_ORIENTATION_LIMIT_ZONE, *LPCONFIG_TOOL_ORIENTATION_LIMIT_ZONE;

typedef struct _CONFIG_NUDGE
{
    /* enalbe/disalbe */
    unsigned char               _bEnable;
    /* input force */
    float                       _fInputForce;
    /* delay time */
    float                       _fDelayTime;

} CONFIG_NUDGE, *LPCONFIG_NUDGE;

typedef struct _CONFIG_COCKPIT_EX
{
    /* disalbe: 0, enable: 1*/
    unsigned char               _bEnable;
    /* Direct Teach: 0, TCP-Z: 1, TCP-XY: 2, Orientation Only: 3, Positon Only: 4 */
    unsigned char               _iButton[2];
    /* disalbe: 0, enable: 1*/
    unsigned char               _bRecoveryTeach;

} CONFIG_COCKPIT_EX, *LPCONFIG_COCKPIT_EX;

typedef struct _CONFIG_IDLE_OFF
{
    /* function enable */
    unsigned char        _bFuncEnable;
    /* elapse time */
    float               _fElapseTime;

} CONFIG_IDLE_OFF, *LPCONFIG_IDLE_OFF;

typedef struct _WRITE_MODBUS_DATA
{
    /* symbol name */
    char                        _szSymbol[MAX_SYMBOL_SIZE];
    /* tcp address */
    char                        _szIpAddr[16];
    /* tcp port */
    unsigned short              _iPort;
    /* Slave ID*/
    int                         _iSlaveID;
    /* i/o type */
    unsigned char               _iRegType;
    /* register address */
    unsigned short              _iRegIndex;
    /* register value */
    unsigned short              _iRegValue;

} WRITE_MODBUS_DATA, *LPWRITE_MODBUS_DATA;

typedef WRITE_MODBUS_DATA
    WRITE_MODBUS_TCP_DATA, *LPWRITE_MODBUS_TCP_DATA;

typedef struct _WRITE_MODBUS_RTU_DATA
{
    /* symbol name */
    char                        _szSymbol[MAX_SYMBOL_SIZE];
    /* tty device */
    char                        _szttyPort[16];
    /* Slave ID*/
    int                         _iSlaveID;
    /* Baud Rate */
    int                         _iBaudRate;
    /* Byte size */
    int                         _iByteSize;
    /* Parity */
    char                        _szParity;
    /* Stop bit */
    int                         _iStopBit;
    /* i/o type */
    unsigned char               _iRegType;
    /* register address */
    unsigned short              _iRegIndex;
    /* register value */
    unsigned short              _iRegValue;

} WRITE_MODBUS_RTU_DATA, *LPWRITE_MODBUS_RTU_DATA;

typedef struct _MODBUS_DATA {

    /*0: TCP, 1: RTU */
    unsigned char _iType;

    union {
        /* modbus tcp */
        WRITE_MODBUS_TCP_DATA _tcp;
        /* modbus rtu */
        WRITE_MODBUS_RTU_DATA _rtu;
        /* data buffer */
        unsigned char _szBuffer[70];
    } _tData;

} MODBUS_DATA, * LPMODBUS_DATA;

typedef struct _MODBUS_DATA_LIST
{
    /* modbus count */
    unsigned short              _nCount;
    /* modbus data*/
    MODBUS_DATA                 _tRegister[MAX_MODBUS_TOTAL_REGISTERS];

} MODBUS_DATA_LIST, *LPMODBUS_DATA_LIST;

typedef struct _CONFIG_WORLD_COORDINATE
{
    /* world2base: 0, base2ref: 1, world2ref: 2 */
    unsigned char               _iType;
    /* target pose */
    float                       _fPosition[NUMBER_OF_JOINT];

} CONFIG_WORLD_COORDINATE, *LPCONFIG_WORLD_COORDINATE;

typedef struct _CONFIG_CONFIGURABLE_IO
{
    /* Safety I/O */
    unsigned char               _iIO[TYPE_LAST][NUM_DIGITAL];

} CONFIG_CONFIGURABLE_IO, *LPCONFIG_CONFIGURABLE_IO;

typedef struct _CONFIG_CONFIGURABLE_IO_EX
{
    /* Safety I/O */
    unsigned char               _iIO[TYPE_LAST][NUM_DIGITAL * 2];
    ///* trigger level */
    //unsigned char               _bLevel[TYPE_LAST][NUM_DIGITAL];

} CONFIG_CONFIGURABLE_IO_EX, *LPCONFIG_CONFIGURABLE_IO_EX;

typedef struct _CONFIG_TOOL_SHAPE
{
    /* validity of safety object: 0(invalid), 1(valid) */
    unsigned char                   _iValidity[5];
    /* safety object */
    SAFETY_OBJECT                   _tShape[5];

} CONFIG_TOOL_SHAPE, *LPCONFIG_TOOL_SHAPE;

typedef struct _CONFIG_TOOL_SYMBOL
{
    /* tool name */
    char                _szSymbol[MAX_SYMBOL_SIZE];
    /* tool data */
    CONFIG_TOOL         _tTool;

} CONFIG_TOOL_SYMBOL, *LPCONFIG_TOOL_SYMBOL;

typedef struct _CONFIG_TCP_SYMBOL
{
    /* tcp name */
    char                _szSymbol[MAX_SYMBOL_SIZE];
    /* tcp data */
    CONFIG_TCP          _tTCP;

} CONFIG_TCP_SYMBOL, *LPCONFIG_TCP_SYMBOL;

typedef struct _CONFIG_TOOL_LIST
{
    int                         _iToolCount;
    CONFIG_TOOL_SYMBOL          _tTooList[MAX_CONFIG_TOOL_SIZE];
} CONFIG_TOOL_LIST, *LPCONFIG_TOOL_LIST;

typedef struct _CONFIG_TOOL_SHAPE_SYMBOL
{
    char                _szSymbol[MAX_SYMBOL_SIZE];
    CONFIG_TOOL_SHAPE   _tToolShape;
} CONFIG_TOOL_SHAPE_SYMBOL, *LPCONFIG_TOOL_SHAPE_SYMBOL;

typedef struct _CONFIG_TCP_LIST
{
    int                         _iToolCount;
    CONFIG_TCP_SYMBOL           _tTooList[MAX_CONFIG_TCP_SIZE];
} CONFIG_TCP_LIST, *LPCONFIG_TCP_LIST;

typedef struct _CONFIG_TOOL_SHAPE_LIST
{
    int                         _iToolCount;
    CONFIG_TOOL_SHAPE_SYMBOL    _tTooList[MAX_CONFIG_TOOL_SIZE];
} CONFIG_TOOL_SHAPE_LIST, *LPCONFIG_TOOL_SHAPE_LIST;

typedef struct _CONFIG_USER_COORDINATE_EX
{
    /* base: 0, world: 2 */
    unsigned char               _iTargetRef;
    /* task position*/
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* unified id */
    unsigned char               _iUserID;

} CONFIG_USER_COORDINATE_EX, *LPCONFIG_USER_COORDINATE_EX;


typedef struct _CONFIG_PAYLOAD_EX
{
    /* mass(kg) */
    float                       _fWeight;
    /* center of mass */
    float                       _fXYZ[3];

    /* COG Reference */
    unsigned short              _iCogReference;
    /* Add Up*/
    unsigned short              _iAddUp;

    float                       _fStartTime;

    float                       _fTransitionTime;

    unsigned char               _iReserved[32];
} CONFIG_PAYLOAD_EX, *LPCONFIG_PAYLOAD_EX;

typedef struct _SYSTEM_TIME
{
    /* YYMMDD */
    char                        _szDate[8];
    /* HHMMSS */
    char                        _szTime[8];
} SYSTEM_TIME, *LPSYSTEM_TIME;

typedef struct _SYSTEM_IPADDRESS
{
#ifndef USE_ONE_IP
    /* internal use: 0, external link: 1*/
#else
    /* Disable: 0, Enable: 1*/
#endif
    unsigned char               _iUsage;
    /* dynamic ip: 0, static ip: 1*/
    unsigned char               _iIpType;
    /* ip address */
    char                        _szHotsIp[16];
    /* subnet mask address */
    char                        _szSubnet[16];
    /* defaltu gateway address */
    char                        _szGateway[16];
    /* primary/secondary dns address */
    char                        _szDNS[2][16];

} SYSTEM_IPADDRESS, *LPSYSTEM_IPADDRESS;

typedef struct _SYSTEM_POWER
{
    /* robot(inverter): 0, control pc: 1*/
    unsigned char               _iTarget;
    /* shutdown: 0, restart: 1 */
    unsigned char               _iPower;
} SYSTEM_POWER, *LPSYSTEM_POWER;

typedef struct _SYSTEM_CPUUSAGE
{
    /* process name */
    //char                        _processName[255];
    /* total cpu usage */
    float                        _iTotalUsage;
    /* specific process usage */
    float                        _iProcessUsage;
} SYSTEM_CPUUSAGE, *LPSYSTEM_CPUUSAGE;

typedef struct _SYSTEM_DISKSIZE
{
    /* total disk size */
    unsigned int                _iTotalDiskSize;
    /* used disk size */
    unsigned int                _iUsedDiskSize;
} SYSTEM_DISKSIZE, *LPSYSTEM_DISKSIZE;

typedef struct _JTS_PARAM_DATA
{
    /* jts offset */
    float                       _fOffset[NUMBER_OF_JOINT];
    /* jts scale */
    float                       _fScale[NUMBER_OF_JOINT];

} JTS_PARAM_DATA, *LPJTS_PARAM_DATA;

typedef JTS_PARAM_DATA
    CALIBRATE_JTS_RESPONSE, *LPCALIBRATE_JTS_RESPONSE;

typedef struct _INSTALL_SUB_SYSTEM {
    // process button
    unsigned char       _bProcessButton;
    // force torque sensoer
    unsigned char       _bFTS;
    // cockpit type: (0: 5button, 1: 6button)
    unsigned char       _bCockpit;
} INSTALL_SUB_SYSTEM, *LPINSTALL_SUB_SYSTEM;

typedef struct _FTS_PARAM_DATA
{
    /* offset */
    float                       _fOffset[NUMBER_OF_JOINT];

} FTS_PARAM_DATA, *LPFTS_PARAM_DATA;

typedef struct _SYSTEM_UPDATE_RESPONSE
{
    /* updating: 0, completed: 1 fail: 2*/
    unsigned char               _iProcess;
    /* inverter : non-update: 0, update: 1, sucess: 2, fail: 3*/
    /* safety controller(CPUA / B) : 0 - 100 */
    unsigned char               _iInverter[NUM_AXIS];

} SYSTEM_UPDATE_RESPONSE, *LPSYSTEM_UPDATE_RESPONSE;

typedef struct _KT_5G_CONFIG_PARAM
{
    /* 0: stop 1: start*/
    int                     _bEnable;
    /* server ip */
    char                    _szIpAddress[16];
    /*server port */
    int                     _nPort;
    /* deveice info*/
    char                    _szDeviceId[MAX_STRING_SIZE];
    char                    _szDevicePw[MAX_STRING_SIZE];
    char                    _szGatewayId[MAX_STRING_SIZE];
    /* monitoring period */
    float                  _fPeriod;
} KT_5G_CONFIG_PARAM, *LPKT_5G_CONFIG_PARAM;

typedef struct _VECTOR3D
{
    /* normal vector */
    float                       _fTargetPos[3];

} VECTOR3D, *LPVECTOR3D;

typedef struct _POSITION
{
    /* target pose */
    float                       _fTargetPos[NUMBER_OF_JOINT];

} POSITION, *LPPOSITION;

typedef POSITION VECTOR6D, *LPVECTOR6D;

typedef VECTOR3D NORMAL_VECTOR_RESPONSE, *LPNORMAL_VECTOR_RESPONSE;

typedef struct _NORMAL_VECTOR
{
    /* task position for normal vector */
    float                       _fTargetPos[3][NUMBER_OF_JOINT];
} NORMAL_VECTOR, *LPNORMAL_VECTOR;

typedef FTS_PARAM_DATA
    CALIBRATE_FTS_RESPONSE, *LPCALIBRATE_FTS_RESPONSE;

typedef struct _GPIO_PORT
{
    /* port index */
    unsigned char               _iIndex;
    /* port value */
    float                       _fValue;
} GPIO_PORT, *LPGPIO_PORT;

typedef struct _GPIO_SERIAL_BURST
{
    /* controlbox: 0, arm: 1 */
    unsigned char               _iLocation;
    /* i/o port count */
    unsigned short              _iCount;
    /* i/o port data */
    GPIO_PORT                   _tPort[MAX_DIGITAL_BURST_SIZE];

} WRITE_SERIAL_BURST, *LPWRITE_SERIAL_BURST;

typedef WRITE_SERIAL_BURST
    GPIO_SETOUTPUT_BURST, *LPGPIO_SETOUTPUT_BURST;

typedef struct _MODBUS_REGISTER_MONITORING
{
    /* modbus i/o name */
    char                        _szSymbol[MAX_SYMBOL_SIZE];
    /* modbus i/o  value */
    unsigned short              _iRegValue;

} MODBUS_REGISTER_MONITORING, *LPMODBUS_REGISTER_MONITORING;

typedef struct _WRITE_MODBUS_BURST
{
    /* modbus i/o count */
    unsigned short              _iCount;
    /* modbus i/o values */
    MODBUS_REGISTER_MONITORING  _tRegister[MAX_MODBUS_BURST_SIZE];

} WRITE_MODBUS_BURST, *LPWRITE_MODBUS_BURST;

typedef WRITE_MODBUS_BURST
    MODBUS_REGISTER_BURST, *LPMODBUS_REGISTER_BURST;

typedef struct _MODBUS_MULTI_REGISTER
{
    /* modbus i/o name */
    char                        _szSymbol[MAX_SYMBOL_SIZE];
    /* register count */
    unsigned char               _iRegCount;
    /* modbus i/o  value */
    unsigned short              _iRegValue[MAX_MODBUS_REGISTER_PER_DEVICE];
    /* register start address */
    unsigned short              _iRegIndex;
    /* Slave ID*/
    unsigned int                _iSlaveID;

} MODBUS_MULTI_REGISTER, *LPMODBUS_MULTI_REGISTER;

typedef struct _UPDATE_MODBUS_MULTI_REGISTER
{
    /* modbus i/o name */
    char                        _szSymbol[MAX_SYMBOL_SIZE];
    /* register count */
    unsigned char               _iRegCount;
    /* modbus i/o  value */
    unsigned short              _iRegValue[MAX_MODBUS_REGISTER_PER_DEVICE];
    /* register start address */
    //unsigned short              _iRegIndex;
    /* Slave ID*/
    //unsigned int                _iSlaveID;

} UPDATE_MODBUS_MULTI_REGISTER, *LPUPDATE_MODBUS_MULTI_REGISTER;

typedef struct _LOCAL_ZONE_PROPERTY_JOINT_RANGE
{
    /* 0(No Override): 1(Override global property) */
    unsigned char               _iOverride[6];
    /* 0(No Override): 1(Override global property) */
    /*unsigned char               _iOverrideReduce[6];*/
    /* Override Value */
    float                       _fMinRange[6];
    float                       _fMaxRange[6];
} LOCAL_ZONE_PROPERTY_JOINT_RANGE, *LPLOCAL_ZONE_PROPERTY_JOINT_RANGE;

typedef struct _SAFETY_ZONE_PROPERTY_SPACE_LIMIT
{
    /* Inspection Type: 0(body), 1(tcp) */
    unsigned char               _iInspectionType;
    /* Valid Space: 0(inside), 1(outside) */
    /*unsigned char               _iValidSpace;*/
    /* Override Joint Range */
    LOCAL_ZONE_PROPERTY_JOINT_RANGE _tJointRangeOverride;
    /* Dynamic Zone Enable Option: 0(Not used), 1~8(safety input channel) */
    unsigned char               _iDynamicZoneEnable;	
    /* Inside Zone Dectection Option: 0(Not used), 1~8(safety input channel) */
    unsigned char               _iInsideZoneDectection;
} SAFETY_ZONE_PROPERTY_SPACE_LIMIT, *LPSAFETY_ZONE_PROPERTY_SPACE_LIMIT;

typedef struct _LOCAL_ZONE_PROPERTY_JOINT_SPEED
{
    /* 0(No Override): 1(Override global property) */
    unsigned char               _iOverride[6];
    /* 0(No Override): 1(Override global property) */
    /*unsigned char               _iOverrideReduce[6];*/
    /* Override Value */
    float                       _fSpeed[6];
} LOCAL_ZONE_PROPERTY_JOINT_SPEED, *LPLOCAL_ZONE_PROPERTY_JOINT_SPEED;

typedef struct _LOCAL_ZONE_PROPERTY_TCP_FORCE
{
    /* 0(No Override): 1(Override global property) */
    unsigned char               _iOverride;
    /* 0(No Override): 1(Override global property) */
    /*unsigned char               _iOverrideReduce;*/
    /* Override Value */
    float                       _fForce;
} LOCAL_ZONE_PROPERTY_TCP_FORCE, *LPLOCAL_ZONE_PROPERTY_TCP_FORCE;

typedef struct _LOCAL_ZONE_PROPERTY_TCP_POWER
{
    /* 0(No Override): 1(Override global property) */
    unsigned char               _iOverride;
    /* 0(No Override): 1(Override global property) */
    /*unsigned char               _iOverrideReduce;*/
    /* Override Value */
    float                       _fPower;
} LOCAL_ZONE_PROPERTY_TCP_POWER, *LPLOCAL_ZONE_PROPERTY_TCP_POWER;

typedef struct _LOCAL_ZONE_PROPERTY_TCP_SPEED
{
    /* 0(No Override): 1(Override global property) */
    unsigned char               _iOverride;
    /* 0(No Override): 1(Override global property) */
    /*unsigned char               _iOverrideReduce;*/
    /* Override Value */
    float                       _fSpeed;
} LOCAL_ZONE_PROPERTY_TCP_SPEED, *LPLOCAL_ZONE_PROPERTY_TCP_SPEED;

typedef struct _LOCAL_ZONE_PROPERTY_TCP_MOMENTUM
{
    /* 0(No Override): 1(Override global property) */
    unsigned char               _iOverride;
    /* 0(No Override): 1(Override global property) */
    /*unsigned char               _iOverrideReduce;*/
    /* Override Value */
    float                       _fMomentum;
} LOCAL_ZONE_PROPERTY_TCP_MOMENTUM, *LPLOCAL_ZONE_PROPERTY_TCP_MOMENTUM;

typedef struct _LOCAL_ZONE_PROPERTY_COLLISION
{
    /* 0(No Override): 1(Override global property) */
    unsigned char               _iOverride;
    /* 0(No Inspection): 1(Inspect, Apply local sensitivity) */
    /*unsigned char               _iCollisionInspection;*/
    /* Override Value */
    float                       _fSensitivity;
} LOCAL_ZONE_PROPERTY_COLLISION, *LPLOCAL_ZONE_PROPERTY_COLLISION;

typedef struct _LOCAL_ZONE_PROPERTY_SPEED_RATE
{
    /* 0(No Override): 1(Override global property) */
    unsigned char               _iOverride;
    /* Override Value */
    float                       _fSpeedRate;
} LOCAL_ZONE_PROPERTY_SPEED_RATE, *LPLOCAL_ZONE_PROPERTY_SPEED_RATE;

typedef struct _LOCAL_ZONE_PROPERTY_SPEED_REDUCTION
{
    /* 0(No Override): 1(Override global property) */
    unsigned char               _iOverride;    
    /* Override Value */
    float                       _fReductionRate;
} LOCAL_ZONE_PROPERTY_SPEED_REDUCTION, *LPLOCAL_ZONE_PROPERTY_SPEED_REDUCTION;

typedef struct _LOCAL_ZONE_PROPERTY_COLLISION_STOPMODE
{
    /* 0(No Override): 1(Override global property) */
    unsigned char               _iOverride;    
    /* Override Value: 0(STO), 2(SS1), 3(SS2), 4(RS1) */
    unsigned char               _iStopMode;
} LOCAL_ZONE_PROPERTY_COLLISION_STOPMODE, *LPLOCAL_ZONE_PROPERTY_COLLISION_STOPMODE;

typedef struct _LOCAL_ZONE_PROPERTY_TCPSLF_STOPMODE
{
    /* 0(No Override): 1(Override global property) */
    unsigned char               _iOverride;    
    /* Override Value: 0(STO), 2(SS1), 3(SS2), 4(RS1) */
    unsigned char               _iStopMode;
} LOCAL_ZONE_PROPERTY_TCPSLF_STOPMODE, *LPLOCAL_ZONE_PROPERTY_TCPSLF_STOPMODE;

typedef struct _LOCAL_ZONE_PROPERTY_TOOL_ORIENTATION
{
    /* 0(No Override): 1(Override global property) */
    unsigned char               _iOverride;    
    /* Override Value */
    float                       _fDirection[3];
    float                       _fAngle;
} LOCAL_ZONE_PROPERTY_TOOL_ORIENTATION, *LPLOCAL_ZONE_PROPERTY_TOOL_ORIENTATION;

typedef struct _SAFETY_ZONE_PROPERTY_LOCAL_ZONE
{    
    /* Override Joint Range */
    LOCAL_ZONE_PROPERTY_JOINT_RANGE _tJointRangeOverride;
    /* Override Joint Speed */
    LOCAL_ZONE_PROPERTY_JOINT_SPEED _tJointSpeedOverride;
    /* Override Tcp Force */
    LOCAL_ZONE_PROPERTY_TCP_FORCE _tTcpForceOverride;
    /* Override Tcp Power */
    LOCAL_ZONE_PROPERTY_TCP_POWER _tTcpPowerOverride;
    /* Override Tcp Speed */
    LOCAL_ZONE_PROPERTY_TCP_SPEED _tTcpSpeedOverride;
    /* Override Tcp Momentum */
    LOCAL_ZONE_PROPERTY_TCP_MOMENTUM _tTcpMomentumOverride;
    /* Override Collision */
    LOCAL_ZONE_PROPERTY_COLLISION _tCollisionOverride;
    /* Override Speed Rate */
    LOCAL_ZONE_PROPERTY_SPEED_RATE _tSpeedRate;
    /* Override Collision Stop mode */
    LOCAL_ZONE_PROPERTY_COLLISION_STOPMODE _tCollisionViolationStopmodeOverride;
    /* Override Tcp-SLF Stop mode */
    LOCAL_ZONE_PROPERTY_TCPSLF_STOPMODE _tForceViolationStopmodeOverride;
    /* Override Tool Orientation Limit */
    LOCAL_ZONE_PROPERTY_TOOL_ORIENTATION _tToolOrientationLimitOverride;
    /* Dynamic Zone Enable Option: 0(Not used), 1~8(safety input channel) */
    unsigned char               _iDynamicZoneEnable;
    /* Led: 0(Not used), 1(Green), 2(Yellow) */
    unsigned char               _iLedOverride;
    /* Nudge Enable: 0(Disable), 1(Enable) */
    unsigned char               _iNundgeEanble;
    /* Allow less safe work: 0(not allowed), 1(allowed) */
    unsigned char               _iAllowLessSafeWork;
	/* 0(No Override: Consider Reduce Mode): 1(Override: Ignore Reduce Mode) */
    unsigned char               _iOverrideReduce;
    /* Inside Zone Dectection Option: 0(Not used), 1~8(safety input channel) */
    unsigned char               _iInsideZoneDectection;
    /* collaborative zone option: 0(Not used), 1(Used) */
    unsigned char               _bCollaborativeZone;

	/* _tReservedBuffer[0] as 'is collaborativew workspace' */
	/* _tReservedBuffer[1] as 'is collision mute zone' */
	/* _tReservedBuffer[2] as 'is tool orientation limit zone' */
	/* _tReservedBuffer[3] as 'is clamping prevention zone'  */
	unsigned char				_tReservedBuffer[58];

} SAFETY_ZONE_PROPERTY_LOCAL_ZONE, *LPSAFETY_ZONE_PROPERTY_LOCAL_ZONE;

typedef union _SAFETY_ZONE_PROPERTY_DATA
{
    SAFETY_ZONE_PROPERTY_SPACE_LIMIT    _tSpaceLimitZone;
    SAFETY_ZONE_PROPERTY_LOCAL_ZONE     _tLocalZone;    
    unsigned char                       _iBuffer[200];
} SAFETY_ZONE_PROPERTY_DATA, *LPSAFETY_ZONE_PROPERTY_DATA;

typedef struct _CONFIG_USER_COORDINATE
{
	float _fTargetPos[NUM_TASK];
	unsigned char _iReqId;
} CONFIG_USER_COORDINATE, *LPCONFIG_USER_COORDINATE;


typedef struct _SAFETY_ZONE_SHAPE_SPHERE
{   
    /* center */
    POINT_3D                _tCenter;
    /* radius */
    float                   _fRadius;
} SAFETY_ZONE_SHAPE_SPHERE, *LPSAFETY_ZONE_SHAPE_SPHERE;


typedef struct _SAFETY_ZONE_SHAPE_CYLINDER
{
    /* center */
    POINT_2D                _tCenter;
    /* radius */
    float                   _fRadius;    
    float                   _fZLoLimit;
    float                   _fZUpLimit;

} SAFETY_ZONE_SHAPE_CYLINDER, *LPSAFETY_ZONE_SHAPE_CYLINDER;

typedef struct _SAFETY_ZONE_SHAPE_CUBOID
{
    float                   _fXLoLimit;
    float                   _fYLoLimit;
    float                   _fZLoLimit;
    float                   _fXUpLimit;
    float                   _fYUpLimit;
    float                   _fZUpLimit;

} SAFETY_ZONE_SHAPE_CUBOID, *LPSAFETY_ZONE_SHAPE_CUBOID;

typedef struct _SAFETY_ZONE_SHAPE_TILTED_CUBOID
{
    POINT_3D                _tOrigin;
    POINT_3D                _tUAxisEnd;
    POINT_3D                _tVAxisEnd;
    POINT_3D                _tWAxisEnd;
} SAFETY_ZONE_SHAPE_TILTED_CUBOID, *LPSAFETY_ZONE_SHAPE_TILTED_CUBOID;

typedef LINE LINE_2D;

typedef struct _SAFETY_ZONE_SHAPE_MULTI_PLANE
{
    unsigned char           _iValidPlane[6];    
    LINE_2D                 _tPlane[6];
    float                   _fZLoLimit;
    float                   _fZUpLimit;
    POINT_2D                _tSpacePoint;
} SAFETY_ZONE_SHAPE_MULTI_PLANE, *LPSAFETY_ZONE_SHAPE_MULTI_PLANE;

typedef struct _SAFETY_ZONE_SHAPE_CAPSULE
{
    POINT_3D                _tCenter1;
    POINT_3D                _tCenter2;
    /* radius */
    float                   _fRadius;
} SAFETY_ZONE_SHAPE_CAPSULE, *LPSAFETY_ZONE_SHAPE_CAPSULE;

typedef union _SAFETY_ZONE_SHAPE_DATA
{
    SAFETY_ZONE_SHAPE_SPHERE            _tSphere;
    SAFETY_ZONE_SHAPE_CYLINDER          _tCylinder;
    SAFETY_ZONE_SHAPE_CUBOID            _tCuboid;
    SAFETY_ZONE_SHAPE_TILTED_CUBOID     _tOBB;
    SAFETY_ZONE_SHAPE_MULTI_PLANE       _tMultiPlane;
    SAFETY_ZONE_SHAPE_CAPSULE           _tCapsule;
    unsigned char                       _iBuffer[120];
} SAFETY_ZONE_SHAPE_DATA, *LPSAFETY_ZONE_SHAPE_DATA;

typedef struct _SAFETY_ZONE_SHAPE
{
    /* coordinated: 0(base), 2(world) */
    unsigned char               _iCoordinate;
    /* geometry object type :  0(Sphere), 1(Cylinder), 2(Cuboid), 3(Tilted Cuboid), 4(Multi-Plane), 5(Capsule) */
    unsigned char               _iShapeType;
    /* geometry object  data */
    SAFETY_ZONE_SHAPE_DATA      _tShapeData;
    /* Positive value: expand shape, Negative value: shrink value */
    float                       _fMargin;
    /* Valid space: 0(inside), 1(outside) */
    unsigned char               _iValidSpace;
    /* Reserved for future use */
    unsigned char               _iReserved[13];
} SAFETY_ZONE_SHAPE, *LPSAFETY_ZONE_SHAPE;

typedef struct _CONFIG_ADD_SAFETY_ZONE
{
    /* Zone Identifier: uuid */
    char                            _szIdentifier[32];
    /* Zone alias : nullable */
    char                            _szAlias[32];
    /* Zone type: 0(space limit), 1(local zone) */
    unsigned char                   _iZoneType;
    /* Zone property */
    SAFETY_ZONE_PROPERTY_DATA       _tZoneProperty;
    /* Zone shape */
    SAFETY_ZONE_SHAPE               _tShape;
} CONFIG_ADD_SAFETY_ZONE, *LPCONFIG_ADD_SAFETY_ZONE;

typedef CONFIG_ADD_SAFETY_ZONE CONFIG_SAFETY_ZONE;


typedef struct _CONFIG_DELETE_SAFETY_ZONE
{
    char                            _szIdentifier[32];
} CONFIG_DELETE_SAFETY_ZONE, *LPCONFIG_DELETE_SAFETY_ZONE;

typedef struct _CONFIG_ENCODER_POLARITY {
    /* encorder channel: 0 ~ 1*/
    unsigned char _iChannel;
    /* encoder polarity : 0 ~1*/
    unsigned char _iPolarity[ENCORDER_POLARITY_LAST];
} CONFIG_ENCODER_POLARITY, *LPCONFIG_ENCODER_POLARITY;

typedef struct _CONFIG_ENCODER_MODE {
    /* encorder channel: 0~ 1*/
    unsigned char _iChannel;
    /* AB polariity mode */
    // 0: A(Not Used),B(Not Used)
    // 1: A(QEP),B(QEP)
    // 2: A(Count), B(Direction)
    // 3: A(Up Count), B(Not Used)
    // 4: A(Down Count), B(Not Used)
    unsigned char _iABMode;
    /* Z polarity mode*/
    // 0: Not Used
    // 1: Count Cumulative compensation
    // 2: Count Clear
    unsigned char _iZMode;
    /* S polarity mode*/
    // 0: Not Used
    unsigned char _iSMode;
    /* 0: forward 1: inversed  */
    unsigned char _iInvMode;
    /* 'A' pulse count per 'Z': 0~ 100000 */
    unsigned int  _nPulseAZ;

} CONFIG_ENCODER_MODE, *LPCONFIG_ENCODER_MODE;

typedef struct _CONFIG_IO_FUNC
{
    /* port number: not used: -1, used: 0~15 */
    char                        _iPort;
    /* low: 0, high: 1*/
    char                        _bLevel;

} CONFIG_IO_FUNC, *LPCONFIG_IO_FUNC;

typedef struct _CONFIG_REMOTE_CONTROL
{
    /* use or noet*/
    unsigned char               _bEnable;
    /* input Fucnc*/
    CONFIG_IO_FUNC              _tFunc[TYPE_LAST][NUM_REMOTE_CONTROL];

} CONFIG_REMOTE_CONTROL, *LPCONFIG_REMOTE_CONTROL;

typedef struct _PROGRAM_SYNTAX_CHECK
{
    /*  length               */
    unsigned int                _iTextLength;
    /*  variable length */
    //char*                   _lpszTextString;
} PROGRAM_SYNTAX_CHECK, *LPPROGRAM_SYNTAX_CHECK;

typedef struct _CONVEYOR_BASIC
{
    /* linear:0 , circular: 1*/
    unsigned char       _iType;
    /* encorder input */
    struct {
        /* index: 0 ~ 1 */
        unsigned char   _iEncoderChannel;
        /* index: 0 ~ 1 */
        unsigned char   _iTriggerChannel;
        /* falling: 0, rising: 1*/
        unsigned char   _iTriggerEdgeType;
        /* mute time (sec) */
        float            _fTriggerMuteTime;
    } ExtEncoderInput;
    /* digital outpout to conveyor */
    struct {
        /* not used : -1, used: 0 ~ 15 */
        char           _iChannel;
        /* value : 0 ~ 1*/
        unsigned char   _iValue;
    } DigitalOutput[2];

} CONVEYOR_BASIC, *LPCONVEYOR_BASIC;

typedef struct _CONVEYOR_COORD_EX
{
    int                _iDistance2Count;
    /* converyor coordination */
    POSITION            _tPosConCoord;
    /*Base : 0, World: 2 */
    unsigned char       _iTargetRef;
} CONVEYOR_COORD_EX, *LPCONVEYOR_COORD_EX;

typedef struct _MEASURE_CONVEYOR_COORD_RESPONSE
{
    /* count per distance */
    int                 _iDistance2Count;
    /* converyor coordination */
    POSITION            _tPosCoord;

} MEASURE_CONVEYOR_COORD_RESPONSE, *LPMEASURE_CONVEYOR_COORD_RESPONSE;

typedef struct _MEASURE_CONVEYOR_DISTANCE_RESPONSE
{
    /* conveyor speed */
    float               _fSpeed;
    /* minimum distance */
    float               _fMinDistance;
    /* watch window distance */
    float               _fWatchWinDist;
    /* maximum distance */
    float               _fMaxDistance;
    /* sync out distance*/
    float               _fSyncOutDist;

} MEASURE_CONVEYOR_DISTANCE_RESPONSE, *LPMEASURE_CONVEYOR_DISTANCE_RESPONSE;

typedef struct _CONVEYOR_DISTANCE
{
    /* conveyor speed */
    float               _fSpeed;
    /* conveyor speed moving average filter size */
    unsigned int        _iSpeedFilterSize;
    /* minimum distance */
    float               _fMinDistance;
    /* watch window distance */
    float               _fWatchWinDist;
    /* maximum distance */
    float               _fMaxDistance;
    /* sync out distance*/
    float               _fOutTrackingDist;

} CONVEYOR_DISTANCE, *LPCONVEYOR_DISTANCE;

typedef struct _CONFIG_CONVEYOR
{
    char              _szName[MAX_SYMBOL_SIZE];
    /* 1st view */
    CONVEYOR_BASIC      _tBasic;
    /* 2nd view*/
    CONVEYOR_COORD_EX   _tCoord;
    /* 3th view*/
    CONVEYOR_DISTANCE   _tDistance;

} CONFIG_CONVEYOR, *LPCONFIG_CONVEYOR;

typedef struct _MEASURE_CONVEYOR_COORD
{
    /* Q1 */
    POSITION            _tPosTeachPointQ;
    /* Number of Teaching Point */
    unsigned char       _nTeachCount;
    /* P1~P5 */
    POSITION            _tPosTeachPointP[5];
    /* Encoder Count Values */
    unsigned int        _EncoderCount[5];

} MEASURE_CONVEYOR_COORD, *LPMEASURE_CONVEYOR_COORD;

typedef struct _MONITOR_CONVEYOR
{
    /* conveyor name */
    char              _szName[MAX_SYMBOL_SIZE];
    /* stop: 0, start: 1 */
    unsigned char      _bStart;

} MONITOR_CONVEYOR, *LPMONITOR_CONVEYOR;

typedef struct _CONVERYOR_OBJECT
{
    /* conveyor id */
    unsigned char      _iConId;
    /* time-out*/
    float              _tTimeout;
    /* container type, 0: FIFO, 1: LIFO */
    unsigned char        _iContainerType;
    /* object coordination */
    POSITION            _tPosObjCoord;

} CONVEYOR_OBJECT, *LPCONVEYOR_OBJECT;

typedef struct _CONVERYOR_TRACK
{
    /* conveyor id */
    unsigned char       _iConId;
    /* tracking or untracking */
    unsigned char        _bTracking;
    /* mate or not */
    unsigned char        _bMate;
    /* time-out*/
    //POSITION            _tPosMate;
    float               _fDuration;
    float               _fDummy[5];

} CONVEYOR_TRACK, *LPCONVEYOR_TRACK;

typedef struct _WELDING_CHANNEL
{
    /* channel : none(0), 1~2 */
    unsigned char               _bTargetCh;
    /* type : current(0), voltage: 1*/
    unsigned char               _bTargetAT;
    /* const value : A, B*/
    float                       _ConstValue[2];
    /* min value: default(0)  */
    float                       _fMinValue;
    /* max value: default(0)  */
    float                       _fMaxValue;
} WELDING_CHANNEL, *LPWELDING_CHANNEL;

typedef struct _CONFIG_WELDING_INTERFACE
{
    /*disable: 0 , enable: 1*/
    unsigned char               _bEnable;
    WELDING_CHANNEL             _tChOut[2];		//0th:for Voltage ch, 1st: for Current ch
    WELDING_CHANNEL             _tChIn[2];		//0th:for Voltage ch, 1st: for Current ch
    /* arc on minus: 0~15 */
    unsigned char               _iArcOnDO;
    /* gas on minus: 0~15 */
    unsigned char               _iGasOnDO;
    /* inching plus: 0~15 */
    unsigned char               _iInchPDO;
    /* inching minus: 0~15 */
    unsigned char               _iInchNDO;

} CONFIG_WELDING_INTERFACE,  *LPCONFIG_WELDING_INTERFACE;

typedef struct _CONFIG_WELD_SETTING
{
    /*disable: 0 , enable: 1*/
    unsigned char               _bVirtualMode;
    /* target voltage */
    float                       _fTargetVol;
    /* target current */
    float                       _fTargetCur;
    /* target Velocity */
    float                       _fTargetVel;
    /* target min Velocity */
    float                       _fTargetMinVel;
    /* target max Velocity */
    float                       _fTargetMaxVel;
    struct {
        /* ratio start */
        float                   _fRs;
        float                   _fTss;
        float                   _fTas;
        float                   _fTwc;
        /* ratio finish */
        float                   _fRf;
        float                   _fTaf;
        float                   _fTsf;
    } _tDetail;
} CONFIG_WELD_SETTING, *LPCONFIG_WELD_SETTING;

typedef CONFIG_WELD_SETTING
    GET_WELDING_SETTING_RESPONSE, *LPGET_WELDING_SETTING_RESPONSE;

typedef struct _ADJUST_WELDING_SETTING
{
    /* preset: 0, real time: 1*/
    unsigned char               _bRealTime;
    /* reset: current value(0), setting value(1) */
    unsigned char               _bResetFlag;
    /* target voltage */
    float                       _fTargetVol;
    /* target current */
    float                       _fTargetCur;
    /* target velocity */
    float                       _fTargetVel;
    /* weaving offset Y*/
    float                       _fOffsetY;
    /* weaving offset Z*/
    float                       _fOffsetZ;
    /* weaving width rate */
    float                       _fWidthRate;
} ADJUST_WELDING_SETTING, *LPADJUST_WELDING_SETTING;


typedef struct _MOVE_MOVESINE
{
    /* X : 0, Y: 1, Z: 2 */
    unsigned char               _iTargetAxs;
    /* base: 0, tool: 1*/
    unsigned char               _iTargetRef;
    /* acceleration time */
    float                       _fTargetAcc;
    /* sinusoidal amplitude */
    float                       _fAmplitude;
    /* sinusoidal frequency */
    float                       _fFrequency;
    /* repeat count*/
    unsigned int                _iRepeatCount;
    /* sync move: 0, async move: 1 */
    unsigned char               _iTargetAsyn;
} MOVE_MOVESINE, *LPMOVE_MOVESINE;

typedef struct _MOVE_MOVELISSAJOUS
{
    /* X : 0, Y: 1, Z: 2 */
    unsigned char               _iMotionType;
    /* XY : 0, YZ: 1, ZX: 2 */
    unsigned char               _iPlaneType;
    /* base: 0, tool: 1*/
    unsigned char               _iTargetRef;
    /* acceleration time */
    float                       _fTargetAcc;
    /* sinusoidal amplitude */
    float                       _fAmplitude;
    /* sinusoidal frequency */
    float                       _fFrequency;
    /* repeat count*/
    unsigned int                _iRepeatCount;
    /* sync move: 0, async move: 1 */
    unsigned char               _iTargetAsyn;
} MOVE_MOVELISSAJOUS, *LPMOVE_MOVELISSAJOUS;


typedef struct _CONFIG_TRAPEZOID_WEAVING_SETTING
{
    /* weaving offset Y*/
    float                       _fOffsetY;
    /* weaving offset Z*/
    float                       _fOffsetZ;
    /* weaving gradient */
    float                       _fGradient;
    /* weaving wPT1(X, Y) */
    float                       _fwPT1[2];
    /* weaving wPT2(X, Y) */
    float                       _fwPT2[2];
    /* weaving wT1 */
    float                       _fwT1;
    /* weaving wT2 */
    float                       _fwT2;
    /* weaving wTAcc1 */
    float                       _fwTAcc1;
    /* weaving wTAcc2 */
    float                       _fwTAcc2;
    /* weaving wTTD1 */
    float                       _fwTTD1;
    /* weaving wTTD2 */
    float                       _fwTTD2;
} CONFIG_TRAPEZOID_WEAVING_SETTING, *LPCONFIG_TRAPEZOID_WEAVING_SETTING;

typedef struct _CONFIG_ZIGZAG_WEAVING_SETTING
{
    /* weaving offset Y*/
    float                       _fOffsetY;
    /* weaving offset Z*/
    float                       _fOffsetZ;
    /* weaving gradient */
    float                       _fGradient;
    /* Weaving width */
    float                       _fWeavingWidth;
    /* Weaving Cycle */
    float                       _fWeavingCycle;
} CONFIG_ZIGZAG_WEAVING_SETTING, *LPCONFIG_ZIGZAG_WEAVING_SETTING;

typedef struct _CONFIG_CIRCULE_WEAVING_SETTING
{
    /* weaving offset Y*/
    float                       _fOffsetY;
    /* weaving offset Z*/
    float                       _fOffsetZ;
    /* weaving gradient */
    float                       _fGradient;
    /* Weaving width(wWdt) */
    float                       _fwWdt[2];
    /* Weaving Cycle(wT) */
    float                       _fwT[2];
} CONFIG_CIRCULE_WEAVING_SETTING, *LPCONFIG_CIRCULE_WEAVING_SETTING;

typedef struct _CONFIG_SINE_WEAVING_SETTING
{
    /* weaving offset Y*/
    float                       _fOffsetY;
    /* weaving offset Z*/
    float                       _fOffsetZ;
    /* weaving gradient */
    float                       _fGradient;
    /* Weaving width */
    float                       _fWeavingWidth;
    /* Weaving Cycle */
    float                       _fWeavingCycle;
} CONFIG_SINE_WEAVING_SETTING, *LPCONFIG_SINE_WEAVING_SETTING;

typedef struct _CONFIG_WELDING_DETAIL_INFO
{
    unsigned char               _iChannel;        /* 0 : 0 if not specified,1~2 : channel Number */
    unsigned char               _iChannelType;   /* 0 : Current, 1 : Voltage*/
    float                       _iRealMinOut;     /* 0 if not set */
    float                       _iMinOut;         /* 0 if not set */
    float                       _iRealMaxOut;     /* 0 if not set */
    float                       _iMaxOut;         /* 0 if not set */
}CONFIG_WELDING_DETAIL_INFO, *LPCONFIG_WELDING_DETAIL_INFO;

typedef struct _CONFIG_ANALOG_WELDING_INTERFACE
{
    unsigned char               _bMode;             /*Stop: 0 , Start: 1*/
    CONFIG_WELDING_DETAIL_INFO  _tTargetVoltage;
    CONFIG_WELDING_DETAIL_INFO  _tFeedingSpeed;
    CONFIG_WELDING_DETAIL_INFO  _tWeldingVoltage;
    CONFIG_WELDING_DETAIL_INFO  _tWeldingCurrent;

    /* arc on minus: 0~16 */
    unsigned char               _iArcOnDO;
    /* gas on minus: 0~16 */
    unsigned char               _iGasOnDO;
    /* inching plus: 0~16 */
    unsigned char               _iInchPDO;
    /* inching minus: 0~16 */
    unsigned char               _iInchNDO;
    /* Blow Out Value: 0~16 */
    unsigned char               _iBlowOutValue;
} CONFIG_ANALOG_WELDING_INTERFACE,  *LPCONFIG_ANALOG_WELDING_INTERFACE;

typedef struct _CONFIG_ANALOG_WELDING_SETTING
{
    unsigned char               _iVirtualWelding;   /* 0: Welding, 1: Virtual Welding*/
    float                       _fTargetVoltage;    /* Unit : V*/
    float                       _fTargetCurrent;    /* Unit : A*/
    float                       _fTargetVel;      /* Unit : mm/sec*/
    float                       _fMinVel;         /* Unit : mm/sec*/
    float                       _fMaxVel;         /* Unit : mm/sec*/
    struct
    {
        /* ratio start */
        float                   _fRs;
        /* 보호가스방출시간 */
        float                   _fTss;
        /* 시작전류시간 */
        float                   _fTas;
        /* 용접조건변경시간 */
        float                   _fTwc;
        /* ratio finish */
        float                   _fRf;
        /* 종료전류시간 */
        float                   _fTaf;
        /* 종료보호가스방출시간 */
        float                   _fTsf;
        /* 시작 전압 조건 */
        float                   _fStartVoltage;
        /* 종료 전압 조건 */
        float                   _fEndVoltage;
    } _tDetail;
    float                       _fTargetFeedingSpeed;

} CONFIG_ANALOG_WELDING_SETTING, *LPCONFIG_ANALOG_WELDING_SETTING;


typedef struct _ANALOG_WELDING_ADJUST_SETTING
{
    /* preset: 0, real time: 1*/
    unsigned char               _bRealTime;
    /* reset: current value(0), setting value(1) */
    unsigned char               _bResetFlag;
    /* target voltage */
    float                       _fTargetVol;
    /* Feeding Velocity */
    float                       _fFeedingVel;
    /* target velocity */
    float                       _fTargetVel;
    /* weaving offset Y*/
    float                       _fOffsetY;
    /* weaving offset Z*/
    float                       _fOffsetZ;
    /* weaving width rate */
    float                       _fWidthRate;
} ANALOG_WELDING_ADJUST_SETTING, *LPANALOG_WELDING_ADJUST_SETTING;
typedef struct _CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA
{
    unsigned char      _bEnable;                //0:Unused, 1: Used
    unsigned char      _nDataType;              //0:On/Off, 1:Value
    unsigned char      _nPositionalNumber;      //0:1, 1:0.1, 2:0.001
    float              _fMinData;
    float              _fMaxData;
    unsigned char      _nByteOffset;
    unsigned char      _nBitOffset;
    unsigned char      _nComnDataType;          //1Bit(Disable Low):0, 1Bit(Disable High):1, 2Bit:2, 4Bit:3 , 8Bit:4, 15Byte:5, 16Bit:6, 32Bit:7
    unsigned char      _nMaxDigitSize;          //Maximum value converted to digital value
}CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA, *LPCONFIG_DIGITAL_WELDING_IF_MAPPING_DATA;

typedef struct _CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS
{
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tWeldingStart;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tRobotReady;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tErrorReset;

}CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS, *LPCONFIG_DIGITAL_WELDING_INTERFACE_PROCESS;

typedef struct _CONFIG_DIGITAL_WELDING_INTERFACE_MODE
{
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tWeldingMode;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _t2T2TSpecial;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tPulseMode;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tWMopt1;

}CONFIG_DIGITAL_WELDING_INTERFACE_MODE, *LPCONFIG_DIGITAL_WELDING_INTERFACE_MODE;

typedef struct _CONFIG_DIGITAL_WELDING_INTERFACE_TEST
{
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tGasTest;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tInchingP;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tInchingM;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tBlowOutTorch;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tSimulation;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tTSopt1;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tTSopt2;

}CONFIG_DIGITAL_WELDING_INTERFACE_TEST, *LPCONFIG_DIGITAL_WELDING_INTERFACE_TEST;

typedef struct _CONFIG_DIGITAL_WELDING_INTERFACE_CONDITION
{
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tJobNumber;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tSynegicID;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tWireFeedSpeed;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tArclengthCorrection;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tDynamicCorrection;

}CONFIG_DIGITAL_WELDING_INTERFACE_CONDITION, *LPCONFIG_DIGITAL_WELDING_INTERFACE_CONDITION;

typedef struct _CONFIG_DIGITAL_WELDING_INTERFACE_OPTION
{
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption1;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption2;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption3;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption4;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption5;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption6;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption7;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption8;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption9;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption10;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption11;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption12;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption13;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption14;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption15;

}CONFIG_DIGITAL_WELDING_INTERFACE_OPTION, *LPCONFIG_DIGITAL_WELDING_INTERFACE_OPTION;

typedef struct _CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS2
{
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tCurrentFlow;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tProcessActive;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tMainCurrent;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tMachineReady;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tCommReady;

}CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS2, *LPCONFIG_DIGITAL_WELDING_INTERFACE_PROCESS2;

typedef struct _CONFIG_DIGITAL_WELDING_INTERFACE_MONITORING
{
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tWeldingVoltage;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tWeldingCurrent;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tWireFeedSpeed;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tWireStick;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tError;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tErrorNumber;

}CONFIG_DIGITAL_WELDING_INTERFACE_MONITORING, *LPCONFIG_DIGITAL_WELDING_INTERFACE_MONITORING;

typedef struct _CONFIG_DIGITAL_WELDING_INTERFACE_OTHER
{
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption1;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption2;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption3;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption4;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption5;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption6;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption7;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption8;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption9;
	CONFIG_DIGITAL_WELDING_IF_MAPPING_DATA _tOption10;

}CONFIG_DIGITAL_WELDING_INTERFACE_OTHER, *LPCONFIG_DIGITAL_WELDING_INTERFACE_OTHER;

typedef struct _ROBOT_LED_CONFIG
{
    unsigned char _szLedRule;
    unsigned char _szStateColor[SAFETY_STATE_LAST][2]; // [][0]: color 1, [][1]: color 2
    unsigned char _szCommandColor;

} ROBOT_LED_CONFIG, *LPROBOT_LED_CONFIG;

typedef struct _DIGITAL_WELDING_RESET
{
    unsigned char      _bReset;   //
}DIGITAL_WELDING_RESET, *LPDIGITAL_WELDING_RESET;
typedef struct _CONFIG_DIGITAL_WELDING_MODE
{
    unsigned char      _bMode;   //   0: Stop, 1:Start
}CONFIG_DIGITAL_WELDING_MODE, *LPCONFIG_DIGITAL_WELDING_MODE;

typedef struct _CONFIG_DIGITAL_WELDING_CONDITION
{
    unsigned char      _cVirtualWelding;
    float              _fTargetVel;
    float              _fMinVelLimit;
    float              _fMaxVelLimit;
    unsigned int       _nWeldingMode;
    unsigned int       _n2t2tSpecial;
    unsigned int       _nPulseMode;
    unsigned int       _nWMopt1;
    unsigned char      _cSimulation;
    unsigned char      _cTSopt1;
    unsigned char      _cTSopt2;
    unsigned int       _nJobNumber;
    unsigned int       _nSynergicID;
    float              _fWireFeedSpeed;
    float              _fArclengthCorrection;
    float              _fDynamicCorrection;
    float              _fOption1;
    float              _fOption2;
    float              _fOption3;
    float              _fOption4;
    float              _fOption5;
    float              _fOption6;
    float              _fOption7;
    float              _fOption8;
    float              _fOption9;
    float              _fOption10;
    float              _fOption11;
    float              _fOption12;
    float              _fOption13;
    float              _fOption14;
    float              _fOption15;
}CONFIG_DIGITAL_WELDING_CONDITION, *LPCONFIG_DIGITAL_WELDING_CONDITION;

typedef struct _CONFIG_DIGITAL_WELDING_ADJUST
{
     /* preset: 0, real time: 1*/
    unsigned char               _bRealTime;
    /* reset: current value(0), setting value(1) */
    unsigned char               _bResetFlag;
    /* target Feeding */
    float                       _fFeedingVel;
    /* target Velocity */
    float                       _fTargetVel;
    /* weaving offset Y*/
    float                       _fOffsetY;
    /* weaving offset Z*/
    float                       _fOffsetZ;
    /* weaving width rate */
    float                       _fWidthRate;

    float                       _fDynamicCor;

    float                       _fVoltageCor;

    int                         _nJobNumber;

    int                         _nSynergicID;
}CONFIG_DIGITAL_WELDING_ADJUST, *LPCONFIG_DIGITAL_WELDING_ADJUST;

typedef struct _MEASURE_TCP_WELDING
{
    /* orientation measure mode (not measure: 0, measure: 1) */
    unsigned char               _iMode;
    /* stick out */
    float                       _fStickout;
    /* reference pose */
    float                       _fTargetPos[9][NUMBER_OF_JOINT];

} MEASURE_TCP_WELDING, *LPMEASURE_TCP_WELDING;

typedef struct _TACK_WELDING_SETTING
{
    /* Deactivate: 0, Activate: 1 */
    unsigned char               _bEnable;
    /* Analog: 0, Digital: 1 */
    unsigned char               _bWeldingType;

} TACK_WELDING_SETTING, *LPTACK_WELDING_SETTING;


typedef struct _DIGITAL_FORCE_WRITE_DATA
{
    unsigned char              _cDataType;
    float                      _fData;
}DIGITAL_FORCE_WRITE_DATA, *LPDIGITAL_FORCE_WRITE_DATA;

typedef struct _WEAVING_OFFSET
{
    /* weaving offset Y*/
    float                       _fOffsetY;
    /* weaving offset Z*/
    float                       _fOffsetZ;
} WEAVING_OFFSET, *LPWEAVING_OFFSET;

typedef struct _READ_IETHERNET_SLAVE_DATA
{
    /* IE GPR data type 0 : bit, 1 : int, 2 : float */
    unsigned char      _iGprType;
    /* IE GPR data address */
    unsigned char      _iGprAddr;
    /* IE GPR data value */
    char                _szData[128];

} READ_IETHERNET_SLAVE_DATA, *LPREAD_IETHERNET_SLAVE_DATA;

typedef READ_IETHERNET_SLAVE_DATA
    MONITORING_IETHERNET_SLAVE, *LPMONITORING_IETHERNET_SLAVE;

typedef struct _COUNTER_BALANCE_PARAM_DATA {
    // Spring constant
    float                       _fK;
    // Irod (Connecting rod length)
    float                       _fIrod;
    // Distance between center of motor and connecting center of connecting rod
    float                       _fR;
    // Pre-load length of spring
    float                       _fSi;

} COUNTER_BALANCE_PARAM_DATA, *LPCOUNTER_BALANCE_PARAM_DATA;

typedef struct _CALIBRATION_PARAM_DATA {
    // Ax
    float                       _Ax;
    // Bx
    float                       _Bx;
    // Cx
    float                       _Cx;
    // Dx
    float                       _Dx;
} CALIBRATION_PARAM_DATA, *LPCALIBRATION_PARAM_DATA;


typedef struct _IETHERNET_SLAVE_DATA_EX
{
    unsigned char      _iGprType;
    unsigned char      _iGprAddr;
    unsigned char      _iInOut;
} IETHERNET_SLAVE_DATA_EX, *LPIETHERNET_SLAVE_DATA_EX;

typedef struct _IETHERNET_SLAVE_RESPONSE_DATA_EX
{
    unsigned char      _iGprType;
    unsigned char      _iGprAddr;
    unsigned char      _iInOut;
    char               _szData[128];
} IETHERNET_SLAVE_RESPONSE_DATA_EX, *LPIETHERNET_SLAVE_RESPONSE_DATA_EX;

typedef struct _MACHINE_TENDING_FOCAS_ERR_STRING
{
    unsigned short     _hHandle;
    short              _ErrorCode;
    char               _szErrorString[256];
}MACHINE_TENDING_FOCAS_ERR_STRING, *LPMACHINE_TENDING_FOCAS_ERR_STRING;


typedef struct _MACHINE_TENDING_FOCAS_CONNECT  //Connect Request& Response Commnon Struct
{
    short              _ErrorCode;
    char               _szIpAddr[16];
    unsigned short     _iPort;
    unsigned short     _hHandle;
    float              _fTimeOut;
} MACHINE_TENDING_FOCAS_CONNECT, *LPMACHINE_TENDING_FOCAS_CONNECT;


typedef struct _MACHINE_TENDING_FOCAS_DISCONNECT //Connect Request& Response Commnon Struct
{
    short              _ErrorCode;
    unsigned short     _hHandle;
} MACHINE_TENDING_FOCAS_DISCONNECT, *LPMACHINE_TENDING_FOCAS_DISCONNECT;


typedef struct _MACHINE_TENDING_FOCAS_PMC_DATA
{
    unsigned char  _PmcDataBool;
    char           _PmcDataChar[5];
    short          _PmcDataShort[5];
    long           _PmcDataLong[5];
    float          _PmcDataFloat[5];
    double         _PmcDataDouble[5];
}MACHINE_TENDING_FOCAS_PMC_DATA, *LPMACHINE_TENDING_FOCAS_PMC_DATA;


typedef struct _MACHINE_TENDING_FOCAS_PMC //PMC Infomation Header
{
    short              _ErrorCode;
    unsigned short     _hHandle;
    short              _iDataType;
    char               _szAddressType[2];//One String, Exam : "G", "D"...
    unsigned short     _iStartAddressNum;
    unsigned short     _iCount;//Read&Write DataCnt
    unsigned char      _iBitOffset;
    MACHINE_TENDING_FOCAS_PMC_DATA _tData;
} MACHINE_TENDING_FOCAS_PMC, *LPMACHINE_TENDING_FOCAS_PMC;


typedef struct _MACHINE_TENDING_FOCAS_CNC_PARAM
{
    unsigned short     _hHandle;
    short              _iParamNumber;
    short              _iAxisNumber;
    short              _iDataLength;
}MACHINE_TENDING_FOCAS_CNC_PARAM,*LPMACHINE_TENDING_FOCAS_CNC_PARAM;

typedef struct _MACHINE_TENDING_RESPONSE_FOCAS_CNC_PARAM
{
	short              _ErrorCode;
	unsigned short     _hHandle;
	struct
	{
		short              _iDataNumber;
		short              _iDataType;      //Axis Number
		char               _Data[256];
	}tData;
}MACHINE_TENDING_RESPONSE_FOCAS_CNC_PARAM, *LPMACHINE_TENDING_RESPONSE_FOCAS_CNC_PARAM;


typedef struct _MACHINE_TENDING_RESPONSE_FOCAS_PROGRAM_NUMBER
{
    short              _ErrorCode;
    unsigned short     _hHandle;
    short              _iRunningProgramNumber;
    short              _iMainProgramNumber;
}MACHINE_TENDING_RESPONSE_FOCAS_PROGRAM_NUMBER, *LPMACHINE_TENDING_RESPONSE_FOCAS_PROGRAM_NUMBER;


typedef struct _FOCAS_IS_ALIVE_RESPONSE 
{
    unsigned short handle; 
    unsigned short response; 
}FOCAS_IS_ALIVE_RESPONSE, *LPFOCAS_IS_ALIVE_RESPONSE ;

typedef struct _GRAVITY_PARAM_DATA
{
    COUNTER_BALANCE_PARAM_DATA _tClParam;

    CALIBRATION_PARAM_DATA     _tCrParam;

} GRAVITY_PARAM_DATA, *LPGRAVITY_PARAM_DATA;


typedef GRAVITY_PARAM_DATA
    CALIBRATE_GRAVITY_RESPONSE, *LPCALIBRATE_GRAVITY_RESPONSE;

typedef struct _CONFIG_INDUSTRIAL_ETHERNET
{
    char               _szEtherNetIpIpAddress[16];
    char               _szProfinetIpAddress[16];
    char               _szProfinetDeviceName[240];
    char               _szProfinetSubnetMask[16];
    char               _szProfinetGateway[16];
}CONFIG_INDUSTRIAL_ETHERNET, *LPCONFIG_INDUSTRIAL_ETHERNET;

typedef struct _SETUP_OPERARION_INDUSTRIAL_ETHERNET
{
    unsigned char      _nEtherNetIP_OpMode;    // 0 : Monitoring&GPR (476bytes)  1 : Only GPR (32bytes)
    CONFIG_INDUSTRIAL_ETHERNET tConfig;
}SETUP_OPERARION_INDUSTRIAL_ETHERNET, *LPSETUP_OPERARION_INDUSTRIAL_ETHERNET;

typedef struct _SYSTEM_VERSION_EX
{
    char                        _szPackage[MAX_SYMBOL_SIZE];
    /* smarttp version */
    char                        _szSmartTp[MAX_SYMBOL_SIZE];
    /* controller version */
    char                        _szController[MAX_SYMBOL_SIZE];
    /* interpreter version */
    char                        _szInterpreter[MAX_SYMBOL_SIZE];
    /* inverter version */
    char                        _szInverter[NUM_AXIS][MAX_SYMBOL_SIZE];
    /* SafetyBoard version */
    char                        _szSafetyBoard[MAX_SYMBOL_SIZE];
    /* robot serial number */
    char                        _szRobotSerial[MAX_SYMBOL_SIZE];
    /* robot model number*/
    char                        _szRobotModel[MAX_SYMBOL_SIZE];
    /* jts board version */
    char                        _szJTSBoard[MAX_SYMBOL_SIZE];
    /* flange board version */
    char                        _szFlangeBoard[MAX_SYMBOL_SIZE];
    /* flange board version */
    char                        _szSVMBoard[MAX_SYMBOL_SIZE];
} SYSTEM_VERSION_EX, *LPSYSTEM_VERSION_EX;

typedef struct _PACKAGE_LOCAL_UPDATE
{
    /* 0: non-update, 1:  update */
    unsigned char                _iTarget[UPDATE_TARGET_LAST];
    /* update path*/
    char                         _szDirName[MAX_STRING_SIZE];

} PACKAGE_LOCAL_UPDATE, *LPPACKAGE_LOCAL_UPDATE;

typedef struct _PACKAGE_NETWORK_UPDATE
{
    /* 0: non-update, 1:  update */
    unsigned char                _iTarget[UPDATE_TARGET_LAST];
    /* ttfp: 0, samba: 1 */
    unsigned char               _iNetType;
    /* tftp server Ip address */
    char                        _szIpAddress[16];
    /* file name */
    char                        _szFileName[MAX_STRING_SIZE];

} PACKAGE_NETWORK_UPDATE , *LPPACKAGE_NETWORK_UPDATE;

typedef struct _PACKAGE_UNZIP_COMMAND
{
    /* file name */
    char                        _szFileName[MAX_STRING_SIZE];

} PACKAGE_UNZIP_COMMAND, *LPACKAGE_UNZIP_COMMAND;


typedef PACKAGE_UNZIP_COMMAND
    SVM_LOCAL_UPDATE, *LPSVM_LOCAL_UPDATE;

typedef struct _SVM_NETWORK_UPDATE
{
    /* ttfp: 0, samba: 1 */
    unsigned char               _iNetType;
    /* tftp server Ip address */
    char                        _szIpAddress[16];
    /* file name */
    char                        _szFileName[MAX_STRING_SIZE];

} SVM_NETWORK_UPDATE , *LPSVM_NETWORK_UPDATE;

typedef struct _PACKAGE_RESTORE
{
    /* version name */
    char                         _szVersName[MAX_SYMBOL_SIZE];

} PACKAGE_RESTORE, *LPPACKAGE_RESTORE;

typedef struct _PACKAGE_UNZIP_RESPONSE
{
    /* 0: fail, 1:  success */
    unsigned char               _iResult;
    /* file name */
    char                        _szDirName[MAX_STRING_SIZE];

} PACKAGE_UNZIP_RESPONSE, *LPACKAGE_UNZIP_RESPONSE;

typedef struct _PACKAGE_RESTORE_LIST
{
    /* version name */
    char                        _szVersName[5][MAX_SYMBOL_SIZE];
    /* install date */
    char                        _szversDate[5][MAX_SYMBOL_SIZE];
    /* current version*/
    char                        _szCurrVers[MAX_SYMBOL_SIZE];
    /* recovery issue : ok(0), error(1~) */
    unsigned char               _iIssueCode[5];

} PACKAGE_RESTORE_LIST , *LPPACKAGE_RESTORE_LIST;

typedef POSITION RESPONSE_TRANS_PALLET_POS, *LPRESPONSE_TRANS_PALLET_POS;

typedef struct _CONTROL_TRANS_PALLET_POS
{
    float                           _fPosition1[6];
    float                           _fPosition2[6];
    float                           _fPosition3[6];
    float                           _fPosition4[6];
    unsigned char                   _cPattern;// 0 : Snake, 1 : zigzag
    unsigned short                  _sIndex;
    unsigned char                   _cRow;
    unsigned char                   _cColumn;
    unsigned char                   _cStack;
    float                           _fThickness;
    float                           _fOffsetValue[3];
}CONTROL_TRANS_PALLET_POS, *LPCONTROL_TRANS_PALLET_POS;

typedef struct _USER_COORD_EXTERNAL_FORCE_INFO
{
    unsigned char bIsMonitoring;
    unsigned char iUserID[MAX_USER_COORD_MONITORING_EXT_FORCE_SIZE];
}USER_COORD_EXTERNAL_FORCE_INFO, *LPUSER_COORD_EXTERNAL_FORCE_INFO;

typedef struct _MEASURE_FRICTION_RESPONSE
{
    /* measure result : 0(), 1() */
    unsigned char               _iResult[NUMBER_OF_JOINT];
    /* measrue error (N/m) */
    float                       _fError[NUMBER_OF_JOINT];
    /* poistive velocity */
    float                       _fPositive[4][NUMBER_OF_JOINT];
    /* negavite velocity */
    float                       _fNegative[4][NUMBER_OF_JOINT];
    /* temperatrue */
    float                       _fTemperature[NUMBER_OF_JOINT];

} MEASURE_FRICTION_RESPONSE, *LPMEASURE_FRICTION_RESPONSE;

typedef struct _MEASURE_TCP
{
    /* base: 0, tool: 1 */
    unsigned char               _iTargetRef;
    /* reference pose */
    float                       _fTargetPos[4][NUMBER_OF_JOINT];

} MEASURE_TCP, *LPMEASURE_TCP;

typedef struct _USER_COORDINATE_MATRIX_RESPONSE
{
    /* x, y, z in orientation */
    float                       _fOrientXYZ[3][3];
    /* x, y, z in translation */
    float                       _fTranslXYZ[3];

} USER_COORDINATE_MATRIX_RESPONSE, *LPUSER_COORDINATE_MATRIX_RESPONSE;

typedef POSITION POSITION_ADDTO_RESPONSE, *LPPOSITION_ADDTO_RESPONSE;

typedef struct _POSITION_ADDTO
{
    /* target pose */
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* target offset */
    float                       _fTargetVal[NUMBER_OF_JOINT];

} POSITION_ADDTO, *LPPOSITION_ADDTO;

typedef struct _MEASURE_FRICTION
{
    /* measure type : */
    unsigned char               _iType;
    /* select joint */
    unsigned char               _iSelect[NUMBER_OF_JOINT];
    /* start position */
    float                       _fStart[NUMBER_OF_JOINT];
    /* motion range */
    float                       _fRange[NUMBER_OF_JOINT];
} MEASURE_FRICTION, *LPMEASURE_FRICTION;

typedef struct _VIRTUAL_FENCE_RESPONSE
{
    unsigned char               _iCubeResult[6];
    unsigned char               _iPolyResult[6];
    unsigned char               _iCylinderResult;

} VIRTUAL_FENCE_RESPONSE, *LPVIRTUAL_FENCE_RESPONSE;

typedef struct _REPORT_TCP_CLIENT
{    
    unsigned char               _iId;
	unsigned char               _iCount;
} REPORT_TCP_CLIENT, *LPREPORT_TCP_CLIENT;

typedef struct _SERIAL_PORT_NAME
{
    /* serial port */
    char                        _szPort[16];
    /* serial name */
    char                        _szName[128];

} SERIAL_PORT_NAME, *LPSERIAL_PORT_NAME;

typedef struct _SERIAL_SEARCH
{
    /* number of Serial Device */
    int                       _nCount;
    /* port & name of each Serial Device */
    SERIAL_PORT_NAME            _tSerial[10];

} SERIAL_SEARCH, *LPSERIAL_SEARCH;

typedef struct _LICENSE_TEXT_PARAM {
    // position of license key
    unsigned char       _bLicenseInController;
    // license key string data
    unsigned char       _szLicenseKey[48];
} LICENSE_TEXT_PARAM, *LPLICENSE_TEXT_PARAM;

typedef struct _WRITE_MODBUS_RTU_MULTI_DATA
{
    /* symbol name */
    char                        _szSymbol[MAX_SYMBOL_SIZE];
    /* tty device */
    char                        _szttyPort[16];
    /* Slave ID*/
    int                         _iSlaveID;
    /* Baud Rate */
    int                         _iBaudRate;
    /* Byte size */
    int                         _iByteSize;
    /* Parity */
    char                        _szParity;
    /* Stop bit */
    int                         _iStopBit;
    /* i/o type */
    unsigned char               _iRegType;
    /* register start address */
    unsigned short              _iRegIndex;
    /* register count */
    unsigned char               _iRegCount;

} WRITE_MODBUS_RTU_MULTI_DATA, *LPWRITE_MODBUS_RTU_MULTI_DATA;

typedef struct _CONFIG_WEAVING_SETTING
{
    unsigned char               _iType;
    /* weaving offset Y*/
    float                       _fOffsetY;
    /* weaving offset Z*/
    float                       _fOffsetZ;
    /* weaving gradient */
    float                       _fGradient;

    union {
        struct {
            POINT_2D            _tPoint1;
            POINT_2D            _tPoint2;
            float               _fTMove1;
            float               _fTMove2;
            float               _fTAcc1;
            float               _fTAcc2;
            float               _fTDwell1;
            float               _fTDwell2;
        } _tTrap;

        struct {
            float               _fWidthX;
            float               _fWidthY;
            float               _fPeriodX;
            float               _fPeriodY;
        } _tCirc;

        unsigned char           _szBuffer[48];
    } _tDetail;
} CONFIG_WEAVING_SETTING, *LPCONFIG_WEAVING_SETTING;

typedef struct _WRITE_MODBUS_MULTI_DATA
{
    /* symbol name */
    char                        _szSymbol[MAX_SYMBOL_SIZE];
    /* tcp address */
    char                        _szIpAddr[16];
    /* tcp port */
    unsigned short              _iPort;
    /* Slave ID*/
    int                         _iSlaveID;
    /* i/o type */
    unsigned char               _iRegType;
    /* register start address */
    unsigned short              _iRegIndex;
    /* register count */
    unsigned char               _iRegCount;

} WRITE_MODBUS_MULTI_DATA, *LPWRITE_MODBUS_MULTI_DATA;

typedef WRITE_MODBUS_MULTI_DATA
        WRITE_MODBUS_TCP_MULTI_DATA, *LPWRITE_MODBUS_TCP_MULTI_DATA;

typedef struct _ROBOT_WELDING_DATA
{
	/* adj available status: 0~1 */
	unsigned char               _iAdjAvail;
    /* target voltage */
    float                       _fTargetVol;
    /* target current */
    float                       _fTargetCur;
    /* target Velocity */
    float                       _fTargetVel;
    /* actual voltage */
    float                       _fActualVol;
    /* actual current */
    float                       _fActualCur;
    /* weaving offset Y */
    float                       _fOffsetY;
    /* weaving offset Z */
    float                       _fOffsetZ;
    /* arc on : 0~1*/
    unsigned char               _iArcOnDO;
    /* gas on : 0~1 */
    unsigned char               _iGasOnDO;
    /* inching plus: 0~1 */
    unsigned char               _iInchPDO;
    /* inching minus: 0~1 */
    unsigned char               _iInchNPO;
    /* welding status: 0(S)~1(F) */
    unsigned char               _iStatus;
} ROBOT_WELDING_DATA, *LPROBOT_WELDING_DATA;

typedef ROBOT_WELDING_DATA
    MONITORING_WELDING, *LPMONITORING_WELDING;

typedef struct _ROBOT_ALALOG_WELDING_DATA
{
    /* adj available status: 0~1 */
    unsigned char               _iAdjAvail;
    /* target voltage */
    float                       _fTargetVol;
    /* target current */
    float                       _fTargetCur;
    /* target Velocity */
    float                       _fTargetVel;
    /* actual voltage */
    float                       _fActualVol;
    /* actual current */
    float                       _fActualCur;
    /* weaving offset Y */
    float                       _fOffsetY;
    /* weaving offset Z */
    float                       _fOffsetZ;
    /* arc on : 0~1*/
    unsigned char               _iArcOnDO;
    /* gas on : 0~1 */
    unsigned char               _iGasOnDO;
    /* inching plus: 0~1 */
    unsigned char               _iInchPDO;
    /* inching minus: 0~1 */
    unsigned char               _iInchNPO;
    /* welding status: 0(S)~1(F) */
    unsigned char               _iStatus;
    /* BlowOutValue */
    unsigned char               _iBlowOut;
    /* FeedingVel */
    float                       _iFeedingVel;
} ROBOT_ALALOG_WELDING_DATA, *LPROBOT_ALALOG_WELDING_DATA;

typedef ROBOT_ALALOG_WELDING_DATA
    MONITORING_ALALOG_WELDING, *LPMONITORING_ALALOG_WELDING;

typedef struct _ROBOT_DIGITAL_WELDING_DATA
{
    /* adj available status: 0~1 */
    unsigned char               _iAdjAvail;
    /* target voltage */
    float                       _fTargetVol;
    /* target current */
    float                       _fTargetCur;
    /* target Velocity */
    float                       _fTargetVel;
    /* actual voltage */
    float                       _fActualVol;
    /* actual current */
    float                       _fActualCur;
    /* weaving offset Y */
    float                       _fOffsetY;
    /* weaving offset Z */
    float                       _fOffsetZ;
    /* arc on : 0~1*/
    unsigned char               _iArcOnDO;
    /* gas on : 0~1 */
    unsigned char               _iGasOnDO;
    /* inching plus: 0~1 */
    unsigned char               _iInchPDO;
    /* inching minus: 0~1 */
    unsigned char               _iInchNPO;
    /* welding status: 0(S)~1(F) */
    unsigned char               _iStatus;
    /* BlowOutValue */
    unsigned char               _iBlowOut;
    /* FeedingVel */
    float                       _fFeedingVel;
    /* actual feeding velocity */
    float                       _fActualFeedingVel;
    /* error number */
    int                         _iErrorNumber;
    /* wire stick */
    float                       _fWireStick;
    /* error */
    int                         _iError;
    /* option 1 */
    float                       _fOption1;
    /* option 2 */
    float                       _fOption2;
    /* option 3 */
    float                       _fOption3;
    /* option 4 */
    float                       _fOption4;
    /* option 5 */
    float                       _fOption5;
    /* option 6 */
    float                       _fOption6;
    /* option 7 */
    float                       _fOption7;
    /* option 8 */
    float                       _fOption8;
    /* option 9 */
    float                       _fOption9;
    /* option 10 */
    float                       _fOption10;
    /* current flow */
    unsigned char               _iCurrentFlow;
    /* process active */
    unsigned char               _iProcessActive;
    /* machine ready */
    unsigned char               _iMachineryReady;
    /* voltage correction */
    float                       _fVoltageCorrection;
    /* dynamic correction */
    float                       _fDynamicCorrection;

} ROBOT_DIGITAL_WELDING_DATA, *LPROBOT_DIGITAL_WELDING_DATA;

typedef ROBOT_DIGITAL_WELDING_DATA
    MONITORING_DIGITAL_WELDING, *LPMONITORING_DIGITAL_WELDING;

typedef struct _DIGITAL_WELDING_COMM_STATE
{
    unsigned char               _cWeldingMachineOnline; //0:Offline, 1:Online
    unsigned char               _cWeldingEipSlaveState; //0:EIP Slave Offline, 1:Master OnLine, 2:RobotData Online, 3:WeldingMachine Online
}DIGITAL_WELDING_COMM_STATE, *LPDIGITAL_WELDING_COMM_STATE;

typedef struct _USER_COORD_EXTERNAL_FORCE
{
    unsigned char            _iUserId;
    float                    _fExternalForce[6];
}USER_COORD_EXTERNAL_FORCE, *LPUSER_COORD_EXTERNAL_FORCE;

typedef struct _USER_COORD_EXTERNAL_FORCE_PACKET
{
	USER_COORD_EXTERNAL_FORCE _iUserCoord[MAX_USER_COORD_MONITORING_EXT_FORCE_SIZE];
}USER_COORD_EXTERNAL_FORCE_PACKET, *LPUSER_COORD_EXTERNAL_FORCE_PACKET;


typedef struct _MONITORING_MBUS_SLAVE_COIL
{
    unsigned short                      _nCtrlDigitalInput;
    unsigned short                      _nCtrlDigitalOutput;
    unsigned char                       _nToolDigitalInput;
    unsigned char                       _nToolDigitalOutput;
    unsigned char                       _nServoOnRobot;
    unsigned char                       _nEmergencyStopped;
    unsigned char                       _nSafetyStopped;
    unsigned char                       _nDirectTeachButtonPress;
    unsigned char                       _nPowerButtonPress;
    unsigned char                       _nSafetyStoppedRequiredRecoveryMode;
}MONITORING_MBUS_SLAVE_COIL;

typedef struct _MONITORING_MBUS_SLAVE_HOILDING_REGISTER
{
    unsigned short                      _nCtrlDigitalInput;
    unsigned short                      _nCtrlDigitalOutput;
    unsigned short                      _nCtrlAnalogInput1;
    unsigned short                      _nCtrlAnalogInput1Type;
    unsigned short                      _nCtrlAnalogInput2;
    unsigned short                      _nCtrlAnalogInput2Type;
    unsigned short                      _nCtrlAnalogOutput1;
    unsigned short                      _nCtrlAnalogOutput1Type;
    unsigned short                      _nCtrlAnalogOutput2;
    unsigned short                      _nCtrlAnalogOutput2Type;
    unsigned short                      _nCtrlToolDigitalInput;
    unsigned short                      _nCtrlToolDigitalOutput;
    unsigned short                      _nGPR[128];
    unsigned short                      _nCtrlMajorVer;
    unsigned short                      _nCtrlMinorVer;
    unsigned short                      _nCtrlPatchVer;
    unsigned short                      _nRobotState;
    unsigned short                      _nServoOnRobot;
    unsigned short                      _nEmergencyStopped;
    unsigned short                      _nSafetyStopped;
    unsigned short                      _nDirectTeachButtonPressed;
    unsigned short                      _nPowerButtonPressed;
    unsigned short                      _nJointPosition[NUMBER_OF_JOINT];
    unsigned short                      _nJointVelocity[NUMBER_OF_JOINT];
    unsigned short                      _nJointMotorCurrent[NUMBER_OF_JOINT];
    unsigned short                      _nJointMotorTemp[NUMBER_OF_JOINT];
    unsigned short                      _nJointTorque[NUMBER_OF_JOINT];
    unsigned short                      _nTaskPosition[NUMBER_OF_JOINT];
    unsigned short                      _nTaskVelocity[NUMBER_OF_JOINT];
    unsigned short                      _nToolOffsetLength[NUMBER_OF_JOINT];
    unsigned short                      _nTaskExternalForce[NUMBER_OF_JOINT];
}MONITORING_MBUS_SLAVE_HOILDING_REGISTER;

typedef struct _MONITORING_IE_GPR
{
    unsigned char                       _nGpr[464];         //IE_MONITORING_PACKET_TOTAL_DATA
}MONITORING_IE_GPR;

typedef struct _MONITORING_IE_SLAVE
{
    MONITORING_MBUS_SLAVE_COIL                  _tMbusCoil;
    MONITORING_MBUS_SLAVE_HOILDING_REGISTER     _tMbusHoldingRegister;
    MONITORING_IE_GPR                           _tIndustrialEthernetGPR;
}MONITORING_IE_SLAVE, *LPMONITORING_IE_SLAVE;

typedef struct _CONFIG_SAFETY_PARAM_ENABLE 
{
	unsigned short _wPreviousCmdid;
	unsigned int _iRefCrc32;
} CONFIG_SAFETY_PARAM_ENABLE, *LPCONFIG_SAFETY_PARAM_ENABLE;

typedef struct _PROGRAM_EXECUTION_EX
{
    /* line number */
    unsigned int         _iLineNumber;
    /* elapse time */
    float               _fElapseTime;
    /* file name */
    char                _szFile[MAX_STRING_SIZE];

} PROGRAM_EXECUTION_EX, *LPPROGRAM_EXECUTION_EX;

typedef struct _PROGRAM_WATCH_VARIABLE
{
    /* install:0,  geneal: 1*/
    unsigned char        _iDivision;
    /* bool: 0, int: 1, flaot: 2, string: 3, posj: 4, posx: 5, unknonwn: 6*/
    unsigned char        _iType;
    /* variable name */
    char                _szName[128];
    /* data */
    char                _szData[128];

} PROGRAM_WATCH_VARIABLE, *LPPROGRAM_WATCH_VARIABLE;

typedef struct _PROGRAM_ERROR
{
    /* error code */
    unsigned int                 _iError;
    /* line number */
    unsigned int                 _nLine;
    /* file name */
    char                         _szFile[MAX_STRING_SIZE];

} PROGRAM_ERROR, *LPPROGRAM_ERROR;



typedef struct _SAFETY_CONFIGURATION_EX
{
    unsigned int _iDataVersion;
    CONFIG_JOINT_RANGE _tJointRange;
    CONFIG_GENERAL_RANGE _tGeneralRange;
    float _fCollisionSensitivity;
    CONFIG_SAFETY_FUNCTION _tSafetyFunc;
    CONFIG_TOOL_SYMBOL _tTool;
    CONFIG_TCP_SYMBOL _tTcp;
    CONFIG_INSTALL_POSE _tInstallPose;
    CONFIG_SAFETY_IO _tSafetyIO;
    //CONFIG_SAFETY_IO_EX         _tSafetyIO;

    CONFIG_VIRTUAL_FENCE _tSafetySpaceVF;
    CONFIG_SAFE_ZONE _tSafetySpaceSZ;
    ENABLE_SAFE_ZONE _tSafetySpaceESZ;
    CONFIG_PROTECTED_ZONE _tSafetySpacePZ;
    CONFIG_COLLISION_MUTE_ZONE _tSafetySpaceCM;
    CONFIG_TOOL_ORIENTATION_LIMIT_ZONE _tSafetySpaceTO;
    CONFIG_TOOL_SHAPE _tSafetySpaceTS;

    CONFIG_NUDGE _tConfigNudge;
    CONFIG_COCKPIT_EX _tCockPit;
    CONFIG_IDLE_OFF _tIdleOff;
    CONFIG_TCP_LIST _tConfigTCP;
    CONFIG_TOOL_LIST _tConfigTool;
    CONFIG_TOOL_SHAPE_LIST _tConfigToolShape;

    char                _szActiveTcp[MAX_SYMBOL_SIZE];
    char                _szActiveTool[MAX_SYMBOL_SIZE];
    char                _szActiveToolShape[MAX_SYMBOL_SIZE];

    MODBUS_DATA_LIST _tModbusList;
    CONFIG_WORLD_COORDINATE     _tWorld2BaseRelation;
    float m_CwsSpeedRatio;
    float m_IoSpeedRatio;


    int _iSafetyZoneCount;
    CONFIG_SAFETY_ZONE _tSafetyZone[20];



    int _iUserCoordCount;
    CONFIG_USER_COORDINATE_EX _tUserCoordinates[20];

    CONFIG_CONFIGURABLE_IO _tConfigurableIO;

} SAFETY_CONFIGURATION_EX, *LPSAFETY_CONFIGURATION_EX;

typedef struct _SAFETY_CONFIGURATION_EX2
{
    unsigned int _iDataVersion;
    CONFIG_JOINT_RANGE _tJointRange;
    CONFIG_GENERAL_RANGE _tGeneralRange;
    float _fCollisionSensitivity;
    CONFIG_SAFETY_FUNCTION _tSafetyFunc;
    CONFIG_TOOL_SYMBOL _tTool;
    CONFIG_TCP_SYMBOL _tTcp;
    CONFIG_INSTALL_POSE _tInstallPose;
    CONFIG_SAFETY_IO_OP _tSafetyIO;
    //CONFIG_SAFETY_IO_EX         _tSafetyIO;

    CONFIG_VIRTUAL_FENCE _tSafetySpaceVF;
    CONFIG_SAFE_ZONE _tSafetySpaceSZ;
    ENABLE_SAFE_ZONE _tSafetySpaceESZ;
    CONFIG_PROTECTED_ZONE _tSafetySpacePZ;
    CONFIG_COLLISION_MUTE_ZONE _tSafetySpaceCM;
    CONFIG_TOOL_ORIENTATION_LIMIT_ZONE _tSafetySpaceTO;
    CONFIG_TOOL_SHAPE _tSafetySpaceTS;

    CONFIG_NUDGE _tConfigNudge;
    CONFIG_COCKPIT_EX _tCockPit;
    CONFIG_IDLE_OFF _tIdleOff;
    CONFIG_TCP_LIST _tConfigTCP;
    CONFIG_TOOL_LIST _tConfigTool;
    CONFIG_TOOL_SHAPE_LIST _tConfigToolShape;

    char                _szActiveTcp[MAX_SYMBOL_SIZE];
    char                _szActiveTool[MAX_SYMBOL_SIZE];
    char                _szActiveToolShape[MAX_SYMBOL_SIZE];

    MODBUS_DATA_LIST _tModbusList;
    CONFIG_WORLD_COORDINATE     _tWorld2BaseRelation;
    float m_CwsSpeedRatio;
    float m_IoSpeedRatio;


    int _iSafetyZoneCount;
    CONFIG_SAFETY_ZONE _tSafetyZone[20];



    int _iUserCoordCount;
    CONFIG_USER_COORDINATE_EX _tUserCoordinates[100];

    CONFIG_CONFIGURABLE_IO _tConfigurableIO;

} SAFETY_CONFIGURATION_EX2, *LPSAFETY_CONFIGURATION_EX2;


typedef struct _POSITION_EX
{
	//// member variable
	union {
		/* posj */
		struct {
			/* joint position (posj) */
			float                   _pos[NUMBER_OF_JOINT];
			///* reserved: additional axes later */
			//float                   _reserved[10];
			///* dummy */
			//unsigned char           _reserved_[8];
		} _posj;

		/* posx */
		struct {
			/* task position */
			float                   _pos[NUMBER_OF_TASK_EX];
			///* reserved: additional axes later */
			//float                   _fReserved[9];
			/* orientation type (0: Euler ZYZ, 1: Euler ZYX, 2: Euler XYZ, 3: Fixed XYZ, 4: Axis-Angle, 5: Quaternion ) */
			unsigned char           _ori_type;
			/* solution space (0~7: fixed, 255: auto) */
			unsigned char           _sol_space;
			/* multi turn (255: auto) */
			unsigned char           _multi_turn;
			///* dummy */
			//unsigned char           _reserved[5];
		} _posx;
	};

	unsigned char                   _pos_type;      // posx, posj
	//unsigned char                   _reserved[3];

} POSITION_EX, *LPPOSITION_EX; // SUPPORT_ORIENTATION_TYPE

typedef POSITION_EX CONFIG_TCP_EX, *LPCONFIG_TCP_EX; // SUPPORT_ORIENTATION_TYPE


typedef struct _CONFIG_TCP_SYMBOL_EX
{
	/* tcp name */
	char                _szSymbol[MAX_SYMBOL_SIZE];
	/* tcp data */
	CONFIG_TCP_EX          _tTCP;

} CONFIG_TCP_SYMBOL_EX, *LPCONFIG_TCP_SYMBOL_EX; // SUPPORT_ORIENTATION_TYPE

typedef struct _CONFIG_TCP_LIST_EX
{
	int                         _iToolCount;
	CONFIG_TCP_SYMBOL_EX           _tTooList[MAX_CONFIG_TCP_SIZE];
} CONFIG_TCP_LIST_EX, *LPCONFIG_TCP_LIST_EX; // SUPPORT_ORIENTATION_TYPE

typedef struct _CONFIG_WORLD_COORDINATE_EX
{
	/* setting type : world2base: 0, base2ref: 1, world2ref: 2 */
	/* setting flag : disable: 0, enable: 1*/
	unsigned char               _iType;
	/* dummy */
	unsigned char               _iReserved[3];
	/* target pose */
	POSITION_EX                 _tPosition;

} CONFIG_WORLD_COORDINATE_EX, *LPCONFIG_WORLD_COORDINATE_EX; // SUPPORT_ORIENTATION_TYPE

typedef struct _CONFIG_USER_COORDINATE_EX2
{
	/* base: 0, world: 2 */
	unsigned char               _iTargetRef;
	/* dummy */
	unsigned char               _iReserved[3];
	/* task position*/
	POSITION_EX                 _tTargetPos;
	/* unified id */
	unsigned char               _iUserID;
	/* dummy */
	unsigned char               _iReserved_[3];

} CONFIG_USER_COORDINATE_EX2, *LPCONFIG_USER_COORDINATE_EX2; // SUPPORT_ORIENTATION_TYPE

typedef struct _SAFETY_CONFIGURATION_EX2_V3
{
	unsigned int _iDataVersion;
	CONFIG_JOINT_RANGE _tJointRange;
	CONFIG_GENERAL_RANGE _tGeneralRange;
	float _fCollisionSensitivity;
	CONFIG_SAFETY_FUNCTION _tSafetyFunc;
	CONFIG_TOOL_SYMBOL _tTool;
	CONFIG_TCP_SYMBOL_EX _tTcp; //v3
	CONFIG_INSTALL_POSE _tInstallPose;
	//CONFIG_SAFETY_IO _tSafetyIO;
	CONFIG_SAFETY_IO_EX         _tSafetyIO;

	CONFIG_VIRTUAL_FENCE _tSafetySpaceVF;
	CONFIG_SAFE_ZONE _tSafetySpaceSZ;
	ENABLE_SAFE_ZONE _tSafetySpaceESZ;
	CONFIG_PROTECTED_ZONE _tSafetySpacePZ;
	CONFIG_COLLISION_MUTE_ZONE _tSafetySpaceCM;
	CONFIG_TOOL_ORIENTATION_LIMIT_ZONE _tSafetySpaceTO;
	CONFIG_TOOL_SHAPE _tSafetySpaceTS;

	CONFIG_NUDGE _tConfigNudge;
	CONFIG_COCKPIT_EX _tCockPit;
	CONFIG_IDLE_OFF _tIdleOff;
	CONFIG_TCP_LIST_EX _tConfigTCP; //v3
	CONFIG_TOOL_LIST _tConfigTool; 
	CONFIG_TOOL_SHAPE_LIST _tConfigToolShape;

	char                _szActiveTcp[MAX_SYMBOL_SIZE];
	char                _szActiveTool[MAX_SYMBOL_SIZE];
	char                _szActiveToolShape[MAX_SYMBOL_SIZE];

	MODBUS_DATA_LIST _tModbusList;
	CONFIG_WORLD_COORDINATE_EX     _tWorld2BaseRelation; //v3
	float m_CwsSpeedRatio;
	float m_IoSpeedRatio;


	int _iSafetyZoneCount;
	CONFIG_SAFETY_ZONE _tSafetyZone[20];

	int _iUserCoordCount;
	CONFIG_USER_COORDINATE_EX2 _tUserCoordinates[100]; //v3

	//CONFIG_CONFIGURABLE_IO _tConfigurableIO;
	CONFIG_CONFIGURABLE_IO_EX _tConfigurableIO;

} SAFETY_CONFIGURATION_EX2_V3, *LPSAFETY_CONFIGURATION_EX2_V3;



#pragma pack()
