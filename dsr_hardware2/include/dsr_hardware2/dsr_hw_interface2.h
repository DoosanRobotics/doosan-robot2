/*********************************************************************
 *
 *  Inferfaces for doosan robot controllor 
 * Author: Minsoo Song (minsoo.song@doosan.com)
 *
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
#ifndef DSR_HARDWARE2__DR_HW_INTERFACE2_H
#define DSR_HARDWARE2__DR_HW_INTERFACE2_H

//ROS2 #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/thread/thread.hpp>
#include <array>
#include <algorithm>  // std::copy

// #ifdef _OLD_ROS2_CONTROL_
//     #include <hardware_interface/joint_command_handle.hpp>
//     #include <hardware_interface/joint_state_handle.hpp>
// #else 
//     #include "hardware_interface/joint_handle.hpp"
// #endif

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// msg
#include "dsr_msgs2/msg/robot_error.hpp"
#include "dsr_msgs2/msg/robot_state.hpp"
#include "dsr_msgs2/msg/robot_stop.hpp"
#include "dsr_msgs2/msg/jog_multi_axis.hpp"



// service
//system
#include "dsr_msgs2/srv/set_robot_mode.hpp"
#include "dsr_msgs2/srv/get_robot_mode.hpp"
#include "dsr_msgs2/srv/set_robot_system.hpp"
#include "dsr_msgs2/srv/get_robot_system.hpp"
#include "dsr_msgs2/srv/get_robot_state.hpp"
#include "dsr_msgs2/srv/set_robot_speed_mode.hpp"
#include "dsr_msgs2/srv/get_robot_speed_mode.hpp"
#include "dsr_msgs2/srv/get_current_pose.hpp"
#include "dsr_msgs2/srv/set_safe_stop_reset_type.hpp"
#include "dsr_msgs2/srv/get_last_alarm.hpp"

// motion
#include "dsr_msgs2/srv/move_joint.hpp"
#include "dsr_msgs2/srv/move_line.hpp"
#include "dsr_msgs2/srv/move_jointx.hpp"
#include "dsr_msgs2/srv/move_circle.hpp"
#include "dsr_msgs2/srv/move_spline_joint.hpp"
#include "dsr_msgs2/srv/move_spline_task.hpp"
#include "dsr_msgs2/srv/move_blending.hpp"
#include "dsr_msgs2/srv/move_spiral.hpp"
#include "dsr_msgs2/srv/move_periodic.hpp"
#include "dsr_msgs2/srv/move_wait.hpp"
#include "dsr_msgs2/srv/jog.hpp"
#include "dsr_msgs2/srv/jog_multi.hpp"
#include "dsr_msgs2/srv/move_pause.hpp"
#include "dsr_msgs2/srv/move_stop.hpp"
#include "dsr_msgs2/srv/move_resume.hpp"
#include "dsr_msgs2/srv/trans.hpp"
#include "dsr_msgs2/srv/fkin.hpp"
#include "dsr_msgs2/srv/ikin.hpp"
#include "dsr_msgs2/srv/set_ref_coord.hpp"
#include "dsr_msgs2/srv/move_home.hpp"
#include "dsr_msgs2/srv/check_motion.hpp"
#include "dsr_msgs2/srv/change_operation_speed.hpp"
#include "dsr_msgs2/srv/enable_alter_motion.hpp"
#include "dsr_msgs2/srv/alter_motion.hpp"
#include "dsr_msgs2/srv/disable_alter_motion.hpp"
#include "dsr_msgs2/srv/set_singularity_handling.hpp"

//----- auxiliary_control
#include "dsr_msgs2/srv/get_control_mode.hpp"          
#include "dsr_msgs2/srv/get_control_space.hpp"         
#include "dsr_msgs2/srv/get_current_posj.hpp"          
#include "dsr_msgs2/srv/get_current_velj.hpp"          
#include "dsr_msgs2/srv/get_desired_posj.hpp"
#include "dsr_msgs2/srv/get_desired_velj.hpp"          
#include "dsr_msgs2/srv/get_current_posx.hpp"          
#include "dsr_msgs2/srv/get_current_tool_flange_posx.hpp"
#include "dsr_msgs2/srv/get_current_velx.hpp"          
#include "dsr_msgs2/srv/get_desired_posx.hpp"
#include "dsr_msgs2/srv/get_desired_velx.hpp"          
#include "dsr_msgs2/srv/get_current_solution_space.hpp" 
#include "dsr_msgs2/srv/get_current_rotm.hpp"          
#include "dsr_msgs2/srv/get_joint_torque.hpp"          
#include "dsr_msgs2/srv/get_external_torque.hpp"      
#include "dsr_msgs2/srv/get_tool_force.hpp"            
#include "dsr_msgs2/srv/get_solution_space.hpp"
#include "dsr_msgs2/srv/get_orientation_error.hpp"

//----- force/stiffness
#include "dsr_msgs2/srv/parallel_axis1.hpp"
#include "dsr_msgs2/srv/parallel_axis2.hpp"
#include "dsr_msgs2/srv/align_axis1.hpp"
#include "dsr_msgs2/srv/align_axis2.hpp"
#include "dsr_msgs2/srv/is_done_bolt_tightening.hpp"
#include "dsr_msgs2/srv/release_compliance_ctrl.hpp"
#include "dsr_msgs2/srv/task_compliance_ctrl.hpp"
#include "dsr_msgs2/srv/set_stiffnessx.hpp"
#include "dsr_msgs2/srv/calc_coord.hpp"
#include "dsr_msgs2/srv/set_user_cart_coord1.hpp"
#include "dsr_msgs2/srv/set_user_cart_coord2.hpp"
#include "dsr_msgs2/srv/set_user_cart_coord3.hpp"
#include "dsr_msgs2/srv/overwrite_user_cart_coord.hpp"
#include "dsr_msgs2/srv/get_user_cart_coord.hpp"
#include "dsr_msgs2/srv/set_desired_force.hpp"
#include "dsr_msgs2/srv/release_force.hpp"
#include "dsr_msgs2/srv/check_position_condition.hpp"
#include "dsr_msgs2/srv/check_force_condition.hpp"
#include "dsr_msgs2/srv/check_orientation_condition1.hpp"
#include "dsr_msgs2/srv/check_orientation_condition2.hpp"
#include "dsr_msgs2/srv/coord_transform.hpp"
#include "dsr_msgs2/srv/get_workpiece_weight.hpp"
#include "dsr_msgs2/srv/reset_workpiece_weight.hpp"

//io
#include "dsr_msgs2/srv/set_ctrl_box_digital_output.hpp"
#include "dsr_msgs2/srv/get_ctrl_box_digital_input.hpp"
#include "dsr_msgs2/srv/set_tool_digital_output.hpp"
#include "dsr_msgs2/srv/get_tool_digital_input.hpp"
#include "dsr_msgs2/srv/set_ctrl_box_analog_output.hpp"
#include "dsr_msgs2/srv/get_ctrl_box_analog_input.hpp"
#include "dsr_msgs2/srv/set_ctrl_box_analog_output_type.hpp"
#include "dsr_msgs2/srv/set_ctrl_box_analog_input_type.hpp"
#include "dsr_msgs2/srv/get_ctrl_box_digital_output.hpp"
#include "dsr_msgs2/srv/get_tool_digital_output.hpp"

//modbus
#include "dsr_msgs2/srv/set_modbus_output.hpp"
#include "dsr_msgs2/srv/get_modbus_input.hpp"
#include "dsr_msgs2/srv/config_create_modbus.hpp"
#include "dsr_msgs2/srv/config_delete_modbus.hpp"

//drl
#include "dsr_msgs2/srv/drl_pause.hpp"
#include "dsr_msgs2/srv/drl_start.hpp"
#include "dsr_msgs2/srv/drl_stop.hpp"
#include "dsr_msgs2/srv/drl_resume.hpp"
#include "dsr_msgs2/srv/get_drl_state.hpp"


//tcp
#include "dsr_msgs2/srv/config_create_tcp.hpp"
#include "dsr_msgs2/srv/config_delete_tcp.hpp"
#include "dsr_msgs2/srv/get_current_tcp.hpp"
#include "dsr_msgs2/srv/set_current_tcp.hpp"

//tool
#include "dsr_msgs2/srv/config_create_tool.hpp"
#include "dsr_msgs2/srv/config_delete_tool.hpp"
#include "dsr_msgs2/srv/get_current_tool.hpp"
#include "dsr_msgs2/srv/set_current_tool.hpp"
#include "dsr_msgs2/srv/set_tool_shape.hpp"

//gripper
#include "dsr_msgs2/srv/robotiq2_f_open.hpp"
#include "dsr_msgs2/srv/robotiq2_f_close.hpp"
#include "dsr_msgs2/srv/robotiq2_f_move.hpp"

//serial
#include "dsr_msgs2/srv/serial_send_data.hpp"

// moveit
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>  //ROS2 ???

#include "../../../dsr_common2/include/DRFLEx.h"
//TODO #include "../../../dsr_common2/include/dsr_serial.h"


#ifndef PI
#define PI 3.14159265359
#endif
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)

//_____ defines for Dooan Robot Controller _______________
#define POINT_COUNT         6

// solution space
#define DR_SOL_MIN          0
#define DR_SOL_MAX          7

// posb seg_type
#define DR_LINE             0
#define DR_CIRCLE           1

// move reference
#define DR_BASE             0
#define DR_TOOL             1
#define DR_WORLD            2
#define DR_TC_USER_MIN      101
#define DR_TC_USER_MAX      200

// move mod
#define DR_MV_MOD_ABS       0
#define DR_MV_MOD_REL       1

// move reaction
#define DR_MV_RA_NONE       0
#define DR_MV_RA_DUPLICATE  0
#define DR_MV_RA_OVERRIDE   1

// move command type
#define DR_MV_COMMAND_NORM  0

// movesx velocity
#define DR_MVS_VEL_NONE     0
#define DR_MVS_VEL_CONST    1

// motion state
#define DR_STATE_IDLE       0
#define DR_STATE_INIT       1
#define DR_STATE_BUSY       2
#define DR_STATE_BLEND      3
#define DR_STATE_ACC        4
#define DR_STATE_CRZ        5
#define DR_STATE_DEC        6

// axis
#define DR_AXIS_X           0
#define DR_AXIS_Y           1
#define DR_AXIS_Z           2
#define DR_AXIS_A          10
#define DR_AXIS_B          11
#define DR_AXIS_C          12

// collision sensitivity
#define DR_COLSENS_DEFAULT 20
#define DR_COLSENS_MIN      1   
#define DR_COLSENS_MAX    300

// speed
#define DR_OP_SPEED_MIN     1
#define DR_OP_SPEED_MAX   100

// stop
#define DR_QSTOP_STO        0
#define DR_QSTOP            1
#define DR_SSTOP            2
#define DR_HOLD             3

#define DR_STOP_FIRST       DR_QSTOP_STO
#define DR_STOP_LAST        DR_HOLD

// condition
#define DR_COND_NONE        -10000

// digital I/O
#define DR_DIO_MIN_INDEX    1
#define DR_DIO_MAX_INDEX    20 // Input IO :16 -> 20 , Output IO : 16   

// tool digital I/O
#define DR_TDIO_MIN_INDEX   1
#define DR_TDIO_MAX_INDEX   6

// I/O value
#define ON                  1
#define OFF                 0

// Analog I/O mode
#define DR_ANALOG_CURRENT   0
#define DR_ANALOG_VOLTAGE   1

// modbus type
#define DR_MODBUS_DIG_INPUT     0
#define DR_MODBUS_DIG_OUTPUT    1
#define DR_MODBUS_REG_INPUT     2
#define DR_MODBUS_REG_OUTPUT    3
#define DR_DISCRETE_INPUT       0
#define DR_COIL                 1
#define DR_INPUT_REGISTER       2
#define DR_HOLDING_REGISTER     3

#define DR_MODBUS_ACCESS_MAX    32
#define DR_MAX_MODBUS_NAME_SIZE 32

// tp_popup pm_type
#define DR_PM_MESSAGE           0
#define DR_PM_WARNING           1
#define DR_PM_ALARM             2

// tp_get_user_input type
#define DR_VAR_INT              0
#define DR_VAR_FLOAT            1
#define DR_VAR_STR              2
#define DR_VAR_BOOL             3   

// len
#define DR_VELJ_DT_LEN          6
#define DR_ACCJ_DT_LEN          6

#define DR_VELX_DT_LEN          2
#define DR_ACCX_DT_LEN          2

#define DR_ANGLE_DT_LEN         2
#define DR_COG_DT_LEN           3
#define DR_WEIGHT_DT_LEN        3
#define DR_VECTOR_DT_LEN        3
#define DR_ST_DT_LEN            6
#define DR_FD_DT_LEN            6
#define DR_DIR_DT_LEN           6
#define DR_INERTIA_DT_LEN       6
#define DR_VECTOR_U1_LEN        3
#define DR_VECTOR_V1_LEN        3

#define DR_AVOID                0
#define DR_TASK_STOP            1

#define DR_FIFO                 0
#define DR_LIFO                 1

#define DR_FC_MOD_ABS           0
#define DR_FC_MOD_REL           1

#define DR_GLOBAL_VAR_TYPE_BOOL         0
#define DR_GLOBAL_VAR_TYPE_INT          1
#define DR_GLOBAL_VAR_TYPE_FLOAT        2
#define DR_GLOBAL_VAR_TYPE_STR          3
#define DR_GLOBAL_VAR_TYPE_POSJ         4
#define DR_GLOBAL_VAR_TYPE_POSX         5
#define DR_GLOBAL_VAR_TYPE_UNKNOWN      6

#define DR_IE_SLAVE_GPR_ADDR_START      0
#define DR_IE_SLAVE_GPR_ADDR_END       23
#define DR_IE_SLAVE_GPR_ADDR_END_BIT   63

#define DR_DPOS                         0
#define DR_DVEL                         1

#define DR_HOME_TARGET_MECHANIC         0
#define DR_HOME_TARGET_USER             1

#define DR_MV_ORI_TEACH                 0    
#define DR_MV_ORI_FIXED                 1    
#define DR_MV_ORI_RADIAL                2    

#define DR_MV_APP_NONE                  0
#define DR_MV_APP_WELD                  1
//________________________________________________________

typedef struct {
    int	    nLevel;         // INFO =1, WARN =2, ERROR =3 
    int	    nGroup;         // SYSTEM =1, MOTION =2, TP =3, INVERTER =4, SAFETY_CONTROLLER =5   
    int	    nCode;          // error code 
    char    strMsg1[MAX_STRING_SIZE];   // error msg 1
    char    strMsg2[MAX_STRING_SIZE];   // error msg 2
    char    strMsg3[MAX_STRING_SIZE];   // error msg 3
} DR_ERROR, *LPDR_ERROR;

typedef struct {
    int     nRobotState;
    char    strRobotState[MAX_SYMBOL_SIZE];
    float   fCurrentPosj[NUM_JOINT];
    float   fCurrentPosx[NUM_TASK];
    float   fCurrentToolPosx[NUM_TASK];

    int     nActualMode;
    int     nActualSpace;
    
    float   fJointAbs[NUM_JOINT];
    float   fJointErr[NUM_JOINT];
    float   fTargetPosj[NUM_JOINT];
    float   fTargetVelj[NUM_JOINT];
    float   fCurrentVelj[NUM_JOINT];

    float   fTaskErr[NUM_TASK];
    float   fTargetPosx[NUM_TASK];
    float   fTargetVelx[NUM_TASK];
    float   fCurrentVelx[NUM_TASK];
    int     nSolutionSpace;
    float   fRotationMatrix[3][3];

    float   fDynamicTor[NUM_JOINT];
    float   fActualJTS[NUM_JOINT];
    float   fActualEJT[NUM_JOINT];
    float   fActualETT[NUM_JOINT];

    double  dSyncTime;
    int     nActualBK[NUM_JOINT];
    int     nActualBT[NUM_BUTTON];
    float   fActualMC[NUM_JOINT];
    float   fActualMT[NUM_JOINT];
    bool    bCtrlBoxDigitalOutput[16];
    bool    bCtrlBoxDigitalInput[16];
    bool    bFlangeDigitalOutput[6];
    bool    bFlangeDigitalInput[6];

    int     nRegCount;
    string  strModbusSymbol[100];
    int     nModbusValue[100];
  
    int     nAccessControl;
    bool    bHommingCompleted;
    bool    bTpInitialized;
    bool    bMasteringNeed;
    bool    bDrlStopped;
    bool    bDisconnected;

    //--- The following variables have been updated since version M2.50 or higher. ---
	//ROBOT_MONITORING_WORLD
	float   fActualW2B[6];
	float   fCurrentPosW[2][6];
	float   fCurrentVelW[6];
	float   fWorldETT[6];
	float   fTargetPosW[6];
	float   fTargetVelW[6];
	float   fRotationMatrixWorld[3][3];

	//ROBOT_MONITORING_USER
	int     iActualUCN;
	int     iParent;
	float   fCurrentPosU[2][6];
	float   fCurrentVelU[6];
	float   fUserETT[6];
	float   fTargetPosU[6];
	float   fTargetVelU[6];
	float   fRotationMatrixUser[3][3];

    //READ_CTRLIO_INPUT_EX
	float   fActualAI[6];
	bool    bActualSW[3];
	bool    bActualSI[2];
	int     iActualAT[2];

	//READ_CTRLIO_OUTPUT_EX
	float   fTargetAO[2];
	int     iTargetAT[2];

	//READ_ENCODER_INPUT
	bool    bActualES[2];
	int     iActualED[2];
	bool    bActualER[2];
    //---------------------------------------------------------------------------------

} DR_STATE, *LPDR_STATE;
std::string m_name;
std::string m_host;
std::string m_mode;
std::string m_model;
std::string m_gripper;
std::string m_mobile;
std::string m_rt_host;
unsigned int m_rate;
unsigned int m_standby;
bool m_command;
unsigned int m_port;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub_;
typedef struct _ROBOT_JOINT_DATA
{
    double cmd;
    double pos;
    double vel;
    double eff;
} ROBOT_JOINT_DATA, *LPROBOT_JOINT_DATA;
using namespace DRAFramework;
using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
namespace dsr_hardware2{

    class HARDWARE_INTERFACE_PUBLIC DRHWInterface : public hardware_interface::SystemInterface
    {
    public:
        int m_nVersionDRCF;
        bool m_bCommand_;
        std::array<float, NUM_JOINT> m_fCmd_;
        CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
        ~DRHWInterface();

    protected:
        /// The size of this vector is (standard_interfaces_.size() x nr_joints)
        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocities_command_;
        std::vector<double> joint_effort_command_; /* not used*/
        std::vector<double> pre_joint_position_command_;
        std::vector<double> joint_position_;
        std::vector<double> joint_velocities_;
        std::vector<double> joint_effort_; /* not used*/

        std::vector<double> ft_states_;
        std::vector<double> ft_command_;
        std::vector<std::vector<float>> init_joint_position_command;

        std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
            {"position", {}}, {"velocity", {}}, {"effort", {}}};
        
        std::unordered_map<std::string, std::vector<std::string>> joint_comm_interfaces = {
            {"position", {}}, {"velocity", {}}, {"effort", {}}};
    };
}
    class DSRInterface : public rclcpp::Node
    {
    public:
        DSRInterface();
        virtual ~DSRInterface();

        int MsgPublisher_RobotState();
        int MsgPublisher_JointState();

        ///int MsgPublisher_RobotError();  현재 미사용 : DRHWInterface::OnLogAlarm 에서 바로 퍼블리싱 함.
        static void OnHommingCompletedCB();
        static void OnProgramStoppedCB(const PROGRAM_STOP_CAUSE iStopCause);
        static void OnMonitoringDataCB(const LPMONITORING_DATA pData);
        static void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData);
        static void OnTpInitializingCompletedCB();

        static void OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO);
        static void OnMonitoringCtrlIOExCB (const LPMONITORING_CTRLIO_EX pCtrlIO);
        static void OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus);
        static void OnMonitoringStateCB(const ROBOT_STATE eState);
        static void OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl);
        static void OnLogAlarm(LPLOG_ALARM pLogAlarm);
        void DSRInterfaceNode();

        std::string GetRobotName();
        std::string GetRobotModel();

    private:
        
        // rclcpp::Node::SharedPtr private_nh_;	

        std::string m_strRobotName;
        std::string m_strRobotModel;
        std::string m_strRobotGripper;

        //----- Service ---------------------------------------------------------------
        //----- System
        rclcpp::Service<dsr_msgs2::srv::SetRobotMode>::SharedPtr            m_nh_srv_set_robot_mode;    
        rclcpp::Service<dsr_msgs2::srv::GetRobotMode>::SharedPtr            m_nh_srv_get_robot_mode;
        rclcpp::Service<dsr_msgs2::srv::SetRobotSystem>::SharedPtr          m_nh_srv_set_robot_system;
        rclcpp::Service<dsr_msgs2::srv::GetRobotSystem>::SharedPtr          m_nh_srv_get_robot_system;
        rclcpp::Service<dsr_msgs2::srv::GetRobotState>::SharedPtr           m_nh_srv_get_robot_state;
        rclcpp::Service<dsr_msgs2::srv::SetRobotSpeedMode>::SharedPtr       m_nh_srv_set_robot_speed_mode;
        rclcpp::Service<dsr_msgs2::srv::GetRobotSpeedMode>::SharedPtr       m_nh_srv_get_robot_speed_mode;
        rclcpp::Service<dsr_msgs2::srv::GetCurrentPose>::SharedPtr          m_nh_srv_get_current_pose;
        rclcpp::Service<dsr_msgs2::srv::SetSafeStopResetType>::SharedPtr    m_nh_srv_set_safe_stop_reset_type;
        rclcpp::Service<dsr_msgs2::srv::GetLastAlarm>::SharedPtr            m_nh_srv_get_last_alarm;

        //----- MOTION
        rclcpp::Service<dsr_msgs2::srv::MoveJoint>::SharedPtr               m_nh_srv_move_joint;
        rclcpp::Service<dsr_msgs2::srv::MoveLine>::SharedPtr                m_nh_srv_move_line;
        rclcpp::Service<dsr_msgs2::srv::MoveJointx>::SharedPtr              m_nh_srv_move_jointx;
        rclcpp::Service<dsr_msgs2::srv::MoveCircle>::SharedPtr              m_nh_srv_move_circle;
        rclcpp::Service<dsr_msgs2::srv::MoveSplineJoint>::SharedPtr         m_nh_srv_move_spline_joint;
        rclcpp::Service<dsr_msgs2::srv::MoveSplineTask>::SharedPtr          m_nh_srv_move_spline_task;
        rclcpp::Service<dsr_msgs2::srv::MoveBlending>::SharedPtr            m_nh_srv_move_blending;
        rclcpp::Service<dsr_msgs2::srv::MoveSpiral>::SharedPtr              m_nh_srv_move_spiral;
        rclcpp::Service<dsr_msgs2::srv::MovePeriodic>::SharedPtr            m_nh_srv_move_periodic;
        rclcpp::Service<dsr_msgs2::srv::MoveWait>::SharedPtr                m_nh_srv_move_wait;
        rclcpp::Service<dsr_msgs2::srv::Jog>::SharedPtr                     m_nh_srv_jog;
        rclcpp::Service<dsr_msgs2::srv::JogMulti>::SharedPtr                m_nh_srv_jog_multi;
        rclcpp::Service<dsr_msgs2::srv::MoveStop>::SharedPtr                m_nh_srv_move_stop;
        rclcpp::Service<dsr_msgs2::srv::MoveResume>::SharedPtr              m_nh_srv_move_resume;
        rclcpp::Service<dsr_msgs2::srv::MovePause>::SharedPtr               m_nh_srv_move_pause;
        rclcpp::Service<dsr_msgs2::srv::Trans>::SharedPtr                   m_nh_srv_trans;
        rclcpp::Service<dsr_msgs2::srv::Fkin>::SharedPtr                    m_nh_srv_fkin;
        rclcpp::Service<dsr_msgs2::srv::Ikin>::SharedPtr                    m_nh_srv_ikin;
        rclcpp::Service<dsr_msgs2::srv::SetRefCoord>::SharedPtr             m_nh_srv_set_ref_coord;
        rclcpp::Service<dsr_msgs2::srv::MoveHome>::SharedPtr                m_nh_srv_move_home;
        rclcpp::Service<dsr_msgs2::srv::CheckMotion>::SharedPtr             m_nh_srv_check_motion;
        rclcpp::Service<dsr_msgs2::srv::ChangeOperationSpeed>::SharedPtr    m_nh_srv_change_operation_speed;
        rclcpp::Service<dsr_msgs2::srv::EnableAlterMotion>::SharedPtr       m_nh_srv_enable_alter_motion;
        rclcpp::Service<dsr_msgs2::srv::AlterMotion>::SharedPtr             m_nh_srv_alter_motion;
        rclcpp::Service<dsr_msgs2::srv::DisableAlterMotion>::SharedPtr      m_nh_srv_disable_alter_motion;
        rclcpp::Service<dsr_msgs2::srv::SetSingularityHandling>::SharedPtr  m_nh_srv_set_singularity_handling;

        //----- auxiliary_control
        rclcpp::Service<dsr_msgs2::srv::GetControlMode>::SharedPtr               m_nh_srv_get_control_mode;          
        rclcpp::Service<dsr_msgs2::srv::GetControlSpace>::SharedPtr              m_nh_srv_get_control_space;         

        rclcpp::Service<dsr_msgs2::srv::GetCurrentPosj>::SharedPtr               m_nh_srv_get_current_posj;          
        rclcpp::Service<dsr_msgs2::srv::GetCurrentVelj>::SharedPtr               m_nh_srv_get_current_velj;          
        rclcpp::Service<dsr_msgs2::srv::GetDesiredPosj>::SharedPtr               m_nh_srv_get_desired_posj;
        rclcpp::Service<dsr_msgs2::srv::GetDesiredVelj>::SharedPtr               m_nh_srv_get_desired_velj;          

        rclcpp::Service<dsr_msgs2::srv::GetCurrentPosx>::SharedPtr               m_nh_srv_get_current_posx;          
        rclcpp::Service<dsr_msgs2::srv::GetCurrentVelx>::SharedPtr               m_nh_srv_get_current_velx;          
        rclcpp::Service<dsr_msgs2::srv::GetDesiredPosx>::SharedPtr               m_nh_srv_get_desired_posx;
        rclcpp::Service<dsr_msgs2::srv::GetDesiredVelx>::SharedPtr               m_nh_srv_get_desired_velx;          

        rclcpp::Service<dsr_msgs2::srv::GetCurrentToolFlangePosx>::SharedPtr     m_nh_srv_get_current_tool_flange_posx;
        rclcpp::Service<dsr_msgs2::srv::GetCurrentSolutionSpace>::SharedPtr      m_nh_srv_get_current_solution_space; 
        rclcpp::Service<dsr_msgs2::srv::GetCurrentRotm>::SharedPtr               m_nh_srv_get_current_rotm;          
        rclcpp::Service<dsr_msgs2::srv::GetJointTorque>::SharedPtr               m_nh_srv_get_joint_torque;          
        rclcpp::Service<dsr_msgs2::srv::GetExternalTorque>::SharedPtr            m_nh_srv_get_external_torque;      
        rclcpp::Service<dsr_msgs2::srv::GetToolForce>::SharedPtr                 m_nh_srv_get_tool_force;            
        rclcpp::Service<dsr_msgs2::srv::GetSolutionSpace>::SharedPtr             m_nh_srv_get_solution_space;
        rclcpp::Service<dsr_msgs2::srv::GetOrientationError>::SharedPtr          m_nh_srv_get_orientation_error;

        //----- force/stiffness
        rclcpp::Service<dsr_msgs2::srv::ParallelAxis1>::SharedPtr                m_nh_srv_parallel_axis1;
        rclcpp::Service<dsr_msgs2::srv::ParallelAxis2>::SharedPtr                m_nh_srv_parallel_axis2;
        rclcpp::Service<dsr_msgs2::srv::AlignAxis1>::SharedPtr                   m_nh_srv_align_axis1;
        rclcpp::Service<dsr_msgs2::srv::AlignAxis2>::SharedPtr                   m_nh_srv_align_axis2;
        rclcpp::Service<dsr_msgs2::srv::IsDoneBoltTightening>::SharedPtr         m_nh_srv_is_done_bolt_tightening;
        rclcpp::Service<dsr_msgs2::srv::ReleaseComplianceCtrl>::SharedPtr        m_nh_srv_release_compliance_ctrl;
        rclcpp::Service<dsr_msgs2::srv::TaskComplianceCtrl>::SharedPtr           m_nh_srv_task_compliance_ctrl;
        rclcpp::Service<dsr_msgs2::srv::SetStiffnessx>::SharedPtr                m_nh_srv_set_stiffnessx;
        rclcpp::Service<dsr_msgs2::srv::CalcCoord>::SharedPtr                    m_nh_srv_calc_coord;
        rclcpp::Service<dsr_msgs2::srv::SetUserCartCoord1>::SharedPtr            m_nh_srv_set_user_cart_coord1;
        rclcpp::Service<dsr_msgs2::srv::SetUserCartCoord2>::SharedPtr            m_nh_srv_set_user_cart_coord2;
        rclcpp::Service<dsr_msgs2::srv::SetUserCartCoord3>::SharedPtr            m_nh_srv_set_user_cart_coord3;
        rclcpp::Service<dsr_msgs2::srv::OverwriteUserCartCoord>::SharedPtr       m_nh_srv_overwrite_user_cart_coord;
        rclcpp::Service<dsr_msgs2::srv::GetUserCartCoord>::SharedPtr             m_nh_srv_get_user_cart_coord;
        rclcpp::Service<dsr_msgs2::srv::SetDesiredForce>::SharedPtr              m_nh_srv_set_desired_force;
        rclcpp::Service<dsr_msgs2::srv::ReleaseForce>::SharedPtr                 m_nh_srv_release_force;
        rclcpp::Service<dsr_msgs2::srv::CheckPositionCondition>::SharedPtr       m_nh_srv_check_position_condition;
        rclcpp::Service<dsr_msgs2::srv::CheckForceCondition>::SharedPtr          m_nh_srv_check_force_condition;
        rclcpp::Service<dsr_msgs2::srv::CheckOrientationCondition1>::SharedPtr   m_nh_srv_check_orientation_condition1;
        rclcpp::Service<dsr_msgs2::srv::CheckOrientationCondition2>::SharedPtr   m_nh_srv_check_orientation_condition2;
        rclcpp::Service<dsr_msgs2::srv::CoordTransform>::SharedPtr               m_nh_srv_coord_transform;
        rclcpp::Service<dsr_msgs2::srv::GetWorkpieceWeight>::SharedPtr           m_nh_srv_get_workpiece_weight;
        rclcpp::Service<dsr_msgs2::srv::ResetWorkpieceWeight>::SharedPtr         m_nh_srv_reset_workpiece_weight;

        //----- TCP
        rclcpp::Service<dsr_msgs2::srv::SetCurrentTcp>::SharedPtr                m_nh_srv_set_current_tcp; 
        rclcpp::Service<dsr_msgs2::srv::GetCurrentTcp>::SharedPtr                m_nh_srv_get_current_tcp; 
        rclcpp::Service<dsr_msgs2::srv::ConfigCreateTcp>::SharedPtr              m_nh_srv_config_create_tcp; 
        rclcpp::Service<dsr_msgs2::srv::ConfigDeleteTcp>::SharedPtr              m_nh_srv_config_delete_tcp; 

        //----- TOOL
        rclcpp::Service<dsr_msgs2::srv::SetCurrentTool>::SharedPtr               m_nh_srv_set_current_tool; 
        rclcpp::Service<dsr_msgs2::srv::GetCurrentTool>::SharedPtr               m_nh_srv_get_current_tool; 
        rclcpp::Service<dsr_msgs2::srv::ConfigCreateTool>::SharedPtr             m_nh_srv_config_create_tool; 
        rclcpp::Service<dsr_msgs2::srv::ConfigDeleteTool>::SharedPtr             m_nh_srv_config_delete_tool; 
        rclcpp::Service<dsr_msgs2::srv::SetToolShape>::SharedPtr                 m_nh_srv_set_tool_shape; 

        //----- IO
        rclcpp::Service<dsr_msgs2::srv::SetCtrlBoxDigitalOutput>::SharedPtr      m_nh_srv_set_ctrl_box_digital_output; 
        rclcpp::Service<dsr_msgs2::srv::GetCtrlBoxDigitalOutput>::SharedPtr      m_nh_srv_get_ctrl_box_digital_output; 
        rclcpp::Service<dsr_msgs2::srv::GetCtrlBoxDigitalInput>::SharedPtr       m_nh_srv_get_ctrl_box_digital_input; 

        rclcpp::Service<dsr_msgs2::srv::SetToolDigitalOutput>::SharedPtr         m_nh_srv_set_tool_digital_output; 
        rclcpp::Service<dsr_msgs2::srv::GetToolDigitalOutput>::SharedPtr         m_nh_srv_get_tool_digital_output; 
        rclcpp::Service<dsr_msgs2::srv::GetToolDigitalInput>::SharedPtr          m_nh_srv_get_tool_digital_input; 

        rclcpp::Service<dsr_msgs2::srv::SetCtrlBoxAnalogOutput>::SharedPtr       m_nh_srv_set_ctrl_box_analog_output; 
        rclcpp::Service<dsr_msgs2::srv::GetCtrlBoxAnalogInput>::SharedPtr        m_nh_srv_get_ctrl_box_analog_input; 
        rclcpp::Service<dsr_msgs2::srv::SetCtrlBoxAnalogOutputType>::SharedPtr   m_nh_srv_set_ctrl_box_analog_output_type; 
        rclcpp::Service<dsr_msgs2::srv::SetCtrlBoxAnalogInputType>::SharedPtr    m_nh_srv_set_ctrl_box_analog_input_type; 

        //----- MODBUS
        rclcpp::Service<dsr_msgs2::srv::SetModbusOutput>::SharedPtr              m_nh_srv_set_modbus_output; 
        rclcpp::Service<dsr_msgs2::srv::GetModbusInput>::SharedPtr               m_nh_srv_get_modbus_input; 
        rclcpp::Service<dsr_msgs2::srv::ConfigCreateModbus>::SharedPtr           m_nh_srv_config_create_modbus; 
        rclcpp::Service<dsr_msgs2::srv::ConfigDeleteModbus>::SharedPtr           m_nh_srv_config_delete_modbus; 

        //----- DRL        
        rclcpp::Service<dsr_msgs2::srv::DrlPause>::SharedPtr                     m_nh_srv_drl_pause; 
        rclcpp::Service<dsr_msgs2::srv::DrlStart>::SharedPtr                     m_nh_srv_drl_start; 
        rclcpp::Service<dsr_msgs2::srv::DrlStop>::SharedPtr                      m_nh_srv_drl_stop; 
        rclcpp::Service<dsr_msgs2::srv::DrlResume>::SharedPtr                    m_nh_srv_drl_resume; 
        rclcpp::Service<dsr_msgs2::srv::GetDrlState>::SharedPtr                  m_nh_srv_get_drl_state; 

        //----- Gripper
        rclcpp::Service<dsr_msgs2::srv::Robotiq2FOpen>::SharedPtr                m_nh_srv_robotiq2_f_open; 
        rclcpp::Service<dsr_msgs2::srv::Robotiq2FClose>::SharedPtr               m_nh_srv_robotiq2_f_close; 
        rclcpp::Service<dsr_msgs2::srv::Robotiq2FMove>::SharedPtr                m_nh_srv_robotiq2_f_move;

        //----- Serial
        rclcpp::Service<dsr_msgs2::srv::SerialSendData>::SharedPtr               m_nh_srv_serial_send_data;


        //----- Publisher -------------------------------------------------------------
        rclcpp::Publisher<dsr_msgs2::msg::RobotState>::SharedPtr            m_PubRobotState;
        ///rclcpp::Publisher<dsr_msgs2::msg::RobotError>::SharedPtr            m_PubRobotError;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      m_PubtoGazebo;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 m_PubSerialWrite;
        ///TODO rclcpp::Publisher<dsr_msgs::msg::JogMultiAxis>::SharedPtr  m_PubJogMultiAxis;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr/*rclcpp::Node::SharedPtr*/ m_PubJointState;  //add for TEST
        
        //----- Subscriber ------------------------------------------------------------
        rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr m_sub_joint_trajectory;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr/*rclcpp::Node::SharedPtr*/ m_sub_test;
        


        void trajectoryCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg) const;

        void testSubCallback(const std_msgs::msg::String::SharedPtr msg) const;

        //----- Threads ------------------------------------------------------------------
        boost::thread m_th_subscribe;   //subscribe thread
        boost::thread m_th_publisher;   //publisher thread
        boost::thread m_th_publisher_joint_states;   //publisher thread

        static void thread_subscribe(rclcpp::Node::SharedPtr nh);
        static void thread_publisher(DSRInterface* pDSRInterface, rclcpp::Node::SharedPtr nh, int nPubRate);
        static void thread_publisher_direct_access_joint_states(DSRInterface* pDSRInterface, rclcpp::Node::SharedPtr nh, int nPubRate);// '/joint_state'에 직접 퍼블리싱하는 함수(현재 미사용) 
                                                                                                                                         // ros2_control를 통해서 퍼블리싱 함. 

        DR_STATE m_stDrState;
        DR_ERROR m_stDrError;

        //----- Service Call-back functions ----------------------------------------------.

        //----- System
        static bool set_robot_mode_cb(const std::shared_ptr<dsr_msgs2::srv::SetRobotMode::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotMode::Response> res);
        static bool get_robot_mode_cb(const std::shared_ptr<dsr_msgs2::srv::GetRobotMode::Request> req, std::shared_ptr<dsr_msgs2::srv::GetRobotMode::Response> res);      
        static bool set_robot_system_cb(const std::shared_ptr<dsr_msgs2::srv::SetRobotSystem::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotSystem::Response> res);         
        static bool get_robot_system_cb(const std::shared_ptr<dsr_msgs2::srv::GetRobotSystem::Request> req, std::shared_ptr<dsr_msgs2::srv::GetRobotSystem::Response> res);         
        static bool get_robot_state_cb(const std::shared_ptr<dsr_msgs2::srv::GetRobotState::Request> req, std::shared_ptr<dsr_msgs2::srv::GetRobotState::Response> res);        
        static bool set_robot_speed_mode_cb(const std::shared_ptr<dsr_msgs2::srv::SetRobotSpeedMode::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotSpeedMode::Response> res);    
        static bool get_robot_speed_mode_cb(const std::shared_ptr<dsr_msgs2::srv::GetRobotSpeedMode::Request> req, std::shared_ptr<dsr_msgs2::srv::GetRobotSpeedMode::Response> res);       
        static bool get_current_pose_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentPose::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentPose::Response> res);   
        static bool set_safe_stop_reset_type_cb(const std::shared_ptr<dsr_msgs2::srv::SetSafeStopResetType::Request> req, std::shared_ptr<dsr_msgs2::srv::SetSafeStopResetType::Response> res);     
        static bool get_last_alarm_cb(const std::shared_ptr<dsr_msgs2::srv::GetLastAlarm::Request> req, std::shared_ptr<dsr_msgs2::srv::GetLastAlarm::Response> res);   

        //----- MOTION
        static bool movej_cb(const std::shared_ptr<dsr_msgs2::srv::MoveJoint::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveJoint::Response> res);
        static bool movel_cb(const std::shared_ptr<dsr_msgs2::srv::MoveLine::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveLine::Response> res);
        static bool movejx_cb(const std::shared_ptr<dsr_msgs2::srv::MoveJointx::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveJointx::Response> res);               
        static bool movec_cb(const std::shared_ptr<dsr_msgs2::srv::MoveCircle::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveCircle::Response> res);       
        static bool movesj_cb(const std::shared_ptr<dsr_msgs2::srv::MoveSplineJoint::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveSplineJoint::Response> res);      
        static bool movesx_cb(const std::shared_ptr<dsr_msgs2::srv::MoveSplineTask::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveSplineTask::Response> res);          
        static bool moveb_cb(const std::shared_ptr<dsr_msgs2::srv::MoveBlending::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveBlending::Response> res);          
        static bool movespiral_cb(const std::shared_ptr<dsr_msgs2::srv::MoveSpiral::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveSpiral::Response> res);              
        static bool moveperiodic_cb(const std::shared_ptr<dsr_msgs2::srv::MovePeriodic::Request> req, std::shared_ptr<dsr_msgs2::srv::MovePeriodic::Response> res);              

        static bool movewait_cb(const std::shared_ptr<dsr_msgs2::srv::MoveWait::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveWait::Response> res);                  
        static bool jog_cb(const std::shared_ptr<dsr_msgs2::srv::Jog::Request> req, std::shared_ptr<dsr_msgs2::srv::Jog::Response> res);              
        static bool jog_multi_cb(const std::shared_ptr<dsr_msgs2::srv::JogMulti::Request> req, std::shared_ptr<dsr_msgs2::srv::JogMulti::Response> res);                      
        static bool move_stop_cb(const std::shared_ptr<dsr_msgs2::srv::MoveStop::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveStop::Response> res);                  
        static bool move_resume_cb(const std::shared_ptr<dsr_msgs2::srv::MoveResume::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveResume::Response> res);                  
        static bool move_pause_cb(const std::shared_ptr<dsr_msgs2::srv::MovePause::Request> req, std::shared_ptr<dsr_msgs2::srv::MovePause::Response> res);                      
        static bool trans_cb(const std::shared_ptr<dsr_msgs2::srv::Trans::Request> req, std::shared_ptr<dsr_msgs2::srv::Trans::Response> res);                  
        static bool fkin_cb(const std::shared_ptr<dsr_msgs2::srv::Fkin::Request> req, std::shared_ptr<dsr_msgs2::srv::Fkin::Response> res);              
        static bool ikin_cb(const std::shared_ptr<dsr_msgs2::srv::Ikin::Request> req, std::shared_ptr<dsr_msgs2::srv::Ikin::Response> res);              
        static bool set_ref_coord_cb(const std::shared_ptr<dsr_msgs2::srv::SetRefCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRefCoord::Response> res);                  
        static bool move_home_cb(const std::shared_ptr<dsr_msgs2::srv::MoveHome::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveHome::Response> res);                  
        static bool check_motion_cb(const std::shared_ptr<dsr_msgs2::srv::CheckMotion::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckMotion::Response> res);                  
        static bool change_operation_speed_cb(const std::shared_ptr<dsr_msgs2::srv::ChangeOperationSpeed::Request> req, std::shared_ptr<dsr_msgs2::srv::ChangeOperationSpeed::Response> res);        
        static bool enable_alter_motion_cb(const std::shared_ptr<dsr_msgs2::srv::EnableAlterMotion::Request> req, std::shared_ptr<dsr_msgs2::srv::EnableAlterMotion::Response> res);                 
        static bool alter_motion_cb(const std::shared_ptr<dsr_msgs2::srv::AlterMotion::Request> req, std::shared_ptr<dsr_msgs2::srv::AlterMotion::Response> res);              
        static bool disable_alter_motion_cb(const std::shared_ptr<dsr_msgs2::srv::DisableAlterMotion::Request> req, std::shared_ptr<dsr_msgs2::srv::DisableAlterMotion::Response> res);              
        static bool set_singularity_handling_cb(const std::shared_ptr<dsr_msgs2::srv::SetSingularityHandling::Request> req, std::shared_ptr<dsr_msgs2::srv::SetSingularityHandling::Response> res);  

        //----- auxiliary_control
        static bool get_control_mode_cb(const std::shared_ptr<dsr_msgs2::srv::GetControlMode::Request> req, std::shared_ptr<dsr_msgs2::srv::GetControlMode::Response> res);                           
        static bool get_control_space_cb(const std::shared_ptr<dsr_msgs2::srv::GetControlSpace::Request> req, std::shared_ptr<dsr_msgs2::srv::GetControlSpace::Response> res);                          

        static bool get_current_posj_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentPosj::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentPosj::Response> res);                                            
        static bool get_current_velj_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentVelj::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentVelj::Response> res);                                
        static bool get_desired_posj_cb(const std::shared_ptr<dsr_msgs2::srv::GetDesiredPosj::Request> req, std::shared_ptr<dsr_msgs2::srv::GetDesiredPosj::Response> res);         
        static bool get_desired_velj_cb(const std::shared_ptr<dsr_msgs2::srv::GetDesiredVelj::Request> req, std::shared_ptr<dsr_msgs2::srv::GetDesiredVelj::Response> res);                                   

        static bool get_current_posx_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentPosx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentPosx::Response> res);                               
        static bool get_current_velx_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentVelx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentVelx::Response> res);                         
        static bool get_desired_posx_cb(const std::shared_ptr<dsr_msgs2::srv::GetDesiredPosx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetDesiredPosx::Response> res);     
        static bool get_desired_velx_cb(const std::shared_ptr<dsr_msgs2::srv::GetDesiredVelx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetDesiredVelx::Response> res);                             

        static bool get_current_tool_flange_posx_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentToolFlangePosx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentToolFlangePosx::Response> res);                 
        static bool get_current_solution_space_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentSolutionSpace::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentSolutionSpace::Response> res);                    
        static bool get_current_rotm_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentRotm::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentRotm::Response> res);                     
        static bool get_joint_torque_cb(const std::shared_ptr<dsr_msgs2::srv::GetJointTorque::Request> req, std::shared_ptr<dsr_msgs2::srv::GetJointTorque::Response> res);                     
        static bool get_external_torque_cb(const std::shared_ptr<dsr_msgs2::srv::GetExternalTorque::Request> req, std::shared_ptr<dsr_msgs2::srv::GetExternalTorque::Response> res);                
        static bool get_tool_force_cb(const std::shared_ptr<dsr_msgs2::srv::GetToolForce::Request> req, std::shared_ptr<dsr_msgs2::srv::GetToolForce::Response> res);                               
        static bool get_solution_space_cb(const std::shared_ptr<dsr_msgs2::srv::GetSolutionSpace::Request> req, std::shared_ptr<dsr_msgs2::srv::GetSolutionSpace::Response> res);       
        static bool get_orientation_error_cb(const std::shared_ptr<dsr_msgs2::srv::GetOrientationError::Request> req, std::shared_ptr<dsr_msgs2::srv::GetOrientationError::Response> res);              

        //----- force/stiffness
        static bool parallel_axis1_cb(const std::shared_ptr<dsr_msgs2::srv::ParallelAxis1::Request> req, std::shared_ptr<dsr_msgs2::srv::ParallelAxis1::Response> res);  
        static bool parallel_axis2_cb(const std::shared_ptr<dsr_msgs2::srv::ParallelAxis2::Request> req, std::shared_ptr<dsr_msgs2::srv::ParallelAxis2::Response> res);  
        static bool align_axis1_cb(const std::shared_ptr<dsr_msgs2::srv::AlignAxis1::Request> req, std::shared_ptr<dsr_msgs2::srv::AlignAxis1::Response> res);          
        static bool align_axis2_cb(const std::shared_ptr<dsr_msgs2::srv::AlignAxis2::Request> req, std::shared_ptr<dsr_msgs2::srv::AlignAxis2::Response> res);      
        static bool is_done_bolt_tightening_cb(const std::shared_ptr<dsr_msgs2::srv::IsDoneBoltTightening::Request> req, std::shared_ptr<dsr_msgs2::srv::IsDoneBoltTightening::Response> res);      
        static bool release_compliance_ctrl_cb(const std::shared_ptr<dsr_msgs2::srv::ReleaseComplianceCtrl::Request> req, std::shared_ptr<dsr_msgs2::srv::ReleaseComplianceCtrl::Response> res);         
        static bool task_compliance_ctrl_cb(const std::shared_ptr<dsr_msgs2::srv::TaskComplianceCtrl::Request> req, std::shared_ptr<dsr_msgs2::srv::TaskComplianceCtrl::Response> res);          
        static bool set_stiffnessx_cb(const std::shared_ptr<dsr_msgs2::srv::SetStiffnessx::Request> req, std::shared_ptr<dsr_msgs2::srv::SetStiffnessx::Response> res);          
        static bool calc_coord_cb(const std::shared_ptr<dsr_msgs2::srv::CalcCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::CalcCoord::Response> res);      
        static bool set_user_cart_coord1_cb(const std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord1::Request> req, std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord1::Response> res);          
        static bool set_user_cart_coord2_cb(const std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord2::Request> req, std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord2::Response> res);              
        static bool set_user_cart_coord3_cb(const std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord3::Request> req, std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord3::Response> res);      
        static bool overwrite_user_cart_coord_cb(const std::shared_ptr<dsr_msgs2::srv::OverwriteUserCartCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::OverwriteUserCartCoord::Response> res);      
        static bool get_user_cart_coord_cb(const std::shared_ptr<dsr_msgs2::srv::GetUserCartCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::GetUserCartCoord::Response> res);          
        static bool set_desired_force_cb(const std::shared_ptr<dsr_msgs2::srv::SetDesiredForce::Request> req, std::shared_ptr<dsr_msgs2::srv::SetDesiredForce::Response> res);          
        static bool release_force_cb(const std::shared_ptr<dsr_msgs2::srv::ReleaseForce::Request> req, std::shared_ptr<dsr_msgs2::srv::ReleaseForce::Response> res);      
        static bool check_position_condition_cb(const std::shared_ptr<dsr_msgs2::srv::CheckPositionCondition::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckPositionCondition::Response> res);          
        static bool check_force_condition_cb(const std::shared_ptr<dsr_msgs2::srv::CheckForceCondition::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckForceCondition::Response> res);      
        static bool check_orientation_condition1_cb(const std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition1::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition1::Response> res);             
        static bool check_orientation_condition2_cb(const std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition2::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition2::Response> res);            
        static bool coord_transform_cb(const std::shared_ptr<dsr_msgs2::srv::CoordTransform::Request> req, std::shared_ptr<dsr_msgs2::srv::CoordTransform::Response> res);          
        static bool get_workpiece_weight_cb(const std::shared_ptr<dsr_msgs2::srv::GetWorkpieceWeight::Request> req, std::shared_ptr<dsr_msgs2::srv::GetWorkpieceWeight::Response> res);          
        static bool reset_workpiece_weight_cb(const std::shared_ptr<dsr_msgs2::srv::ResetWorkpieceWeight::Request> req, std::shared_ptr<dsr_msgs2::srv::ResetWorkpieceWeight::Response> res);          

        //----- TCP
        static bool set_current_tcp_cb(const std::shared_ptr<dsr_msgs2::srv::SetCurrentTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCurrentTcp::Response> res);       
        static bool get_current_tcp_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentTcp::Response> res);       
        static bool config_create_tcp_cb(const std::shared_ptr<dsr_msgs2::srv::ConfigCreateTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigCreateTcp::Response> res);    
        static bool config_delete_tcp_cb(const std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTcp::Response> res);  

        //----- TOOL
        static bool set_current_tool_cb(const std::shared_ptr<dsr_msgs2::srv::SetCurrentTool::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCurrentTool::Response> res);     
        static bool get_current_tool_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentTool::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentTool::Response> res);     
        static bool config_create_tool_cb(const std::shared_ptr<dsr_msgs2::srv::ConfigCreateTool::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigCreateTool::Response> res); 
        static bool config_delete_tool_cb(const std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTool::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTool::Response> res);    
        static bool set_tool_shape_cb(const std::shared_ptr<dsr_msgs2::srv::SetToolShape::Request> req, std::shared_ptr<dsr_msgs2::srv::SetToolShape::Response> res); 

        //----- IO
        static bool set_digital_output_cb(const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxDigitalOutput::Response> res);    
        static bool get_digital_output_cb(const std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalOutput::Response> res);   
        static bool get_digital_input_cb(const std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalInput::Response> res);   
        static bool set_tool_digital_output_cb(const std::shared_ptr<dsr_msgs2::srv::SetToolDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetToolDigitalOutput::Response> res);   
        static bool get_tool_digital_output_cb(const std::shared_ptr<dsr_msgs2::srv::GetToolDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetToolDigitalOutput::Response> res);   
        static bool get_tool_digital_input_cb(const std::shared_ptr<dsr_msgs2::srv::GetToolDigitalInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetToolDigitalInput::Response> res);   

        static bool set_analog_output_cb(const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutput::Response> res);   
        static bool get_analog_input_cb(const std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxAnalogInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxAnalogInput::Response> res);   
        static bool set_analog_output_type_cb(const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutputType::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutputType::Response> res);   
        static bool set_analog_input_type_cb(const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogInputType::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogInputType::Response> res);       

        //----- MODBUS
        static bool set_modbus_output_cb(const std::shared_ptr<dsr_msgs2::srv::SetModbusOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetModbusOutput::Response> res);    
        static bool get_modbus_input_cb(const std::shared_ptr<dsr_msgs2::srv::GetModbusInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetModbusInput::Response> res);    
        static bool config_create_modbus_cb(const std::shared_ptr<dsr_msgs2::srv::ConfigCreateModbus::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigCreateModbus::Response> res);
        static bool config_delete_modbus_cb(const std::shared_ptr<dsr_msgs2::srv::ConfigDeleteModbus::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigDeleteModbus::Response> res);

        //----- DRL        
        static bool drl_pause_cb(const std::shared_ptr<dsr_msgs2::srv::DrlPause::Request> req, std::shared_ptr<dsr_msgs2::srv::DrlPause::Response> res);                         
        static bool drl_start_cb(const std::shared_ptr<dsr_msgs2::srv::DrlStart::Request> req, std::shared_ptr<dsr_msgs2::srv::DrlStart::Response> res);    
        static bool drl_stop_cb(const std::shared_ptr<dsr_msgs2::srv::DrlStop::Request> req, std::shared_ptr<dsr_msgs2::srv::DrlStop::Response> res);    
        static bool drl_resume_cb(const std::shared_ptr<dsr_msgs2::srv::DrlResume::Request> req, std::shared_ptr<dsr_msgs2::srv::DrlResume::Response> res);        
        static bool get_drl_state_cb(const std::shared_ptr<dsr_msgs2::srv::GetDrlState::Request> req, std::shared_ptr<dsr_msgs2::srv::GetDrlState::Response> res);       

        //----- Gripper
        static bool robotiq_2f_open_cb(const std::shared_ptr<dsr_msgs2::srv::Robotiq2FOpen::Request> req, std::shared_ptr<dsr_msgs2::srv::Robotiq2FOpen::Response> res); 
        static bool robotiq_2f_close_cb(const std::shared_ptr<dsr_msgs2::srv::Robotiq2FClose::Request> req, std::shared_ptr<dsr_msgs2::srv::Robotiq2FClose::Response> res);
        static bool robotiq_2f_move_cb(const std::shared_ptr<dsr_msgs2::srv::Robotiq2FMove::Request> req, std::shared_ptr<dsr_msgs2::srv::Robotiq2FMove::Response> res);

        //----- Serial
        static bool serial_send_data_cb(const std::shared_ptr<dsr_msgs2::srv::SerialSendData::Request> req, std::shared_ptr<dsr_msgs2::srv::SerialSendData::Response> res);
    
    };
#endif // end
