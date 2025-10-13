/*
 * dsr_controller2
 * Author: Minsoo Song (minsoo.song@doosan.com)
 *
 * Copyright (c) 2024 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#ifndef DSR_CONTROLLER2__DSR_CONTROLLER2_HPP_
#define DSR_CONTROLLER2__DSR_CONTROLLER2_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// msg
#include <dsr_msgs2/msg/robot_error.hpp>
#include <dsr_msgs2/msg/robot_state.hpp>
#include <dsr_msgs2/msg/robot_stop.hpp>
#include <dsr_msgs2/msg/alter_motion_stream.hpp>
#include <dsr_msgs2/msg/servoj_stream.hpp>
#include <dsr_msgs2/msg/servol_stream.hpp>
#include <dsr_msgs2/msg/speedj_stream.hpp>
#include <dsr_msgs2/msg/speedl_stream.hpp>

#include <dsr_msgs2/msg/robot_state_rt.hpp>
#include <dsr_msgs2/msg/servoj_rt_stream.hpp>
#include <dsr_msgs2/msg/servol_rt_stream.hpp>
#include <dsr_msgs2/msg/speedj_rt_stream.hpp>
#include <dsr_msgs2/msg/speedl_rt_stream.hpp>
#include <dsr_msgs2/msg/torque_rt_stream.hpp>
#include <dsr_msgs2/msg/robot_error.hpp>
#include <dsr_msgs2/msg/robot_disconnection.hpp>



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
#include "dsr_msgs2/srv/servo_off.hpp"
#include "dsr_msgs2/srv/set_robot_control.hpp"
#include "dsr_msgs2/srv/change_collision_sensitivity.hpp"
#include "dsr_msgs2/srv/set_safety_mode.hpp"


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

//moveit


//RT
#include "dsr_msgs2/srv/connect_rt_control.hpp"
#include "dsr_msgs2/srv/disconnect_rt_control.hpp"
#include "dsr_msgs2/srv/get_rt_control_input_data_list.hpp"
#include "dsr_msgs2/srv/get_rt_control_input_version_list.hpp"
#include "dsr_msgs2/srv/get_rt_control_output_data_list.hpp"
#include "dsr_msgs2/srv/get_rt_control_output_version_list.hpp"
#include "dsr_msgs2/srv/read_data_rt.hpp"
#include "dsr_msgs2/srv/set_accj_rt.hpp"
#include "dsr_msgs2/srv/set_accx_rt.hpp"
#include "dsr_msgs2/srv/set_rt_control_input.hpp"
#include "dsr_msgs2/srv/set_rt_control_output.hpp"
#include "dsr_msgs2/srv/set_velj_rt.hpp"
#include "dsr_msgs2/srv/set_velx_rt.hpp"
#include "dsr_msgs2/srv/start_rt_control.hpp"
#include "dsr_msgs2/srv/stop_rt_control.hpp"
#include "dsr_msgs2/srv/write_data_rt.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"


#include "../../../dsr_common2/include/DRFLEx.h"

#define _DEBUG_DSR_CTL      0

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

typedef struct _ROBOT_JOINT_DATA
{
    double cmd;
    double pos;
    double vel;
    double eff;
} ROBOT_JOINT_DATA, *LPROBOT_JOINT_DATA;

std::string m_name;
std::string m_model;

bool g_bIsEmulatorMode = FALSE;
bool g_bHasControlAuthority = FALSE;
bool g_bTpInitailizingComplted = FALSE;
bool g_bHommingCompleted = FALSE;
bool init_state=TRUE;



namespace DRFL_CALLBACKS {
  void OnTpInitializingCompletedCB();
  void OnHommingCompletedCB();
  void OnProgramStoppedCB(const PROGRAM_STOP_CAUSE /*iStopCause*/);
  void OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO);
  void OnMonitoringCtrlIOExCB (const LPMONITORING_CTRLIO_EX pCtrlIO);
  void OnMonitoringDataCB(const LPMONITORING_DATA pData);
  void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData);
  void OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus);
  void OnMonitoringStateCB(const ROBOT_STATE eState);
  void OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl);
  void OnLogAlarm(LPLOG_ALARM pLogAlarm);
  void OnDisConnected();
}


namespace dsr_controller2
{
class RobotController : public controller_interface::ControllerInterface
{
public:
  CONTROLLER_INTERFACE_PUBLIC
  RobotController();

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  
  rclcpp::Publisher<dsr_msgs2::msg::RobotDisconnection>::SharedPtr disconnect_pub_;
  rclcpp::Publisher<dsr_msgs2::msg::RobotError>::SharedPtr error_log_pub_;
protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  // We may need to distinguish the callback group based on the traits of the services.
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  rclcpp::Subscription<dsr_msgs2::msg::AlterMotionStream>::SharedPtr   m_sub_alter_motion_stream;
  rclcpp::Subscription<dsr_msgs2::msg::ServojStream>::SharedPtr        m_sub_servoj_stream;
  rclcpp::Subscription<dsr_msgs2::msg::ServolStream>::SharedPtr        m_sub_servol_stream;
  rclcpp::Subscription<dsr_msgs2::msg::SpeedjStream>::SharedPtr        m_sub_speedj_stream;
  rclcpp::Subscription<dsr_msgs2::msg::SpeedlStream>::SharedPtr        m_sub_speedl_stream;

  rclcpp::Subscription<dsr_msgs2::msg::ServojRtStream>::SharedPtr      m_sub_servoj_rt_stream;
  rclcpp::Subscription<dsr_msgs2::msg::ServolRtStream>::SharedPtr      m_sub_servol_rt_stream;
  rclcpp::Subscription<dsr_msgs2::msg::SpeedjRtStream>::SharedPtr      m_sub_speedj_rt_stream;
  rclcpp::Subscription<dsr_msgs2::msg::SpeedlRtStream>::SharedPtr      m_sub_speedl_rt_stream;
  rclcpp::Subscription<dsr_msgs2::msg::TorqueRtStream>::SharedPtr      m_sub_torque_rt_stream;

  rclcpp::Service<dsr_msgs2::srv::GetRobotMode>::SharedPtr            m_nh_srv_get_robot_mode;
  rclcpp::Service<dsr_msgs2::srv::SetRobotMode>::SharedPtr            m_nh_srv_set_robot_mode;
  rclcpp::Service<dsr_msgs2::srv::SetRobotSystem>::SharedPtr          m_nh_srv_set_robot_system;
  rclcpp::Service<dsr_msgs2::srv::GetRobotSystem>::SharedPtr          m_nh_srv_get_robot_system;
  rclcpp::Service<dsr_msgs2::srv::GetRobotState>::SharedPtr           m_nh_srv_get_robot_state;
  rclcpp::Service<dsr_msgs2::srv::SetRobotSpeedMode>::SharedPtr       m_nh_srv_set_robot_speed_mode;
  rclcpp::Service<dsr_msgs2::srv::GetRobotSpeedMode>::SharedPtr       m_nh_srv_get_robot_speed_mode;
  rclcpp::Service<dsr_msgs2::srv::GetCurrentPose>::SharedPtr          m_nh_srv_get_current_pose;
  rclcpp::Service<dsr_msgs2::srv::SetSafeStopResetType>::SharedPtr    m_nh_srv_set_safe_stop_reset_type;
  rclcpp::Service<dsr_msgs2::srv::GetLastAlarm>::SharedPtr            m_nh_srv_get_last_alarm;
  rclcpp::Service<dsr_msgs2::srv::ServoOff>::SharedPtr                m_nh_srv_servo_off;
  rclcpp::Service<dsr_msgs2::srv::SetRobotControl>::SharedPtr         m_nh_srv_set_robot_control;
  rclcpp::Service<dsr_msgs2::srv::ChangeCollisionSensitivity>::SharedPtr m_nh_srv_change_collision_sensitivity;
  rclcpp::Service<dsr_msgs2::srv::SetSafetyMode>::SharedPtr           m_nh_srv_set_safety_mode;

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

  //----- RT
  rclcpp::Service<dsr_msgs2::srv::ConnectRtControl>::SharedPtr              m_nh_connect_rt_control;
  rclcpp::Service<dsr_msgs2::srv::DisconnectRtControl>::SharedPtr           m_nh_disconnect_rt_control;
  rclcpp::Service<dsr_msgs2::srv::GetRtControlOutputVersionList>::SharedPtr m_nh_get_rt_control_output_version_list;
  rclcpp::Service<dsr_msgs2::srv::GetRtControlInputVersionList>::SharedPtr  m_nh_get_rt_control_input_version_list;
  rclcpp::Service<dsr_msgs2::srv::GetRtControlInputDataList>::SharedPtr     m_nh_get_rt_control_input_data_list;
  rclcpp::Service<dsr_msgs2::srv::GetRtControlOutputDataList>::SharedPtr    m_nh_get_rt_control_output_data_list;
  rclcpp::Service<dsr_msgs2::srv::SetRtControlInput>::SharedPtr             m_nh_set_rt_control_input;
  rclcpp::Service<dsr_msgs2::srv::SetRtControlOutput>::SharedPtr            m_nh_set_rt_control_output;
  rclcpp::Service<dsr_msgs2::srv::StartRtControl>::SharedPtr                m_nh_start_rt_control;
  rclcpp::Service<dsr_msgs2::srv::StopRtControl>::SharedPtr                 m_nh_stop_rt_control;
  rclcpp::Service<dsr_msgs2::srv::SetVeljRt>::SharedPtr                     m_nh_set_velj_rt;
  rclcpp::Service<dsr_msgs2::srv::SetAccjRt>::SharedPtr                     m_nh_set_accj_rt;
  rclcpp::Service<dsr_msgs2::srv::SetVelxRt>::SharedPtr                     m_nh_set_velx_rt;
  rclcpp::Service<dsr_msgs2::srv::SetAccxRt>::SharedPtr                     m_nh_set_accx_rt;
  rclcpp::Service<dsr_msgs2::srv::ReadDataRt>::SharedPtr                    m_nh_read_data_rt;
  rclcpp::Service<dsr_msgs2::srv::WriteDataRt>::SharedPtr                   m_nh_write_data_rt;

  // Real-time data publishing members and parameters for periodic Float32MultiArray topic output.
  bool use_rt_topic_pub_{false};
  std::vector<std::string> rt_topic_keys_;
  std::unordered_map<std::string,rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> rt_pub_map_;
  rclcpp::TimerBase::SharedPtr rt_timer_;
  void publish_read_data_rt_selected();
  static bool extract_field(LPRT_OUTPUT_DATA_LIST temp,const std::string& key,std::vector<float>& out);

private:
  static constexpr const char* PARAM_USE_RT_TOPIC_PUB = "use_rt_topic_pub";
  static constexpr const char* PARAM_RT_TOPIC_KEYS    = "rt_topic_keys";
  static constexpr const char* PARAM_RT_TIMER_MS      = "rt_timer_ms";
};
}  // namespace dsr_controller2

#endif  
