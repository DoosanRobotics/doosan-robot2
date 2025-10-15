/*
 * dsr_controller2
 * Author: Minsoo Song (minsoo.song@doosan.com)
 *
 * Copyright (c) 2024 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/


#include "dsr_controller2/dsr_controller2.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "../../dsr_common2/include/DRFLEx.h"

using config_type = controller_interface::interface_configuration_type;
using namespace DRAFramework;

extern void* get_drfl();
CDRFLEx *Drfl = (CDRFLEx*)get_drfl();

DR_STATE g_stDrState;
DR_ERROR    g_stDrError;

extern bool g_bIsEmulatorMode;
extern std::string g_model;
int g_nAnalogOutputModeCh1;
int g_nAnalogOutputModeCh2;


dsr_controller2::RobotController *instance;

const char* GetRobotStateString(int nState)
{
    switch(nState)
    {
    case STATE_INITIALIZING:    return "(0) INITIALIZING";
    case STATE_STANDBY:         return "(1) STANDBY";
    case STATE_MOVING:          return "(2) MOVING";
    case STATE_SAFE_OFF:        return "(3) SAFE_OFF";
    case STATE_TEACHING:        return "(4) TEACHING";
    case STATE_SAFE_STOP:       return "(5) SAFE_STOP";
    case STATE_EMERGENCY_STOP:  return "(6) EMERGENCY_STOP";
    case STATE_HOMMING:         return "(7) HOMMING";
    case STATE_RECOVERY:        return "(8) RECOVERY";
    case STATE_SAFE_STOP2:      return "(9) SAFE_STOP2";
    case STATE_SAFE_OFF2:       return "(10) SAFE_OFF2";
    case STATE_RESERVED1:       return "(11) RESERVED1";
    case STATE_RESERVED2:       return "(12) RESERVED2";
    case STATE_RESERVED3:       return "(13) RESERVED3";
    case STATE_RESERVED4:       return "(14) RESERVED4";
    case STATE_NOT_READY:       return "(15) NOT_READY";

    default:                  return "UNKNOWN";
    }
    return "UNKNOWN";
}

namespace dsr_controller2
{
RobotController::RobotController() : controller_interface::ControllerInterface() {}


controller_interface::CallbackReturn RobotController::on_init()
{
    instance = this;
    m_model = g_model;

    use_rt_topic_pub_ = auto_declare<bool>(PARAM_USE_RT_TOPIC_PUB, false);
    auto rt_ms        = auto_declare<int>(PARAM_RT_TIMER_MS, 10);

    if (!get_node()->has_parameter(PARAM_RT_TOPIC_KEYS)) {get_node()->declare_parameter<std::vector<std::string>>(PARAM_RT_TOPIC_KEYS,std::vector<std::string>{});}
    rt_topic_keys_ = get_node()->get_parameter(PARAM_RT_TOPIC_KEYS).as_string_array();

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};


  return conf;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
  return conf;
}

controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
{
	//--- doosan API's call-back fuctions : Only work within 50msec in call-back functions
	Drfl->set_on_tp_initializing_completed(DRFL_CALLBACKS::OnTpInitializingCompletedCB);
	Drfl->set_on_homming_completed(DRFL_CALLBACKS::OnHommingCompletedCB);
	Drfl->set_on_program_stopped(DRFL_CALLBACKS::OnProgramStoppedCB);
	Drfl->set_on_monitoring_modbus(DRFL_CALLBACKS::OnMonitoringModbusCB);
	Drfl->set_on_monitoring_data(DRFL_CALLBACKS::OnMonitoringDataCB);           // Callback function in M2.4 and earlier
	Drfl->set_on_monitoring_ctrl_io(DRFL_CALLBACKS::OnMonitoringCtrlIOCB);       // Callback function in M2.4 and earlier
	Drfl->set_on_monitoring_state(DRFL_CALLBACKS::OnMonitoringStateCB);//RELATED TO LOGIC
	Drfl->set_on_monitoring_access_control(DRFL_CALLBACKS::OnMonitoringAccessControlCB);//RELATED TO LOGIC
	Drfl->set_on_log_alarm(DRFL_CALLBACKS::OnLogAlarm);
	Drfl->set_on_disconnected(DRFL_CALLBACKS::OnDisConnected);
	Drfl->set_on_monitoring_data_ex(DRFL_CALLBACKS::OnMonitoringDataExCB);

    // create publishers by key
    if (use_rt_topic_pub_) {
        rt_pub_map_.clear();
        for (const auto& key : rt_topic_keys_) 
        {
        auto topic = "/rt_topic/" + key; // topic name
        rt_pub_map_[key] = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>(topic, rclcpp::SystemDefaultsQoS());
        }

        // timer
        const int rt_ms = get_node()->get_parameter(PARAM_RT_TIMER_MS).as_int();
        rt_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(rt_ms),std::bind(&RobotController::publish_read_data_rt_selected, this));
    }

  return CallbackReturn::SUCCESS;
}

// Publishes selected real-time robot data fields as Float32MultiArray messages.
void RobotController::publish_read_data_rt_selected() {
  LPRT_OUTPUT_DATA_LIST temp = Drfl->read_data_rt();
  if (!temp) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "read_data_rt() returned nullptr");
    return;
  }

  for (const auto& key : rt_topic_keys_) {
    auto it = rt_pub_map_.find(key);
    if (it == rt_pub_map_.end()) continue;

    std::vector<float> vals;
    if (!extract_field(temp, key, vals)) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, "read_data_rt(): unsupported key '%s'", key.c_str());
      continue;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data = std::move(vals); // using raw data
    it->second->publish(msg);
  }
}

// Extracts a specific field from the real-time data structure (LPRT_OUTPUT_DATA_LIST)
bool RobotController::extract_field(LPRT_OUTPUT_DATA_LIST temp, const std::string& key, std::vector<float>& out)
{
    out.clear();

    // Helper lambdas for copying arrays and scalars of various data types
    auto copy_arr_f = [&](const float* src, size_t n) {
        out.assign(src, src + n);
        return true;
    };
    auto copy_arr_d = [&](const double* src, size_t n) {
        out.resize(n);
        for (size_t i = 0; i < n; ++i)
            out[i] = static_cast<float>(src[i]);
        return true;
    };
    auto copy_arr_i32 = [&](const int32_t* src, size_t n) {
        out.resize(n);
        for (size_t i = 0; i < n; ++i)
            out[i] = static_cast<float>(src[i]);
        return true;
    };
    auto copy_arr_u32 = [&](const uint32_t* src, size_t n) {
        out.resize(n);
        for (size_t i = 0; i < n; ++i)
            out[i] = static_cast<float>(src[i]);
        return true;
    };
    auto copy_arr_u8 = [&](const unsigned char* src, size_t n) {
        out.resize(n);
        for (size_t i = 0; i < n; ++i)
            out[i] = static_cast<float>(src[i]);
        return true;
    };

    auto copy_scalar_d = [&](double v) {
        out = { static_cast<float>(v) };
        return true;
    };
    auto copy_scalar_i32 = [&](int32_t v) {
        out = { static_cast<float>(v) };
        return true;
    };
    auto copy_scalar_u32 = [&](uint32_t v) {
        out = { static_cast<float>(v) };
        return true;
    };

    auto copy_mat6x6_f = [&](const float m[6][6]) {
        out.resize(36);
        size_t k = 0;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                out[k++] = m[i][j];
        return true;
    };
    auto copy_mat6x6_d = [&](const double m[6][6]) {
        out.resize(36);
        size_t k = 0;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                out[k++] = static_cast<float>(m[i][j]);
        return true;
    };

    // 6-element float array fields ---
    if (key == "actual_joint_position")        return copy_arr_f(temp->actual_joint_position, 6);
    if (key == "actual_joint_position_abs")    return copy_arr_f(temp->actual_joint_position_abs, 6);
    if (key == "actual_joint_velocity")        return copy_arr_f(temp->actual_joint_velocity, 6);
    if (key == "actual_joint_velocity_abs")    return copy_arr_f(temp->actual_joint_velocity_abs, 6);
    if (key == "actual_tcp_position")          return copy_arr_f(temp->actual_tcp_position, 6);
    if (key == "actual_tcp_velocity")          return copy_arr_f(temp->actual_tcp_velocity, 6);
    if (key == "actual_flange_position")       return copy_arr_f(temp->actual_flange_position, 6);
    if (key == "actual_flange_velocity")       return copy_arr_f(temp->actual_flange_velocity, 6);
    if (key == "actual_motor_torque")          return copy_arr_f(temp->actual_motor_torque, 6);
    if (key == "actual_joint_torque")          return copy_arr_f(temp->actual_joint_torque, 6);
    if (key == "raw_joint_torque")             return copy_arr_f(temp->raw_joint_torque, 6);
    if (key == "raw_force_torque")             return copy_arr_f(temp->raw_force_torque, 6);
    if (key == "external_joint_torque")        return copy_arr_f(temp->external_joint_torque, 6);
    if (key == "external_tcp_force")           return copy_arr_f(temp->external_tcp_force, 6);
    if (key == "target_joint_position")        return copy_arr_f(temp->target_joint_position, 6);
    if (key == "target_joint_velocity")        return copy_arr_f(temp->target_joint_velocity, 6);
    if (key == "target_joint_acceleration")    return copy_arr_f(temp->target_joint_acceleration, 6);
    if (key == "target_motor_torque")          return copy_arr_f(temp->target_motor_torque, 6);
    if (key == "target_tcp_position")          return copy_arr_f(temp->target_tcp_position, 6);
    if (key == "target_tcp_velocity")          return copy_arr_f(temp->target_tcp_velocity, 6);
    if (key == "gravity_torque")               return copy_arr_f(temp->gravity_torque, 6);
    if (key == "joint_temperature")            return copy_arr_f(temp->joint_temperature, 6);
    if (key == "goal_joint_position")          return copy_arr_f(temp->goal_joint_position, 6);
    if (key == "goal_tcp_position")            return copy_arr_f(temp->goal_tcp_position, 6);

    // 6x6 matrix fields
    if (key == "coriolis_matrix")  return copy_mat6x6_f(temp->coriolis_matrix);
    if (key == "mass_matrix")      return copy_mat6x6_f(temp->mass_matrix);
    if (key == "jacobian_matrix")  return copy_mat6x6_f(temp->jacobian_matrix);

    // 2-element fields (controller and encoder info)
    if (key == "controller_analog_input_type")   return copy_arr_u8(temp->controller_analog_input_type,   2);
    if (key == "controller_analog_input")        return copy_arr_f (temp->controller_analog_input,        2);
    if (key == "controller_analog_output_type")  return copy_arr_u8(temp->controller_analog_output_type,  2);
    if (key == "controller_analog_output")       return copy_arr_f (temp->controller_analog_output,       2);
    if (key == "external_encoder_strobe_count")  return copy_arr_u8(temp->external_encoder_strobe_count,  2);
    if (key == "external_encoder_count")         return copy_arr_u32(temp->external_encoder_count,        2);

    // 4-element fields (flange analog input)
    if (key == "flange_analog_input")            return copy_arr_f (temp->flange_analog_input, 4);

    // Single scalar values
    if (key == "time_stamp")               return copy_scalar_d  (temp->time_stamp);
    if (key == "solution_space")           return copy_scalar_i32(temp->solution_space);
    if (key == "singularity")              return copy_scalar_i32(temp->singularity);
    if (key == "operation_speed_rate")     return copy_scalar_d  (temp->operation_speed_rate);
    if (key == "controller_digital_input") return copy_scalar_u32(temp->controller_digital_input);
    if (key == "controller_digital_output")return copy_scalar_u32(temp->controller_digital_output);
    if (key == "flange_digital_input")     return copy_scalar_u32(temp->flange_digital_input);
    if (key == "flange_digital_output")    return copy_scalar_u32(temp->flange_digital_output);
    if (key == "robot_mode")               return copy_scalar_i32(temp->robot_mode);
    if (key == "robot_state")              return copy_scalar_i32(temp->robot_state);
    if (key == "control_mode")             return copy_scalar_i32(temp->control_mode);

    return false;
}

void check_dsr_model(std::array<float, NUM_JOINT>& target_joint){
    if (m_model == "p3020"){
        if (3 < target_joint.size() && target_joint[3] != 0.0) {
            RCLCPP_WARN(rclcpp::get_logger("dsr_controller2"),
                        "p3020 is a 5-axis model, so the value of target_pos[3] cannot be specified by (%.6f).", target_joint[3]);
            target_joint[3] = 0.0;
        }
    }
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
{
      
auto set_robot_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetRobotMode::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotMode::Response> res) -> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_robot_mode_cb() called and calling Drfl->set_robot_mode(%d)",req->robot_mode);

    res->success = Drfl->set_robot_mode((ROBOT_MODE)req->robot_mode); 
};

auto get_robot_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRobotMode::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetRobotMode::Response> res)-> void
{       
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_robot_mode_cb() called and calling Drfl->get_robot_mode()");
    res->robot_mode = Drfl->get_robot_mode();
    res->success = true;
};

auto set_robot_system_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetRobotSystem::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotSystem::Response> res)-> void
{ 
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_robot_system_cb() called and calling Drfl->set_robot_system(%d)",req->robot_system);

    res->success = Drfl->set_robot_system((ROBOT_SYSTEM)req->robot_system);
};        
auto get_robot_system_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRobotSystem::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetRobotSystem::Response> res)-> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_robot_system_cb() called and calling Drfl->get_robot_system()");

    res->robot_system = Drfl->get_robot_system();
    res->success = true;
};

auto get_robot_state_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRobotState::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetRobotState::Response> res)-> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_robot_state_cb() called and calling Drfl->get_robot_state()");

    res->robot_state = Drfl->get_robot_state();
    res->success = true;
};

auto set_robot_speed_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetRobotSpeedMode::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotSpeedMode::Response> res)-> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_robot_speed_mode_cb() called and calling Drfl->set_robot_speed_mode(%d)",req->speed_mode);

    res->success = Drfl->set_robot_speed_mode((SPEED_MODE)req->speed_mode);
};

auto get_robot_speed_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRobotSpeedMode::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetRobotSpeedMode::Response> res)-> void 
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_robot_speed_mode_cb() called and calling Drfl->get_robot_speed_mode()");

    res->speed_mode = Drfl->get_robot_speed_mode();
    res->success = true;
};

auto get_current_pose_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentPose::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentPose::Response> res)-> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_current_pose_cb() called and calling Drfl->get_current_pose(%d)",req->space_type);

    LPROBOT_POSE robot_pos = Drfl->get_current_pose((ROBOT_SPACE)req->space_type);
    for(int i = 0; i < NUM_TASK; i++){
        res->pos[i] = robot_pos->_fPosition[i];
    }
    res->success = true;
};

auto set_safe_stop_reset_type_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetSafeStopResetType::Request> req, std::shared_ptr<dsr_msgs2::srv::SetSafeStopResetType::Response> res)-> void   
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_safe_stop_reset_type_cb() called and calling Drfl->SetSafeStopResetType(%d)",req->reset_type);
    Drfl->SetSafeStopResetType((SAFE_STOP_RESET_TYPE)req->reset_type);
    res->success = true;                           
};

auto get_last_alarm_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetLastAlarm::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetLastAlarm::Response> res)-> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_last_alarm_cb() called and calling Drfl->get_last_alarm()");
    res->log_alarm.level = Drfl->get_last_alarm()->_iLevel;
    res->log_alarm.group = Drfl->get_last_alarm()->_iGroup;
    res->log_alarm.index = Drfl->get_last_alarm()->_iIndex;
    for(int i = 0; i < 3; i++){
        std::string str_temp(Drfl->get_last_alarm()->_szParam[i]);
        res->log_alarm.param[i] = str_temp;
    }
    res->success = true;
};

auto servo_off_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ServoOff::Request> req, std::shared_ptr<dsr_msgs2::srv::ServoOff::Response> res) -> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"servo_off_cb() called and calling Drfl->servo_off()");
    Drfl->servo_off((STOP_TYPE)req->stop_type);
    res->success = true;
};

auto set_robot_control_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetRobotControl::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotControl::Response> res) -> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_robot_control_cb() called and calling Drfl->set_robot_control_cb()");
    Drfl->set_robot_control((ROBOT_CONTROL)req->robot_control);
    res->success = true;
};

auto change_collision_sensitivity_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ChangeCollisionSensitivity::Request> req, std::shared_ptr<dsr_msgs2::srv::ChangeCollisionSensitivity::Response> res)-> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"change_collision_sensitivity_cb() called and calling Drfl->change_collision_sensitivity_cb()");
    Drfl->change_collision_sensitivity((float)req->sensitivity);
    res->success = true;
};

auto set_safety_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetSafetyMode::Request> req, std::shared_ptr<dsr_msgs2::srv::SetSafetyMode::Response> res) -> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_safety_mode() called and calling Drfl->set_safety_mode()");
    Drfl->set_safety_mode((SAFETY_MODE)req->safety_mode, (SAFETY_MODE_EVENT)req->safety_event);
    res->success = true;
};

//----- MOTION Service Call-back functions ------------------------------------------------------------
auto movej_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveJoint::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveJoint::Response> res)-> void
{   
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movej_cb() %p", (void*) Drfl);
    res->success = false;
    std::array<float, NUM_JOINT> target_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
    check_dsr_model(target_pos);
    if(req->sync_type == 0){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movej_cb() called and calling Drfl->movej");
        res->success = Drfl->movej(target_pos.data(), req->vel, req->acc, req->time, (MOVE_MODE)req->mode, req->radius, (BLENDING_SPEED_TYPE)req->blend_type);   
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movej_cb() called and calling Drfl->amovej");
        res->success = Drfl->amovej(target_pos.data(), req->vel, req->acc, req->time, (MOVE_MODE)req->mode, (BLENDING_SPEED_TYPE)req->blend_type);
    }
};

auto movel_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveLine::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveLine::Response> res)-> void
{
    res->success = false;
    std::array<float, NUM_JOINT> target_pos;
    std::array<float, 2> target_vel;
    std::array<float, 2> target_acc;
    std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
    std::copy(req->vel.cbegin(), req->vel.cend(), target_vel.begin());
    std::copy(req->acc.cbegin(), req->acc.cend(), target_acc.begin());
    // check_dsr_model(target_pos);
    if(req->sync_type == 0){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movel_cb() called and calling Drfl->movel");
        res->success = Drfl->movel(target_pos.data(), target_vel.data(), target_acc.data(), req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, req->radius, (BLENDING_SPEED_TYPE)req->blend_type);   
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movel_cb() called and calling Drfl->amovel");
        res->success = Drfl->amovel(target_pos.data(), target_vel.data(), target_acc.data(), req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (BLENDING_SPEED_TYPE)req->blend_type);
    }
};

auto movejx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveJointx::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveJointx::Response> res)-> void            
{
    std::array<float, NUM_TASK> target_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
    check_dsr_model(target_pos);
    if(req->sync_type == 0){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movejx_cb() called and calling Drfl->movejx");
        res->success = Drfl->movejx(target_pos.data(), req->sol, req->vel, req->acc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, req->radius, (BLENDING_SPEED_TYPE)req->blend_type);    
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movejx_cb() called and calling Drfl->amovejx");
        res->success = Drfl->amovejx(target_pos.data(), req->sol, req->vel, req->acc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (BLENDING_SPEED_TYPE)req->blend_type);    
    }
};

auto movec_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveCircle::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveCircle::Response> res)-> void    
{
    float fTargetPos[2][NUM_TASK];
    float fTargetVel[2];
    float fTargetAcc[2];
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < NUM_TASK; j++){
            std_msgs::msg::Float64MultiArray pos = req->pos.at(i);
            fTargetPos[i][j] = pos.data[j];
        }
        std::array<float, NUM_JOINT> target_pos;
        std::copy(std::begin(fTargetPos[i]), std::end(fTargetPos[i]), target_pos.begin());
        check_dsr_model(target_pos);
        std::copy(target_pos.begin(), target_pos.end(), std::begin(fTargetPos[i]));
        
        fTargetVel[i] = req->vel[i];
        fTargetAcc[i] = req->acc[i];
    }
    ///RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"  <xxx pos1> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[0][0],fTargetPos[0][1],fTargetPos[0][2],fTargetPos[0][3],fTargetPos[0][4],fTargetPos[0][5]);
    ///RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"  <xxx pos2> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[1][0],fTargetPos[1][1],fTargetPos[1][2],fTargetPos[1][3],fTargetPos[1][4],fTargetPos[1][5]);
    if(req->sync_type == 0){   
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movec_cb() called and calling Drfl->movec");
        res->success = Drfl->movec(fTargetPos, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, req->angle1, req->angle2, req->radius, (BLENDING_SPEED_TYPE)req->blend_type);      
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movec_cb() called and calling Drfl->amovec");
        res->success = Drfl->amovec(fTargetPos, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, req->angle1, req->angle2, (BLENDING_SPEED_TYPE)req->blend_type);
    }
};


auto movesj_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveSplineJoint::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveSplineJoint::Response> res)-> void     
{
    res->success = false;
    float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT];
    float fTargetVel[NUM_JOINT]= {0.0, };
    float fTargetAcc[NUM_JOINT]= {0.0, };

    for(int i=0; i<req->pos_cnt; i++){
        for(int j=0; j<NUM_JOINT; j++){
            std_msgs::msg::Float64MultiArray pos = req->pos.at(i);
            fTargetPos[i][j] = pos.data[j];
            fTargetVel[j] = req->vel[j];
            fTargetAcc[j] = req->acc[j];
        }
        std::array<float, NUM_JOINT> target_pos;
        std::copy(std::begin(fTargetPos[i]), std::end(fTargetPos[i]), target_pos.begin());
        check_dsr_model(target_pos);
        std::copy(target_pos.begin(), target_pos.end(), std::begin(fTargetPos[i]));
        
    }
    
    std::array<float, NUM_JOINT> target_vel;
    std::array<float, NUM_JOINT> target_acc;
    std::copy(std::begin(fTargetVel), std::end(fTargetVel), target_vel.begin());
    std::copy(std::begin(fTargetAcc), std::end(fTargetAcc), target_acc.begin());
    check_dsr_model(target_vel);
    check_dsr_model(target_acc);
    std::copy(target_vel.begin(), target_vel.end(), std::begin(fTargetVel));
    std::copy(target_acc.begin(), target_acc.end(), std::begin(fTargetAcc));


    if(req->sync_type == 0){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movejx_cb() called and calling Drfl->movesj");
        //res->success = Drfl->MoveSJ(fTargetPos, req->pos_cnt, fTargetVel, req->acc, req->time, (MOVE_MODE)req->mode);
        res->success = Drfl->movesj(fTargetPos, req->pos_cnt, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode); //need updata API
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movejx_cb() called and calling Drfl->amovesj");
        //res->success = Drfl->MoveSJAsync(fTargetPos, req->pos_cnt, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode);
        res->success = Drfl->amovesj(fTargetPos, req->pos_cnt, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode); //need updata API
    }
};

auto movesx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveSplineTask::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveSplineTask::Response> res)-> void       
{
    res->success = false;
    float fTargetPos[MAX_SPLINE_POINT][NUM_TASK];
    float fTargetVel[2];
    float fTargetAcc[2];

    for(int i=0; i<req->pos_cnt; i++){
        for(int j=0; j<NUM_TASK; j++){
            std_msgs::msg::Float64MultiArray pos = req->pos.at(i);
            fTargetPos[i][j] = pos.data[j];
        }
      //  fTargetVel[i] = req->vel[i];
      //  fTargetAcc[i] = req->acc[i];
    }
    for(int i=0; i<2; i++){
        fTargetVel[i] = req->vel[i];
        fTargetAcc[i] = req->acc[i];
    }
    if(req->sync_type == 0){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movesx_cb() called and calling Drfl->movesx");
        res->success = Drfl->movesx(fTargetPos, req->pos_cnt, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (SPLINE_VELOCITY_OPTION)req->opt);
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movesx_cb() called and calling Drfl->amovesx");
        res->success = Drfl->amovesx(fTargetPos, req->pos_cnt, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (SPLINE_VELOCITY_OPTION)req->opt);
    }
};

auto moveb_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveBlending::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveBlending::Response> res)-> void    
{
    res->success = false;
    MOVE_POSB posb[req->pos_cnt];
    for(int i=0; i<req->pos_cnt; i++){
        std_msgs::msg::Float64MultiArray segment = req->segment.at(i);
        for(int j=0; j<NUM_TASK; j++){
            posb[i]._fTargetPos[0][j] = segment.data[j];            //0~5
            posb[i]._fTargetPos[1][j] = segment.data[j + NUM_TASK]; //6~11
        }
        posb[i]._iBlendType = segment.data[NUM_TASK + NUM_TASK];    //12
        posb[i]._fBlendRad = segment.data[NUM_TASK + NUM_TASK +1];  //13
    }

    /*
    for(int i=0; i<req->pos_cnt; i++){
        printf("----- segment %d -----\n",i);
        printf("    pos1: %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n"
                ,posb[i]._fTargetPos[0][0], posb[i]._fTargetPos[0][1], posb[i]._fTargetPos[0][2], posb[i]._fTargetPos[0][3], posb[i]._fTargetPos[0][4], posb[i]._fTargetPos[0][5]);

        printf("    pos2: %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n"
                ,posb[i]._fTargetPos[1][0], posb[i]._fTargetPos[1][1], posb[i]._fTargetPos[1][2], posb[i]._fTargetPos[1][3], posb[i]._fTargetPos[1][4], posb[i]._fTargetPos[1][5]);

        printf("    posb[%d]._iblend_type = %d\n",i,posb[i]._iblend_type); 
        printf("    posb[%d]._fBlendRad  = %f\n",i,posb[i]._fBlendRad); 
    }
    */

    std::array<float, 2> target_vel;
    std::array<float, 2> target_acc;
    std::copy(req->vel.cbegin(), req->vel.cend(), target_vel.begin());
    std::copy(req->acc.cbegin(), req->acc.cend(), target_acc.begin());

    if(req->sync_type == 0){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"moveb_cb() called and calling Drfl->moveb");
        res->success = Drfl->moveb(posb, req->pos_cnt, target_vel.data(), target_acc.data(), req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref);
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"moveb_cb() called and calling Drfl->amoveb");
        res->success = Drfl->amoveb(posb, req->pos_cnt, target_vel.data(), target_acc.data(), req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref);
    }
};

auto movespiral_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveSpiral::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveSpiral::Response> res)-> void          
{
    res->success = false;
    std::array<float, 3> target_pos;
    std::array<float, 2> target_vel;
    std::array<float, 2> target_acc;
    std::copy(req->target_pos.cbegin(), req->target_pos.cend(), target_pos.begin());
    std::copy(req->vel.cbegin(), req->vel.cend(), target_vel.begin());
    std::copy(req->acc.cbegin(), req->acc.cend(), target_acc.begin());

    bool use_spiral_ex = std::any_of(req->target_pos.cbegin(), req->target_pos.cend(), [](float v){ return v != 0.0; });

    if(req->sync_type == 0){
        if (use_spiral_ex){
            RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movespiral_cb() called and calling Drfl->move_spiral_ex");
            res->success = Drfl->move_spiral(
                (TASK_AXIS)req->task_axis,
                req->revolution,
                target_pos.data(),
                target_vel.data(),
                target_acc.data(),
                req->time,
                (MOVE_REFERENCE)req->ref,
                (MOVE_MODE)req->mode,
                (SPIRAL_DIR)req->spiral_dir,
                (ROT_DIR)req->rot_dir
            );
        }
        else {
            RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movespiral_cb() called and calling Drfl->move_spiral");
            res->success = Drfl->move_spiral(
                (TASK_AXIS)req->task_axis,
                req->revolution,
                req->max_radius,
                req->max_length,
                target_vel.data(),
                target_acc.data(),
                req->time,
                (MOVE_REFERENCE)req->ref
            );
        }
    }
    else{
        if (use_spiral_ex){
            RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movespiral_cb() called and calling Drfl->amove_spiral_ex");
            res->success = Drfl->amove_spiral(
                (TASK_AXIS)req->task_axis,
                req->revolution,
                target_pos.data(),
                target_vel.data(),
                target_acc.data(),
                req->time,
                (MOVE_REFERENCE)req->ref,
                (MOVE_MODE)req->mode,
                (SPIRAL_DIR)req->spiral_dir,
                (ROT_DIR)req->rot_dir
            );
        }
        else {
            RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movespiral_cb() called and calling Drfl->amove_spiral");
            res->success = Drfl->amove_spiral(
                (TASK_AXIS)req->task_axis,
                req->revolution,
                req->max_radius,
                req->max_length,
                target_vel.data(),
                target_acc.data(),
                req->time,
                (MOVE_REFERENCE)req->ref
            );
        }
    }
};

auto moveperiodic_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MovePeriodic::Request> req, std::shared_ptr<dsr_msgs2::srv::MovePeriodic::Response> res)-> void          
{
    res->success = false;
    std::array<float, NUM_TASK> target_amp;
    std::array<float, NUM_TASK> target_periodic;
    std::copy(req->amp.cbegin(), req->amp.cend(), target_amp.begin());
    std::copy(req->periodic.cbegin(), req->periodic.cend(), target_periodic.begin());
    if(req->sync_type == 0){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"moveperiodic_cb() called and calling Drfl->move_periodic");
        res->success = Drfl->move_periodic(target_amp.data(), target_periodic.data(), req->acc, req->repeat, (MOVE_REFERENCE)req->ref);
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"moveperiodic_cb() called and calling Drfl->amove_periodic");
        res->success = Drfl->amove_periodic(target_amp.data(), target_periodic.data(), req->acc, req->repeat, (MOVE_REFERENCE)req->ref);
    }
};

auto movewait_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveWait::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::MoveWait::Response> res)-> void                
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movewait_cb() called and calling Drfl->mwait");
    res->success = Drfl->mwait();
};

auto jog_cb = [this](const std::shared_ptr<dsr_msgs2::srv::Jog::Request> req, std::shared_ptr<dsr_msgs2::srv::Jog::Response> res)-> void        
{
    if (m_model == "p3020" && req->jog_axis == JOG_AXIS_JOINT_4){
        RCLCPP_WARN(rclcpp::get_logger("dsr_controller2"),"The p3020 model does not support control of the 4th axis.");
        res->success = FALSE; 
    }

    res->success = Drfl->jog((JOG_AXIS)req->jog_axis, (MOVE_REFERENCE)req->move_reference, req->speed);
};


auto jog_multi_cb = [this](const std::shared_ptr<dsr_msgs2::srv::JogMulti::Request> req, std::shared_ptr<dsr_msgs2::srv::JogMulti::Response> res)-> void                    
{

    std::array<float, NUM_JOINT> target_jog;
    std::copy(req->jog_axis.cbegin(), req->jog_axis.cend(), target_jog.begin());
    check_dsr_model(target_jog);
    res->success = Drfl->multi_jog(target_jog.data(), (MOVE_REFERENCE)req->move_reference, req->speed);
};

auto move_stop_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveStop::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveStop::Response> res)-> void                  
{
    res->success = Drfl->stop((STOP_TYPE)req->stop_mode);
};

auto move_resume_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveResume::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::MoveResume::Response> res)-> void                 
{
    res->success = Drfl->move_resume();
};
auto move_pause_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MovePause::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::MovePause::Response> res)-> void                     
{
    res->success = false;
    res->success = Drfl->move_pause();
};

auto trans_cb = [this](const std::shared_ptr<dsr_msgs2::srv::Trans::Request> req, std::shared_ptr<dsr_msgs2::srv::Trans::Response> res)-> void                
{
    res->success = false;
    std::array<float, NUM_TASK> target_pos;
    std::array<float, NUM_TASK> delta_pos;
    LPROBOT_POSE robot_pos = Drfl->trans(target_pos.data(), delta_pos.data(), (COORDINATE_SYSTEM)req->ref, (COORDINATE_SYSTEM)req->ref_out);
    for(int i=0; i<NUM_TASK; i++){
        res->trans_pos[i] = robot_pos->_fPosition[i];
    }
    res->success = true;
};

auto fkin_cb = [this](const std::shared_ptr<dsr_msgs2::srv::Fkin::Request> req, std::shared_ptr<dsr_msgs2::srv::Fkin::Response> res)-> void             
{
    std::array<float, NUM_TASK> joint_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), joint_pos.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< fkin_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    joint_pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref       = %d",req->ref);      
#endif
    LPROBOT_POSE task_pos = Drfl->fkin(joint_pos.data(), (COORDINATE_SYSTEM)req->ref);
    for(int i=0; i<NUM_TASK; i++){
        res->conv_posx[i] = task_pos->_fPosition[i];
    }
    res->success = true;
};

auto ikin_cb = [this](const std::shared_ptr<dsr_msgs2::srv::Ikin::Request> req, std::shared_ptr<dsr_msgs2::srv::Ikin::Response> res)-> void           
{
    res->success = false;
    std::array<float, NUM_TASK> task_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), task_pos.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< ikin_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    task_pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref       = %d",req->ref);      
#endif

    LPROBOT_POSE joint_pos = Drfl->ikin(task_pos.data(), req->sol_space, (COORDINATE_SYSTEM)req->ref);
    for(int i=0; i<NUM_TASK; i++){
        res->conv_posj[i] = joint_pos->_fPosition[i];
    }
    res->success = true;
};

auto set_ref_coord_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetRefCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRefCoord::Response> res)-> void              
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< set_ref_coord_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    coord = %d",req->coord);      
#endif

    res->success = Drfl->set_ref_coord((COORDINATE_SYSTEM)req->coord);
};

auto move_home_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveHome::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveHome::Response> res)-> void                 
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< move_home_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    target = %d",req->target);      
#endif

    if(0 == req->target) 
        res->success = Drfl->move_home(MOVE_HOME_MECHANIC);
    else 
        res->success = Drfl->move_home(MOVE_HOME_USER);
};

auto check_motion_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CheckMotion::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::CheckMotion::Response> res)-> void               
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< check_motion_cb >");
#endif

    res->status = Drfl->check_motion();
    res->success = true;
};

auto change_operation_speed_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ChangeOperationSpeed::Request> req, std::shared_ptr<dsr_msgs2::srv::ChangeOperationSpeed::Response> res)-> void      
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< change_operation_speed_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    speed = %f",(float)req->speed);
#endif

    res->success = Drfl->change_operation_speed((float)req->speed);
};

auto enable_alter_motion_cb = [this](const std::shared_ptr<dsr_msgs2::srv::EnableAlterMotion::Request> req, std::shared_ptr<dsr_msgs2::srv::EnableAlterMotion::Response> res)-> void               
{
    std::array<float, 2> limit;
    std::array<float, 2> limit_per;
    std::copy(req->limit_dpos.cbegin(), req->limit_dpos.cend(), limit.begin());
    std::copy(req->limit_dpos_per.cbegin(), req->limit_dpos_per.cend(), limit_per.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< enable_alter_motion_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    n         = %d",req->n);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    mode      = %d",req->mode);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref       = %d",req->ref);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    limit     = %7.3f,%7.3f",limit[0],limit[1]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    limit_per = %7.3f,%7.3f",limit_per[0],limit_per[1]);
#endif
    res->success = Drfl->enable_alter_motion((int)req->n, (PATH_MODE)req->mode, (COORDINATE_SYSTEM)req->ref, limit.data(), limit_per.data());      
};

auto alter_motion_cb = [this](const std::shared_ptr<dsr_msgs2::srv::AlterMotion::Request> req, std::shared_ptr<dsr_msgs2::srv::AlterMotion::Response> res)-> void            
{
    std::array<float, NUM_TASK> pos_alter;
    std::copy(req->pos.cbegin(), req->pos.cend(), pos_alter.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< alter_motion_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos_alter = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",pos_alter[0],pos_alter[1],pos_alter[2],pos_alter[3],pos_alter[4],pos_alter[5]);
#endif

    res->success = Drfl->alter_motion(pos_alter.data());       
};

auto disable_alter_motion_cb = [this](const std::shared_ptr<dsr_msgs2::srv::DisableAlterMotion::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::DisableAlterMotion::Response> res)-> void             
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< disable_alter_motion_cb >");
#endif
    res->success = Drfl->disable_alter_motion();
};

auto set_singularity_handling_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetSingularityHandling::Request> req, std::shared_ptr<dsr_msgs2::srv::SetSingularityHandling::Response> res) -> void
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< set_singularity_handling_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    mode = %d",req->mode);
#endif
    res->success = Drfl->set_singularity_handling((SINGULARITY_AVOIDANCE)req->mode);      
};



//----- AUXILIARY_CONTROL Service Call-back functions ------------------------------------------------------------
auto get_control_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetControlMode::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetControlMode::Response> res)-> void                         
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_control_mode_cb >");
#endif
    //NO API , get mon_data      
    res->control_mode = g_stDrState.nActualMode;
    res->success = true;       
};

auto get_control_space_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetControlSpace::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetControlSpace::Response> res)-> void                        
{;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_control_space_cb >");
#endif
    //NO API , get mon_data
    res->space = g_stDrState.nActualSpace;
    res->success = true;
};

auto get_current_posj_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentPosj::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentPosj::Response> res) -> void                                          
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_current_posj_cb >");
#endif
    LPROBOT_POSE robot_pos = Drfl->GetCurrentPose((ROBOT_SPACE)ROBOT_SPACE_JOINT);
    for(int i = 0; i < NUM_TASK; i++){
        res->pos[i] = robot_pos->_fPosition[i];
    }
    res->success = true;        
};

auto get_current_velj_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentVelj::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentVelj::Response> res)-> void                              
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_current_velj_cb >");
#endif

    //NO API , get mon_data
    for(int i=0; i<NUM_TASK; i++){
        res->joint_speed[i] = g_stDrState.fCurrentVelj[i];
    }
    res->success = true;
};

auto get_desired_posj_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetDesiredPosj::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetDesiredPosj::Response> res)-> void      
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_desired_posj_cb >");
#endif
    //NO API , get mon_data
    for(int i=0; i<NUM_TASK; i++){
        res->pos[i] = g_stDrState.fTargetPosj[i];
    }
    res->success = true;        
};

auto get_desired_velj_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetDesiredVelj::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetDesiredVelj::Response> res)-> void                                  
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_desired_velj_cb >");
#endif
    //NO API , get mon_data
    for(int i=0; i<NUM_TASK; i++){
        res->joint_vel[i] = g_stDrState.fTargetVelj[i];
    }
    res->success = true;        
};

auto get_current_posx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentPosx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentPosx::Response> res)-> void                             
{
    std_msgs::msg::Float64MultiArray arr;

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_current_posx_cb >");
#endif

    LPROBOT_TASK_POSE cur_posx = Drfl->get_current_posx((COORDINATE_SYSTEM)req->ref);
    if(nullptr == cur_posx) {
        res->success = false;
        return;
    }
    arr.data.clear();
    for (int i = 0; i < NUM_TASK; i++){
        arr.data.push_back(cur_posx->_fTargetPos[i]);
    }
    arr.data.push_back(cur_posx->_iTargetSol);
    res->task_pos_info.push_back(arr);
    res->success = true;
};

auto get_current_velx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentVelx::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentVelx::Response> res)-> void                     
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_current_velx_cb >");
#endif

    //NO API , get mon_data
    for(int i=0; i<NUM_TASK; i++){
        res->vel[i] = g_stDrState.fCurrentVelx[i];
    }
    res->success = true;            
};

auto get_desired_posx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetDesiredPosx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetDesiredPosx::Response> res)-> void   
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_desired_posx_cb >");
#endif
    LPROBOT_POSE task_pos = Drfl->get_desired_posx((COORDINATE_SYSTEM)req->ref);
    for(int i=0; i<NUM_TASK; i++){
        res->pos[i] = task_pos->_fPosition[i];
    }
    res->success = true;        
};

auto get_desired_velx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetDesiredVelx::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetDesiredVelx::Response> res)-> void                            
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_desired_velx_cb >");
#endif
    //NO API , get mon_data
    for(int i=0; i<NUM_TASK; i++){
        res->vel[i] = g_stDrState.fTargetVelx[i];
    }
    res->success = true;                    
};

auto get_current_tool_flange_posx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentToolFlangePosx::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentToolFlangePosx::Response> res) -> void
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_current_tool_flange_posx_cb >");
#endif
    //NO API , get mon_data
    for(int i=0; i<NUM_TASK; i++){
        res->pos[i] = g_stDrState.fCurrentToolPosx[i];
    }
    res->success = true;        
};
auto get_current_solution_space_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentSolutionSpace::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentSolutionSpace::Response> res)-> void
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_current_solution_space_cb >");
#endif
    res->sol_space = Drfl->get_current_solution_space();
    res->success = true;
};

auto get_current_rotm_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentRotm::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentRotm::Response> res)-> void                    
{
    std_msgs::msg::Float64MultiArray arr;

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_current_rotm_cb >");
#endif
    //NO API , get mon_data
    for (int i = 0; i < 3; i++){
        arr.data.clear();
        for (int j = 0; j < 3; j++){
            arr.data.push_back(g_stDrState.fRotationMatrix[i][j]);
        }
        res->rot_matrix.push_back(arr);
    }
    res->success = true;         
};

auto get_joint_torque_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetJointTorque::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetJointTorque::Response> res)-> void                  
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_joint_torque_cb >");
#endif
    //NO API , get mon_data
    for(int i = 0; i < NUM_TASK; i++){
        res->jts[i] = g_stDrState.fActualJTS[i];
    }
    res->success = true;
};

auto get_external_torque_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetExternalTorque::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetExternalTorque::Response> res)-> void             
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_external_torque_cb >");
#endif
    //NO API , get mon_data
    for(int i = 0; i < NUM_TASK; i++){
        res->ext_torque[i] = g_stDrState.fActualEJT[i];
    }
    res->success = true;        
};

auto get_tool_force_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetToolForce::Request> req,
                                std::shared_ptr<dsr_msgs2::srv::GetToolForce::Response> res)
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"), "< get_tool_force_cb > ref: %d", req->ref);
#endif

    // Load rotation matrix from base to tool (3x3)
    float R[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            R[r][c] = g_stDrState.fRotationMatrix[r][c];

    switch (req->ref) {
        case 0:  // BASE frame (default): use raw ETT values directly
            for (int i = 0; i < NUM_TASK; i++)
                res->tool_force[i] = g_stDrState.fActualETT[i];
            break;

        case 1: {  // TOOL frame: apply coordinate transformation
            double force_base[3] = {
                g_stDrState.fActualETT[0],
                g_stDrState.fActualETT[1],
                g_stDrState.fActualETT[2]
            };
            
            double torque_tool[3] = {
                g_stDrState.fActualETT[3],
                g_stDrState.fActualETT[4],
                g_stDrState.fActualETT[5]
            };

            // Convert force to tool frame:
            for (int i = 0; i < 3; i++) {
                res->tool_force[i] = 0.0;
                for (int j = 0; j < 3; j++)
                    res->tool_force[i] += R[j][i] * force_base[j];
            }
            res->tool_force[3] = torque_tool[0];
            res->tool_force[4] = torque_tool[1];
            res->tool_force[5] = torque_tool[2];
            break;
        }

        case 2:  // WORLD frame: directly use precomputed values from monitoring
            for (int i = 0; i < NUM_TASK; i++)
                res->tool_force[i] = g_stDrState.fWorldETT[i];
            break;

        default: 
            RCLCPP_WARN(rclcpp::get_logger("dsr_controller2"),
                        "[get_tool_force_cb] Invalid ref: %d. Defaulting to BASE frame.", req->ref);
            for (int i = 0; i < NUM_TASK; i++)
                res->tool_force[i] = g_stDrState.fActualETT[i];
            break;
    }
    res->success = true;
};

auto get_solution_space_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetSolutionSpace::Request> req, std::shared_ptr<dsr_msgs2::srv::GetSolutionSpace::Response> res)-> void
{    
    std::array<float, NUM_TASK> task_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), task_pos.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_solution_space_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
#endif
    res->sol_space = Drfl->get_solution_space(task_pos.data());
    res->success = true;        
};

auto get_orientation_error_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetOrientationError::Request> req, std::shared_ptr<dsr_msgs2::srv::GetOrientationError::Response> res)-> void            
{        
    std::array<float, NUM_TASK> task_pos1;
    std::array<float, NUM_TASK> task_pos2;

    std::copy(req->xd.cbegin(), req->xd.cend(), task_pos1.begin());
    std::copy(req->xc.cbegin(), req->xc.cend(), task_pos2.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_orientation_error_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    xd = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos1[0],task_pos1[1],task_pos1[2],task_pos1[3],task_pos1[4],task_pos1[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    xc = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos2[0],task_pos2[1],task_pos2[2],task_pos2[3],task_pos2[4],task_pos2[5]);      
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    axis = %d",req->axis);
#endif
    res->ori_error = Drfl->get_orientation_error(task_pos1.data(), task_pos2.data(), (TASK_AXIS)req->axis);     //check 040404
    res->success = true;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

auto get_workpiece_weight_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetWorkpieceWeight::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetWorkpieceWeight::Response> res)-> void        
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_workpiece_weight_cb >");
#endif
    res->weight = Drfl->get_workpiece_weight();
    res->success = true;
};

auto reset_workpiece_weight_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ResetWorkpieceWeight::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::ResetWorkpieceWeight::Response> res)-> void        
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< reset_workpiece_weight_cb >");
#endif
    res->success = Drfl->reset_workpiece_weight();
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//----- FORCE/STIFFNESS Service Call-back functions ------------------------------------------------------------
auto parallel_axis1_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ParallelAxis1::Request> req, std::shared_ptr<dsr_msgs2::srv::ParallelAxis1::Response> res)-> void
{
    std::array<float, NUM_TASK> task_pos1;
    std::array<float, NUM_TASK> task_pos2;
    std::array<float, NUM_TASK> task_pos3;

    std::copy(req->x1.cbegin(), req->x1.cend(), task_pos1.begin());
    std::copy(req->x2.cbegin(), req->x2.cend(), task_pos2.begin());
    std::copy(req->x3.cbegin(), req->x3.cend(), task_pos3.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< parallel_axis1_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x1 = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos1[0],task_pos1[1],task_pos1[2],task_pos1[3],task_pos1[4],task_pos1[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x2 = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos2[0],task_pos2[1],task_pos2[2],task_pos2[3],task_pos2[4],task_pos2[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x3 = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos3[0],task_pos3[1],task_pos3[2],task_pos3[3],task_pos3[4],task_pos3[5]);
#endif
    res->success = Drfl->parallel_axis(task_pos1.data(), task_pos2.data(), task_pos3.data(), (TASK_AXIS)req->axis, (COORDINATE_SYSTEM)req->ref);
};

auto parallel_axis2_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ParallelAxis2::Request> req, std::shared_ptr<dsr_msgs2::srv::ParallelAxis2::Response> res)-> void
{
    std::array<float, 3> vector;

    std::copy(req->vect.cbegin(), req->vect.cend(), vector.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< parallel_axis2_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    vect = %7.3f,%7.3f,%7.3f",vector[0],vector[1],vector[2]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    axis = %d",req->axis);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref  = %d",req->ref);
#endif
    res->success = Drfl->parallel_axis(vector.data(), (TASK_AXIS)req->axis, (COORDINATE_SYSTEM)req->ref);        
};

auto align_axis1_cb = [this](const std::shared_ptr<dsr_msgs2::srv::AlignAxis1::Request> req, std::shared_ptr<dsr_msgs2::srv::AlignAxis1::Response> res)-> void    
{
    std::array<float, NUM_TASK> task_pos1;
    std::array<float, NUM_TASK> task_pos2;
    std::array<float, NUM_TASK> task_pos3;
    //std::array<float, NUM_TASK> task_pos4;
    float fSourceVec[3] = {0, };

    std::copy(req->x1.cbegin(), req->x1.cend(), task_pos1.begin());
    std::copy(req->x2.cbegin(), req->x2.cend(), task_pos2.begin());
    std::copy(req->x3.cbegin(), req->x3.cend(), task_pos3.begin());
    //std::copy(req->pos.cbegin(),req->pos.cend(),task_pos4.begin());
      //req->pos[6] -> fTargetVec[3] : only use [x,y,z]    
    for(int i=0; i<3; i++)        
        fSourceVec[i] = req->source_vect[i];

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< align_axis1_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x1  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos1[0],task_pos1[1],task_pos1[2],task_pos1[3],task_pos1[4],task_pos1[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x2  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos2[0],task_pos2[1],task_pos2[2],task_pos2[3],task_pos2[4],task_pos2[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x3  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos3[0],task_pos3[1],task_pos3[2],task_pos3[3],task_pos3[4],task_pos3[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos = %7.3f,%7.3f,%7.3f",fSourceVec[0],fSourceVec[1],fSourceVec[2]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    axis = %d",req->axis);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref  = %d",req->ref);
#endif
    res->success = Drfl->align_axis(task_pos1.data(), task_pos2.data(), task_pos3.data(), fSourceVec, (TASK_AXIS)req->axis, (COORDINATE_SYSTEM)req->ref);
};

auto align_axis2_cb = [this](const std::shared_ptr<dsr_msgs2::srv::AlignAxis2::Request> req, std::shared_ptr<dsr_msgs2::srv::AlignAxis2::Response> res)-> void
{
    float fTargetVec[3] = {0, };
    float fSourceVec[3] = {0, };

    for(int i=0; i<3; i++)
    {        
        fTargetVec[i] = req->target_vect[i];
        fSourceVec[i] = req->source_vect[i];     ////req->pos[6] -> fSourceVec[3] : only use [x,y,z]
    }

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< align_axis2_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    vect = %7.3f,%7.3f,%7.3f",fTargetVec[0],fTargetVec[1],fTargetVec[2]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos  = %7.3f,%7.3f,%7.3f",fSourceVec[0],fSourceVec[1],fSourceVec[2]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    axis = %d",req->axis);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref  = %d",req->ref);
#endif
    res->success = Drfl->align_axis(fTargetVec, fSourceVec, (TASK_AXIS)req->axis, (COORDINATE_SYSTEM)req->ref);
};


auto is_done_bolt_tightening_cb = [this](const std::shared_ptr<dsr_msgs2::srv::IsDoneBoltTightening::Request> req, std::shared_ptr<dsr_msgs2::srv::IsDoneBoltTightening::Response> res)-> void 
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< is_done_bolt_tightening_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    m       = %f",req->m);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    timeout = %f",req->timeout);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    axis    = %d",req->axis);
#endif
    res->success = Drfl->is_done_bolt_tightening((FORCE_AXIS)req->axis, req->m, req->timeout);
};


auto release_compliance_ctrl_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ReleaseComplianceCtrl::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::ReleaseComplianceCtrl::Response> res)-> void       
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< release_compliance_ctrl_cb >");
#endif
    res->success = Drfl->release_compliance_ctrl();
};


auto task_compliance_ctrl_cb = [this](const std::shared_ptr<dsr_msgs2::srv::TaskComplianceCtrl::Request> req, std::shared_ptr<dsr_msgs2::srv::TaskComplianceCtrl::Response> res) -> void      
{
    std::array<float, NUM_TASK> stiffnesses;
    std::copy(req->stx.cbegin(), req->stx.cend(), stiffnesses.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< task_compliance_ctrl_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    stx     = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",stiffnesses[0],stiffnesses[1],stiffnesses[2],stiffnesses[3],stiffnesses[4],stiffnesses[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref     = %d",req->ref);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    timeout = %f",req->time);
#endif
    res->success = Drfl->task_compliance_ctrl(stiffnesses.data(), (COORDINATE_SYSTEM)req->ref, req->time);
};
auto set_stiffnessx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetStiffnessx::Request> req, std::shared_ptr<dsr_msgs2::srv::SetStiffnessx::Response> res)-> void      
{
    std::array<float, NUM_TASK> stiffnesses;
    std::copy(req->stx.cbegin(), req->stx.cend(), stiffnesses.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< set_stiffnessx_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    stx     = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",stiffnesses[0],stiffnesses[1],stiffnesses[2],stiffnesses[3],stiffnesses[4],stiffnesses[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref     = %d",req->ref);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    timeout = %f",req->time);
#endif
    res->success = Drfl->set_stiffnessx(stiffnesses.data(), (COORDINATE_SYSTEM)req->ref, req->time);
};

auto calc_coord_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CalcCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::CalcCoord::Response> res)-> void 
{
    std::array<float, NUM_TASK> task_pos1;
    std::array<float, NUM_TASK> task_pos2;
    std::array<float, NUM_TASK> task_pos3;
    std::array<float, NUM_TASK> task_pos4;

    std::copy(req->x1.cbegin(), req->x1.cend(), task_pos1.begin());
    std::copy(req->x2.cbegin(), req->x2.cend(), task_pos2.begin());
    std::copy(req->x3.cbegin(), req->x3.cend(), task_pos3.begin());
    std::copy(req->x4.cbegin(), req->x4.cend(), task_pos4.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< calc_coord_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    input_pos_cnt = %d",req->input_pos_cnt); 
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x1  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos1[0],task_pos1[1],task_pos1[2],task_pos1[3],task_pos1[4],task_pos1[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x2  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos2[0],task_pos2[1],task_pos2[2],task_pos2[3],task_pos2[4],task_pos2[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x3  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos3[0],task_pos3[1],task_pos3[2],task_pos3[3],task_pos3[4],task_pos3[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x4  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos4[0],task_pos4[1],task_pos4[2],task_pos4[3],task_pos4[4],task_pos4[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref = %d",req->ref);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    mod = %d",req->mod);
#endif
    LPROBOT_POSE task_pos = Drfl->calc_coord(req->input_pos_cnt, req->mod, (COORDINATE_SYSTEM)req->ref, task_pos1.data(), task_pos2.data(), task_pos3.data(), task_pos4.data());
    for(int i=0; i<NUM_TASK; i++){
        res->conv_posx[i] = task_pos->_fPosition[i];
    }
    res->success = true;
};

auto set_user_cart_coord1_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord1::Request> req, std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord1::Response> res)-> void         
{
    std::array<float, NUM_TASK> task_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), task_pos.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< set_user_cart_coord1_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref = %d",req->ref);
#endif
    res->id = Drfl->set_user_cart_coord(0, task_pos.data(), (COORDINATE_SYSTEM)req->ref);  
    res->success = true;
};

auto set_user_cart_coord2_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord2::Request> req, std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord2::Response> res)-> void           
{
    //std::array<float, NUM_TASK> task_pos1;
    //std::array<float, NUM_TASK> task_pos2;
    //std::array<float, NUM_TASK> task_pos3;
    //std::array<float, NUM_TASK> target_org;
    float fTargetPos[3][NUM_TASK] = {0, };
    float fTargetOrg[3] = {0, };
    
    //req->x1[6] + req->x2[6] + req->x3[6] -> fTargetPos[3][NUM_TASK] 
    for(int i=0; i<NUM_TASK; i++)        
    {
        fTargetPos[0][i] = req->x1[i];
        fTargetPos[1][i] = req->x2[i];
        fTargetPos[2][i] = req->x3[i];
    }
    //req->pos[6] -> fTargetOrg[3] : only use [x,y,z]    
    for(int i=0; i<3; i++)        
        fTargetOrg[i] = req->pos[i];

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< set_user_cart_coord2_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x1  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",fTargetPos[0][0],fTargetPos[0][1],fTargetPos[0][2],fTargetPos[0][3],fTargetPos[0][4],fTargetPos[0][5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x2  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",fTargetPos[1][0],fTargetPos[1][1],fTargetPos[1][2],fTargetPos[1][3],fTargetPos[1][4],fTargetPos[1][5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    x3  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",fTargetPos[2][0],fTargetPos[2][1],fTargetPos[2][2],fTargetPos[2][3],fTargetPos[2][4],fTargetPos[2][5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",fTargetOrg[0],fTargetOrg[1],fTargetOrg[2],fTargetOrg[3],fTargetOrg[4],fTargetOrg[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref = %d",req->ref);
#endif
    res->id = Drfl->set_user_cart_coord(fTargetPos, fTargetOrg, (COORDINATE_SYSTEM)req->ref);
    res->success = true;        
};

auto set_user_cart_coord3_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord3::Request> req, std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord3::Response> res) -> void   
{
    float fTargetVec[2][3] = {0, };
    float fTargetOrg[3] = {0, };

    //req->u1[3] + req->c1[3] -> fTargetVec[2][3]
    for(int i=0; i<3; i++)        
    {
        fTargetVec[0][i] = req->u1[i];
        fTargetVec[1][i] = req->v1[i];
    }
    //req->pos[6] -> fTargetOrg[3] : only use [x,y,z]    
    for(int i=0; i<3; i++)        
        fTargetOrg[i] = req->pos[i];

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< set_user_cart_coord3_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    u1  = %7.3f,%7.3f,%7.3f",fTargetVec[0][0],fTargetVec[0][1],fTargetVec[0][2]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    v1  = %7.3f,%7.3f,%7.3f",fTargetVec[1][0],fTargetVec[1][1],fTargetVec[1][2]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    org = %7.3f,%7.3f,%7.3f",fTargetOrg[0],fTargetOrg[1],fTargetOrg[2]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref = %d",req->ref);
#endif
    res->id = Drfl->set_user_cart_coord(fTargetVec, fTargetOrg, (COORDINATE_SYSTEM)req->ref);
    res->success = true;
};

auto overwrite_user_cart_coord_cb = [this](const std::shared_ptr<dsr_msgs2::srv::OverwriteUserCartCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::OverwriteUserCartCoord::Response> res)-> void     
{
    std::array<float, NUM_TASK> task_pos;

    std::copy(req->pos.cbegin(),req->pos.cend(),task_pos.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< overwrite_user_cart_coord_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    id  = %d",req->id);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref = %d",req->ref);
#endif
    res->id = Drfl->overwrite_user_cart_coord(0, req->id, task_pos.data(), (COORDINATE_SYSTEM)req->ref);  //0=AUTO 
    res->success = true;
};

auto get_user_cart_coord_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetUserCartCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::GetUserCartCoord::Response> res)-> void       
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_user_cart_coord_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    id  = %d",req->id);
#endif
    LPUSER_COORDINATE result = Drfl->get_user_cart_coord(req->id);
    for(int i=0; i<NUM_TASK; i++){
        res->conv_posx[i] = result->_fTargetPos[i];
    }
    res->ref = result->_iTargetRef;
    res->success = true;
};

auto set_desired_force_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetDesiredForce::Request> req, std::shared_ptr<dsr_msgs2::srv::SetDesiredForce::Response> res)-> void     
{
    std::array<float, NUM_TASK> feedback;
    std::array<unsigned char, NUM_TASK> direction;

    std::copy(req->fd.cbegin(), req->fd.cend(), feedback.begin());
    std::copy(req->dir.cbegin(),req->dir.cend(), direction.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< set_desired_force_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    feedback  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",feedback[0],feedback[1],feedback[2],feedback[3],feedback[4],feedback[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    direction = %d,%d,%d,%d,%d,%d",direction[0],direction[1],direction[2],direction[3],direction[4],direction[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref   = %d", req->ref);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    time  = %f", req->time); 
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    mod   = %d", req->mod);
#endif
    res->success = Drfl->set_desired_force(feedback.data(), direction.data(), (COORDINATE_SYSTEM)req->ref, req->time, (FORCE_MODE)req->mod);
};

auto release_force_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ReleaseForce::Request> req, std::shared_ptr<dsr_msgs2::srv::ReleaseForce::Response> res)-> void   
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< release_force_cb >");
#endif
    res->success = Drfl->release_force(req->time);
};

auto check_position_condition_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CheckPositionCondition::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckPositionCondition::Response> res)-> void      
{
    std::array<float, NUM_TASK> task_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), task_pos.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< check_position_condition_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    axis = %d", req->axis);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    min  = %f", req->min);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    max  = %f", req->max);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref  = %d", req->ref);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    mode = %d", req->mode);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
#endif
    if(0==req->mode)  //DR_MV_MOD_ABS
        res->success = Drfl->check_position_condition_abs((FORCE_AXIS)req->axis, req->min, req->max, (COORDINATE_SYSTEM)req->ref);
    else            //DR_MV_MOD_REL
        res->success = Drfl->check_position_condition_rel((FORCE_AXIS)req->axis, req->min, req->max, task_pos.data(), (COORDINATE_SYSTEM)req->ref);
};

auto check_force_condition_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CheckForceCondition::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckForceCondition::Response> res)-> void    
{
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< check_force_condition_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    axis = %d", req->axis);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    min  = %f", req->min);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    max  = %f", req->max);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref  = %d", req->ref);
#endif
    res->success = Drfl->check_force_condition((FORCE_AXIS)req->axis, req->min, req->max, (COORDINATE_SYSTEM)req->ref);
};

auto check_orientation_condition1_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition1::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition1::Response> res)-> void       
{
    std::array<float, NUM_TASK> task_pos1;
    std::array<float, NUM_TASK> task_pos2;
    std::copy(req->min.cbegin(), req->min.cend(), task_pos1.begin());
    std::copy(req->max.cbegin(), req->max.cend(), task_pos2.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< check_orientation_condition1_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    axis = %d", req->axis);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    min = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos1[0],task_pos1[1],task_pos1[2],task_pos1[3],task_pos1[4],task_pos1[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    max = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos2[0],task_pos2[1],task_pos2[2],task_pos2[3],task_pos2[4],task_pos2[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref  = %d", req->ref);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    mode = %d", req->mode);
#endif
    res->success = Drfl->check_orientation_condition((FORCE_AXIS)req->axis , task_pos1.data(), task_pos2.data(), (COORDINATE_SYSTEM)req->ref);
};

auto check_orientation_condition2_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition2::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition2::Response> res)-> void
{
    std::array<float, NUM_TASK> task_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), task_pos.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< check_orientation_condition2_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    axis = %d", req->axis);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    min  = %f", req->min);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    max  = %f", req->max);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref  = %d", req->ref);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    mode = %d", req->mode);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
#endif
    res->success = Drfl->check_orientation_condition((FORCE_AXIS)req->axis , req->min, req->max, task_pos.data(), (COORDINATE_SYSTEM)req->ref);        
};

auto coord_transform_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CoordTransform::Request> req, std::shared_ptr<dsr_msgs2::srv::CoordTransform::Response> res)-> void         
{
    std::array<float, NUM_TASK> task_pos;
    std::copy(req->pos_in.cbegin(), req->pos_in.cend(), task_pos.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< coord_transform_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos_in  = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref_in  = %d", req->ref_in);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref_out = %d", req->ref_out);
#endif
    LPROBOT_POSE result_pos = Drfl->coord_transform(task_pos.data(), (COORDINATE_SYSTEM)req->ref_in, (COORDINATE_SYSTEM)req->ref_out);
    for(int i=0; i<NUM_TASK; i++){
        res->conv_posx[i] = result_pos->_fPosition[i];
    }
    res->success = true;
};

//----- IO Service Call-back functions ------------------------------------------------------------
auto set_digital_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxDigitalOutput::Response> res)-> void   
{
    res->success = false;
    if((req->index < DR_DIO_MIN_INDEX) || (req->index > DR_DIO_MAX_INDEX)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_digital_output(index=%d, value=%d): index(%d) is out of range. (normal range: %d ~ %d)",req->index ,req->value ,req->index, DR_DIO_MIN_INDEX, DR_DIO_MAX_INDEX);
    }       
    else if((req->value < 0) || (req->value > 1)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_digital_output(index=%d, value=%d): value(%d) is out of range. [normal range: 0 or 1]",req->index ,req->value ,req->value);
    }       
    else{
        req->index -=1;
        res->success = Drfl->set_digital_output((GPIO_CTRLBOX_DIGITAL_INDEX)req->index, req->value);
    }
};

auto get_digital_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalOutput::Response> res)-> void
{
    res->success = false;
    if((req->index < DR_DIO_MIN_INDEX) || (req->index > DR_DIO_MAX_INDEX)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_digital_output(index=%d): index(%d) is out of range. (normal range: %d ~ %d)",req->index ,req->index, DR_DIO_MIN_INDEX, DR_DIO_MAX_INDEX);
    }       
    else{
        req->index -=1;
        res->value = Drfl->get_digital_output((GPIO_CTRLBOX_DIGITAL_INDEX)req->index);
        res->success = true;
    }

};
auto get_digital_input_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalInput::Response> res)-> void
{
    //ROS_INFO("get_digital_input_cb() called and calling Drfl->GetCtrlBoxDigitalInput");
    res->success = false;
    if((req->index < DR_DIO_MIN_INDEX) || (req->index > DR_DIO_MAX_INDEX)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_digital_input(index=%d): index(%d) is out of range. [normal range: %d ~ %d]",req->index ,req->index, DR_DIO_MIN_INDEX, DR_DIO_MAX_INDEX);
    }       
    else{
        req->index -=1;
        res->value = Drfl->get_digital_input((GPIO_CTRLBOX_DIGITAL_INDEX)req->index);
        res->success = true;
    }
};

auto set_tool_digital_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetToolDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetToolDigitalOutput::Response> res)-> void
{
    //ROS_INFO("set_tool_digital_output_cb() called and calling Drfl->set_tool_digital_output");
    res->success = false;

    if((req->index < DR_TDIO_MIN_INDEX) || (req->index > DR_TDIO_MAX_INDEX)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_tool_digital_output(index=%d, value=%d): index(%d) is out of range. [normal range: %d ~ %d]",req->index ,req->value ,req->index, DR_TDIO_MIN_INDEX, DR_TDIO_MAX_INDEX);
    }       
    else if((req->value < 0) || (req->value > 1)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_tool_digital_output(index=%d, value=%d): value(%d) is out of range. [normal range: 0 or 1]",req->index ,req->value ,req->value);
    }
    else{       
        req->index -=1;
        res->success = Drfl->set_tool_digital_output((GPIO_TOOL_DIGITAL_INDEX)req->index, req->value);
    }

};

auto get_tool_digital_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetToolDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetToolDigitalOutput::Response> res)-> void 
{
    //ROS_INFO("get_tool_digital_output_cb() called and calling Drfl->get_tool_digital_output");
    res->success = false;
    if((req->index < DR_TDIO_MIN_INDEX) || (req->index > DR_TDIO_MAX_INDEX)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_tool_digital_output(index=%d): index(%d) is out of range. [normal range: %d ~ %d]",req->index ,req->index, DR_TDIO_MIN_INDEX, DR_TDIO_MAX_INDEX);
    }       
    else{       
        req->index -=1;
        res->value = Drfl->get_tool_digital_output((GPIO_TOOL_DIGITAL_INDEX)req->index);
        res->success = true;
    }
};
auto get_tool_digital_input_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetToolDigitalInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetToolDigitalInput::Response> res)-> void 
{
    //ROS_INFO("get_tool_digital_input_cb() called and calling Drfl->get_tool_digital_input");
    res->success = false;
    if((req->index < DR_TDIO_MIN_INDEX) || (req->index > DR_TDIO_MAX_INDEX)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_tool_digital_input(index=%d): index(%d) is out of range. [normal range: %d ~ %d]",req->index ,req->index, DR_TDIO_MIN_INDEX, DR_TDIO_MAX_INDEX);
    }       
    else{
        req->index -=1;
        res->value = Drfl->get_tool_digital_input((GPIO_TOOL_DIGITAL_INDEX)req->index);
        res->success = true;
    }
};
auto set_analog_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutput::Response> res)-> void
{        
    //ROS_INFO("set_analog_output_cb() called and calling Drfl->set_analog_output");
    res->success = false;
    bool bIsError = 0;   

    if((req->channel < 1) || (req->channel > 2)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_analog_output(channel=%d, value=%f): channel(%d) is out of range. [normal range: 1 or 2]",req->channel ,req->value, req->channel);
        bIsError = 1;
    }       
    else
    {
        if(req->channel == 1){                
            if(g_nAnalogOutputModeCh1==DR_ANALOG_CURRENT){
                if((req->value < 4.0) || (req->value > 20.0)){
                    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_analog_output(channel=%d, value=%f): value(%f) is out of range. [normal range: 4.0 ~ 20.0]",req->channel ,req->value ,req->value);
                    bIsError = 1;
                }
            }
            else if(g_nAnalogOutputModeCh1==DR_ANALOG_VOLTAGE){
                if((req->value < 0.0) || (req->value > 10.0)){
                    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_analog_output(channel=%d, value=%f): value(%f) is out of range. [normal range: 0.0 ~ 10.0]",req->channel ,req->value ,req->value);
                    bIsError = 1;
                }
            }         
            else{
                RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_analog_output(channel=%d, value=%f): Analog output mode(ch%d) is not set",req->channel ,req->value, req->channel);
                bIsError = 1;
            }    
        }
        if(req->channel == 2){                
            if(g_nAnalogOutputModeCh2==DR_ANALOG_CURRENT){
                if((req->value < 4.0) || (req->value > 20.0)){
                    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_analog_output(channel=%d, value=%f): value(%f) is out of range. [normal range: 4.0 ~ 20.0]",req->channel ,req->value ,req->value);
                    bIsError = 1;
                }
            }
            else if(g_nAnalogOutputModeCh2==DR_ANALOG_VOLTAGE){
                if((req->value < 0.0) || (req->value > 10.0)){
                    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_analog_output(channel=%d, value=%f): value(%f) is out of range. [normal range: 0.0 ~ 10.0]",req->channel ,req->value ,req->value);
                    bIsError = 1;
                }
            }         
            else{
                RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_analog_output(channel=%d, value=%f): Analog output mode(ch%d) is not set",req->channel ,req->value, req->channel);
                bIsError = 1;
            }    
        }
    }
    if(!bIsError)
    {
        req->channel -=1;
        res->success = Drfl->set_analog_output((GPIO_CTRLBOX_ANALOG_INDEX)req->channel, req->value);
    }
};

auto get_analog_input_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxAnalogInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxAnalogInput::Response> res)-> void
{
    //ROS_INFO("get_analog_input_cb() called and calling Drfl->get_analog_input");
    res->success = false;
    bool bIsError = 0;   

    if((req->channel < 1) || (req->channel > 2)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_analog_input(channel=%d): channel(%d) is out of range. [normal range: 1 or 2]", req->channel, req->channel);
        bIsError = 1;
    }       
    else{
        if(req->channel == 1){
            if(g_nAnalogOutputModeCh1 == -1){
                RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_analog_input(channel=%d): Analog output mode(ch%d) is not set",req->channel ,req->channel);
                bIsError = 1;
            }                                    
        }
        if(req->channel == 2){
            if(g_nAnalogOutputModeCh2 == -1){
                RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_analog_input(channel=%d): Analog output mode(ch%d) is not set",req->channel ,req->channel);
                bIsError = 1;
            }                                    
        }
    }

    if(!bIsError){
        req->channel -=1;
        res->value = Drfl->get_analog_input((GPIO_CTRLBOX_ANALOG_INDEX)req->channel);
        res->success = true;
    }
};

auto set_analog_output_type_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutputType::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutputType::Response> res)-> void  
{
    //ROS_INFO("set_analog_output_type_cb() called and calling Drfl->set_mode_analog_output");
    res->success = false;

    if((req->channel < 1) || (req->channel > 2)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_analog_output_type(channel=%d, mode=%d): channel(%d) is out of range. [normal range: 1 or 2]",req->channel ,req->mode, req->channel);
    }       
    else if((req->mode < 0) || (req->mode > 1)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_analog_output_type(channel=%d, mode=%d): mode(%d) is out of range. [normal range: 0 or 1]",req->channel ,req->mode, req->mode);
    }       
    else{
        if(req->channel == 1) g_nAnalogOutputModeCh1 = req->mode;    
        if(req->channel == 2) g_nAnalogOutputModeCh2 = req->mode;    
                
        req->channel -=1;
        res->success = Drfl->set_mode_analog_output((GPIO_CTRLBOX_ANALOG_INDEX)req->channel, (GPIO_ANALOG_TYPE)req->mode);
    }
};

auto set_analog_input_type_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogInputType::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogInputType::Response> res) -> void      
{
    //ROS_INFO("set_analog_input_type_cb() called and calling Drfl->set_mode_analog_input");
    res->success = false;

    if((req->channel < 1) || (req->channel > 2)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_analog_input_type(channel=%d, mode=%d): channel(%d) is out of range. [normal range: 1 or 2]",req->channel ,req->mode, req->channel);
    }       
    else if((req->mode < 0) || (req->mode > 1)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_analog_input_type(channel=%d, mode=%d): mode(%d) is out of range. [normal range: 0 or 1]",req->channel ,req->mode, req->mode);
    }       
    else{
        res->success = Drfl->set_mode_analog_input((GPIO_CTRLBOX_ANALOG_INDEX)(req->channel - 1), (GPIO_ANALOG_TYPE)req->mode);
        if(res->success) {
            if(req->channel == 1)   g_nAnalogOutputModeCh1 = req->mode;
            else if(req->channel == 2)  g_nAnalogOutputModeCh2 = req->mode;    
        }        
    }
};

//----- MODBUS Service Call-back functions ------------------------------------------------------------
auto set_modbus_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetModbusOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetModbusOutput::Response> res)-> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_modbus_output_cb() called and calling Drfl->set_modbus_output");
    res->success = false;
    res->success = Drfl->set_modbus_output(req->name, (unsigned short)req->value);
};

auto get_modbus_input_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetModbusInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetModbusInput::Response> res)-> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_modbus_input_cb() called and calling Drfl->get_modbus_input");
    res->value = Drfl->get_modbus_input(req->name);
};

auto config_create_modbus_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConfigCreateModbus::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigCreateModbus::Response> res)-> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"config_create_modbus_cb() called and calling Drfl->add_modbus_signal");
    res->success = Drfl->add_modbus_signal(req->name, req->ip, (unsigned short)req->port, (MODBUS_REGISTER_TYPE)req->reg_type, (unsigned short)req->index, (unsigned short)req->value, (int)req->slave_id);
};
auto config_delete_modbus_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConfigDeleteModbus::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigDeleteModbus::Response> res) -> void
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"config_delete_modbus_cb() called and calling Drfl->del_modbus_signal");
    res->success = Drfl->del_modbus_signal(req->name);
};

//----- DRL Service Call-back functions ------------------------------------------------------------
auto drl_pause_cb = [this](const std::shared_ptr<dsr_msgs2::srv::DrlPause::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::DrlPause::Response> res) -> void                      
{
    res->success = false;
    if(g_bIsEmulatorMode)
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"The drl service cannot be used in emulator mode (available in real mode).");
    else 
        res->success = Drfl->PlayDrlPause();

};
auto drl_start_cb = [this](const std::shared_ptr<dsr_msgs2::srv::DrlStart::Request> req, std::shared_ptr<dsr_msgs2::srv::DrlStart::Response> res)-> void
{
    //ROS_INFO("drl_start_cb() called and calling Drfl->drl_start");
    res->success = false;

    if(g_bIsEmulatorMode)
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"The drl service cannot be used in emulator mode (available in real mode).");
    else 
        res->success = Drfl->drl_start((ROBOT_SYSTEM)req->robot_system, req->code);

};
auto drl_stop_cb = [this](const std::shared_ptr<dsr_msgs2::srv::DrlStop::Request> req, std::shared_ptr<dsr_msgs2::srv::DrlStop::Response> res)-> void 
{
    //ROS_INFO("drl_stop_cb() called and calling Drfl->drl_stop");
    res->success = false;

    if(g_bIsEmulatorMode)
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"The drl service cannot be used in emulator mode (available in real mode).");
    else 
        res->success = Drfl->drl_stop((STOP_TYPE)req->stop_mode);
};

auto drl_resume_cb = [this](const std::shared_ptr<dsr_msgs2::srv::DrlResume::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::DrlResume::Response> res) -> void    
{
    //ROS_INFO("drl_resume_cb() called and calling Drfl->drl_resume");
    res->success = false;

    if(g_bIsEmulatorMode)
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"The drl service cannot be used in emulator mode (available in real mode).");
    else 
        res->success = Drfl->drl_resume();
};

auto get_drl_state_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetDrlState::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetDrlState::Response> res) -> void      
{
    res->success = false;

    if(g_bIsEmulatorMode)
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"The drl service cannot be used in emulator mode (available in real mode).");
    else{ 
        res->drl_state = Drfl->get_program_state();
        res->success = true;
    }    
};


//----- TCP Service Call-back functions -------------------------------------------------------------
auto set_current_tcp_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetCurrentTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCurrentTcp::Response> res) -> void     
{
    //ROS_INFO("set_current_tcp_cb() called and calling Drfl->set_tcp");
    res->success = false;
    res->success = Drfl->set_tcp(req->name);
};

auto get_current_tcp_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentTcp::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentTcp::Response> res) -> void    
{
    //ROS_INFO("get_current_tcp_cb() called and calling Drfl->get_tcp");
    res->info = Drfl->get_tcp();
    res->success = true;

};

auto config_create_tcp_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConfigCreateTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigCreateTcp::Response> res) -> void 
{
    //ROS_INFO("config_create_tcp_cb() called and calling Drfl->add_tcp");
    std::array<float, 6> target_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
    res->success = Drfl->add_tcp(req->name, target_pos.data());
};

auto config_delete_tcp_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTcp::Response> res) -> void
{
    //ROS_INFO("config_delete_tcp_cb() called and calling Drfl->del_tcp");
    res->success = Drfl->del_tcp(req->name);
};

//----- TOOL Service Call-back functions ------------------------------------------------------------
auto set_current_tool_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetCurrentTool::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCurrentTool::Response> res) -> void  
{
    //ROS_INFO("set_current_tool_cb() called and calling Drfl->set_tool");
    res->success = Drfl->set_tool(req->name);
};

auto get_current_tool_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentTool::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentTool::Response> res) -> void 
{
    //ROS_INFO("get_current_tool_cb() called and calling Drfl->get_tool %s", Drfl->GetCurrentTool().c_str());
    res->info = Drfl->get_tool();
    res->success = true;
};

auto config_create_tool_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConfigCreateTool::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigCreateTool::Response> res) -> void
{
    //ROS_INFO("config_create_tool_cb() called and calling Drfl->add_tool");
    std::array<float, 3> target_cog;
    std::array<float, 6> target_inertia;
    std::copy(req->cog.cbegin(), req->cog.cend(), target_cog.begin());
    std::copy(req->inertia.cbegin(), req->inertia.cend(), target_inertia.begin());
    res->success = Drfl->add_tool(req->name, req->weight, target_cog.data(), target_inertia.data());
};

auto config_delete_tool_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTool::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTool::Response> res) -> void  
{
    //ROS_INFO("config_delete_tool_cb() called and calling Drfl->del_tool");
    res->success = Drfl->del_tool(req->name);
};

auto set_tool_shape_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetToolShape::Request> req, std::shared_ptr<dsr_msgs2::srv::SetToolShape::Response> res) -> void
{
    res->success = Drfl->set_tool(req->name);
};

// RT control
auto connect_rt_control_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConnectRtControl::Request> req, 
                                    std::shared_ptr<dsr_msgs2::srv::ConnectRtControl::Response> res) -> void 
{
    res->success = Drfl->connect_rt_control(req->ip_address, req->port);
};

auto disconnect_rt_control_cb = [this](const std::shared_ptr<dsr_msgs2::srv::DisconnectRtControl::Request> /*req*/, 
                                       std::shared_ptr<dsr_msgs2::srv::DisconnectRtControl::Response> res) -> void 
{
    res->success = Drfl->disconnect_rt_control();
};

auto get_rt_control_output_version_list_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRtControlOutputVersionList::Request> /*req*/, 
                                                    std::shared_ptr<dsr_msgs2::srv::GetRtControlOutputVersionList::Response> res) -> void 
{
    res->version = Drfl->get_rt_control_output_version_list();
    res->success = true;
};

auto get_rt_control_input_version_list_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRtControlInputVersionList::Request> /*req*/, 
                                                   std::shared_ptr<dsr_msgs2::srv::GetRtControlInputVersionList::Response> res) -> void 
{
    res->version = Drfl->get_rt_control_input_version_list();
    res->success = true;
};

auto get_rt_control_input_data_list_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRtControlInputDataList::Request> req, 
                                                std::shared_ptr<dsr_msgs2::srv::GetRtControlInputDataList::Response> res) -> void 
{
    res->data = Drfl->get_rt_control_input_data_list(req->version);
    res->success = true;
};

auto get_rt_control_output_data_list_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRtControlOutputDataList::Request> req, 
                                                 std::shared_ptr<dsr_msgs2::srv::GetRtControlOutputDataList::Response> res) -> void 
{
    res->data = Drfl->get_rt_control_output_data_list(req->version);
    res->success = true;
};

auto set_rt_control_input_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetRtControlInput::Request> req, 
                                      std::shared_ptr<dsr_msgs2::srv::SetRtControlInput::Response> res) -> void 
{
    res->success = Drfl->set_rt_control_input(req->version, req->period, req->loss);
};

auto set_rt_control_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetRtControlOutput::Request> req, 
                                       std::shared_ptr<dsr_msgs2::srv::SetRtControlOutput::Response> res) -> void 
{
    res->success = Drfl->set_rt_control_output(req->version, req->period, req->loss);
};

auto start_rt_control_cb = [this](const std::shared_ptr<dsr_msgs2::srv::StartRtControl::Request> /*req*/, 
                                  std::shared_ptr<dsr_msgs2::srv::StartRtControl::Response> res) -> void 
{
    res->success = Drfl->start_rt_control();
};

auto stop_rt_control_cb = [this](const std::shared_ptr<dsr_msgs2::srv::StopRtControl::Request> /*req*/, 
                                 std::shared_ptr<dsr_msgs2::srv::StopRtControl::Response> res) -> void 
{
    res->success = Drfl->stop_rt_control();
};

auto set_velj_rt_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetVeljRt::Request> req, 
                             std::shared_ptr<dsr_msgs2::srv::SetVeljRt::Response> res) -> void 
{
    std::array<float, 6> vel;
    std::copy(req->vel.cbegin(), req->vel.cend(), vel.begin());
    res->success = Drfl->set_velj_rt(vel.data());
};

auto set_accj_rt_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetAccjRt::Request> req, 
                             std::shared_ptr<dsr_msgs2::srv::SetAccjRt::Response> res) -> void 
{
    std::array<float, 6> acc;
    std::copy(req->acc.cbegin(), req->acc.cend(), acc.begin());
    res->success = Drfl->set_accj_rt(acc.data());
};

auto set_velx_rt_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetVelxRt::Request> req, 
                             std::shared_ptr<dsr_msgs2::srv::SetVelxRt::Response> res) -> void 
{
    res->success = Drfl->set_velx_rt(req->trans, req->rotation);
};

auto set_accx_rt_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetAccxRt::Request> req, 
                             std::shared_ptr<dsr_msgs2::srv::SetAccxRt::Response> res) -> void 
{
    res->success = Drfl->set_accx_rt(req->trans, req->rotation);
};

auto read_data_rt_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ReadDataRt::Request> /*req*/, 
                              std::shared_ptr<dsr_msgs2::srv::ReadDataRt::Response> res) -> void 
{
    LPRT_OUTPUT_DATA_LIST temp = Drfl->read_data_rt();
    res->data.time_stamp = temp->time_stamp;
    
    for(int i = 0; i < 6; i++) {
        res->data.actual_joint_position[i] = temp->actual_joint_position[i];
        // Continue for all data fields as before
        res->data.actual_joint_position_abs[i] = temp->actual_joint_position_abs[i];
        res->data.actual_joint_velocity[i] = temp->actual_joint_velocity[i];
        res->data.actual_joint_velocity_abs[i] = temp->actual_joint_velocity_abs[i];
        res->data.actual_tcp_position[i] = temp->actual_tcp_position[i];
        res->data.actual_tcp_velocity[i] = temp->actual_tcp_velocity[i];
        res->data.actual_flange_position[i] = temp->actual_flange_position[i];
        res->data.actual_flange_velocity[i] = temp->actual_flange_velocity[i];
        res->data.actual_motor_torque[i] = temp->actual_motor_torque[i];
        res->data.actual_joint_torque[i] = temp->actual_joint_torque[i];
        res->data.raw_joint_torque[i] = temp->raw_joint_torque[i];
        res->data.raw_force_torque[i] = temp->raw_force_torque[i];
        res->data.external_joint_torque[i] = temp->external_joint_torque[i];
        res->data.external_tcp_force[i] = temp->external_tcp_force[i];
        res->data.target_joint_position[i] = temp->target_joint_position[i];
        res->data.target_joint_velocity[i] = temp->target_joint_velocity[i];
        res->data.target_joint_acceleration[i] = temp->target_joint_acceleration[i];
        res->data.target_motor_torque[i] = temp->target_motor_torque[i];
        res->data.target_tcp_position[i] = temp->target_tcp_position[i];
        res->data.target_tcp_velocity[i] = temp->target_tcp_velocity[i];
        res->data.gravity_torque[i] = temp->gravity_torque[i];
        res->data.joint_temperature[i] = temp->joint_temperature[i];
        res->data.goal_joint_position[i] = temp->goal_joint_position[i];
        res->data.goal_tcp_position[i] = temp->goal_tcp_position[i];
        res->data.goal_joint_position[i] = temp->goal_joint_position[i];
        res->data.goal_tcp_position[i] = temp->goal_tcp_position[i];
    }

    std_msgs::msg::Float64MultiArray arr;
    for(int i = 0; i < 6; i++) {
        arr.data.clear();
        for(int j = 0; j < 6; j++) {
            arr.data.push_back(temp->coriolis_matrix[i][j]);
        }
        res->data.coriolis_matrix.push_back(arr);
    }
    // Similar handling for mass_matrix, jacobian_matrix, and other arrays
    std_msgs::msg::Float64MultiArray arr1;
    for(int i=0; i<6; i++){
        arr1.data.clear();
        for(int j=0; j<6; j++){
            arr1.data.push_back(temp->mass_matrix[i][j]);
        }
        res->data.mass_matrix.push_back(arr1);
    }
    std_msgs::msg::Float64MultiArray arr2;
    for(int i=0; i<6; i++){
        arr2.data.clear();
        for(int j=0; j<6; j++){
            arr2.data.push_back(temp->jacobian_matrix[i][j]);
        }
        res->data.jacobian_matrix.push_back(arr2);
    }
    res->data.solution_space = temp->solution_space;
    res->data.singularity = temp->singularity;
    res->data.operation_speed_rate = temp->operation_speed_rate;
    res->data.controller_digital_input = temp->controller_digital_input;
    res->data.controller_digital_output = temp->controller_digital_output;

    for(int i=0; i<2; i++){
        res->data.controller_analog_input_type[i] = temp->controller_analog_input_type[i];
        res->data.controller_analog_input[i] = temp->controller_analog_input[i];
        res->data.controller_analog_output_type[i] = temp->controller_analog_output_type[i];
        res->data.controller_analog_output[i] = temp->controller_analog_output[i];
        res->data.external_encoder_strobe_count[i] = temp->external_encoder_strobe_count[i];
        res->data.external_encoder_count[i] = temp->external_encoder_count[i];
    }

    res->data.flange_digital_input = temp->flange_digital_input;
    res->data.flange_digital_output = temp->flange_digital_output;

    for(int i=0; i<4; i++){
        res->data.flange_analog_input[i] = temp->flange_analog_input[i];
    }
    res->data.robot_mode = temp->robot_mode;
    res->data.robot_state = temp->robot_state;
    res->data.control_mode = temp->control_mode;
};

auto write_data_rt_cb = [this](const std::shared_ptr<dsr_msgs2::srv::WriteDataRt::Request> req, 
                               std::shared_ptr<dsr_msgs2::srv::WriteDataRt::Response> res) -> void 
{
    std::array<float, 6> external_force_torque;
    std::array<float, 6> external_analog_output;
    std::array<float, 6> external_analog_input;

    std::copy(req->external_force_torque.cbegin(), req->external_force_torque.cend(), external_force_torque.begin());
    std::copy(req->external_analog_output.cbegin(), req->external_analog_output.cend(), external_analog_output.begin());
    std::copy(req->external_analog_input.cbegin(), req->external_analog_input.cend(), external_analog_input.begin());

    res->success = Drfl->write_data_rt(external_force_torque.data(), req->external_digital_input, 
                                       req->external_digital_output, external_analog_input.data(), 
                                       external_analog_output.data());
};


// Callback for alter_motion_stream
auto alter_cb = [this](const std::shared_ptr<dsr_msgs2::msg::AlterMotionStream> msg) -> void
{
    std::array<float, NUM_JOINT> target_pos;
    std::copy(msg->pos.cbegin(), msg->pos.cend(), target_pos.begin());
    Drfl->alter_motion(target_pos.data());
};

// Callback for servoj_stream
auto servoj_cb = [this](const std::shared_ptr<dsr_msgs2::msg::ServojStream> msg) -> void
{
    std::array<float, NUM_JOINT> target_pos;
    std::copy(msg->pos.cbegin(), msg->pos.cend(), target_pos.begin());
    std::array<float, NUM_JOINT> target_vel;
    std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
    std::array<float, NUM_JOINT> target_acc;
    std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
    float time = msg->time;
    DR_SERVOJ_TYPE mode = (DR_SERVOJ_TYPE)msg->mode;
    check_dsr_model(target_pos);
    
    Drfl->servoj(target_pos.data(), target_vel.data(), target_acc.data(), time, mode);
};

// Callback for servol_stream
auto servol_cb = [this](const std::shared_ptr<dsr_msgs2::msg::ServolStream> msg) -> void
{
    std::array<float, NUM_TASK> target_pos;
    std::copy(msg->pos.cbegin(), msg->pos.cend(), target_pos.begin());
    std::array<float, 2> target_vel;
    std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
    std::array<float, 2> target_acc;
    std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
    float time = msg->time;

    Drfl->servol(target_pos.data(), target_vel.data(), target_acc.data(), time);
};

// Callback for speedj_stream
auto speedj_cb = [this](const std::shared_ptr<dsr_msgs2::msg::SpeedjStream> msg) -> void
{
    std::array<float, NUM_JOINT> target_vel;
    std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
    std::array<float, NUM_JOINT> target_acc;
    std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
    float time = msg->time;
    check_dsr_model(target_vel);
    check_dsr_model(target_acc);

    Drfl->speedj(target_vel.data(), target_acc.data(), time);
};

// Callback for speedl_stream
auto speedl_cb = [this](const std::shared_ptr<dsr_msgs2::msg::SpeedlStream> msg) -> void
{
    std::array<float, NUM_TASK> target_vel;
    std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
    std::array<float, 2> target_acc;
    std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
    float time = msg->time;

    Drfl->speedl(target_vel.data(), target_acc.data(), time);
};

// Callback for servoj_rt_stream
auto servoj_rt_cb = [this](const std::shared_ptr<dsr_msgs2::msg::ServojRtStream> msg) -> void
{
    std::array<float, NUM_JOINT> target_pos;
    std::copy(msg->pos.cbegin(), msg->pos.cend(), target_pos.begin());
    std::array<float, NUM_JOINT> target_vel;
    std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
    std::array<float, NUM_JOINT> target_acc;
    std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
    float time = msg->time;
    check_dsr_model(target_pos);
    
    Drfl->servoj_rt(target_pos.data(), target_vel.data(), target_acc.data(), time);
};

// Callback for servol_rt_stream
auto servol_rt_cb = [this](const std::shared_ptr<dsr_msgs2::msg::ServolRtStream> msg) -> void
{
    std::array<float, NUM_TASK> target_pos;
    std::copy(msg->pos.cbegin(), msg->pos.cend(), target_pos.begin());
    std::array<float, NUM_TASK> target_vel;
    std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
    std::array<float, NUM_TASK> target_acc;
    std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
    float time = msg->time;

    Drfl->servol_rt(target_pos.data(), target_vel.data(), target_acc.data(), time);
};

// Callback for speedj_rt_stream
auto speedj_rt_cb = [this](const std::shared_ptr<dsr_msgs2::msg::SpeedjRtStream> msg) -> void
{
    std::array<float, NUM_JOINT> target_vel;
    std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
    std::array<float, NUM_JOINT> target_acc;
    std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
    float time = msg->time;
    check_dsr_model(target_vel);
    check_dsr_model(target_acc);

    Drfl->speedj_rt(target_vel.data(), target_acc.data(), time);
};

// Callback for speedl_rt_stream
auto speedl_rt_cb = [this](const std::shared_ptr<dsr_msgs2::msg::SpeedlRtStream> msg) -> void
{
    std::array<float, NUM_TASK> target_vel;
    std::copy(msg->vel.cbegin(), msg->vel.cend(), target_vel.begin());
    std::array<float, NUM_TASK> target_acc;
    std::copy(msg->acc.cbegin(), msg->acc.cend(), target_acc.begin());
    float time = msg->time;

    Drfl->speedl_rt(target_vel.data(), target_acc.data(), time);
};

// Callback for torque_rt_stream
auto torque_rt_cb = [this](const std::shared_ptr<dsr_msgs2::msg::TorqueRtStream> msg) -> void
{
    std::array<float, NUM_TASK> tor;
    std::copy(msg->tor.cbegin(), msg->tor.cend(), tor.begin());
    float time = msg->time;

    Drfl->torque_rt(tor.data(), time);
};


  error_log_pub_ = get_node()->create_publisher<dsr_msgs2::msg::RobotError>("error", 100);
  disconnect_pub_ = get_node()->create_publisher<dsr_msgs2::msg::RobotDisconnection>("robot_disconnection", 100);

  cb_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Subscription declarations
  m_sub_alter_motion_stream           = get_node()->create_subscription<dsr_msgs2::msg::AlterMotionStream>("alter_motion_stream", 20, alter_cb);
  m_sub_servoj_stream                 = get_node()->create_subscription<dsr_msgs2::msg::ServojStream>("servoj_stream", 20, servoj_cb);
  m_sub_servol_stream                 = get_node()->create_subscription<dsr_msgs2::msg::ServolStream>("servol_stream", 20, servol_cb);
  m_sub_speedj_stream                 = get_node()->create_subscription<dsr_msgs2::msg::SpeedjStream>("speedj_stream", 20, speedj_cb);
  m_sub_speedl_stream                 = get_node()->create_subscription<dsr_msgs2::msg::SpeedlStream>("speedl_stream", 10, speedl_cb);

  m_sub_servoj_rt_stream              = get_node()->create_subscription<dsr_msgs2::msg::ServojRtStream>("servoj_rt_stream", 20, servoj_rt_cb);
  m_sub_servol_rt_stream              = get_node()->create_subscription<dsr_msgs2::msg::ServolRtStream>("servol_rt_stream", 20, servol_rt_cb);
  m_sub_speedj_rt_stream              = get_node()->create_subscription<dsr_msgs2::msg::SpeedjRtStream>("speedj_rt_stream", 20, speedj_rt_cb);
  m_sub_speedl_rt_stream              = get_node()->create_subscription<dsr_msgs2::msg::SpeedlRtStream>("speedl_rt_stream", 20, speedl_rt_cb);
  m_sub_torque_rt_stream              = get_node()->create_subscription<dsr_msgs2::msg::TorqueRtStream>("torque_rt_stream", 20, torque_rt_cb);
  
  m_nh_srv_set_robot_mode             = get_node()->create_service<dsr_msgs2::srv::SetRobotMode>("system/set_robot_mode", set_robot_mode_cb);
  m_nh_srv_get_robot_mode             = get_node()->create_service<dsr_msgs2::srv::GetRobotMode>("system/get_robot_mode", get_robot_mode_cb);     
  m_nh_srv_set_robot_system           = get_node()->create_service<dsr_msgs2::srv::SetRobotSystem>("system/set_robot_system", set_robot_system_cb);         
  m_nh_srv_get_robot_system           = get_node()->create_service<dsr_msgs2::srv::GetRobotSystem>("system/get_robot_system", get_robot_system_cb);         
  m_nh_srv_get_robot_state            = get_node()->create_service<dsr_msgs2::srv::GetRobotState>("system/get_robot_state", get_robot_state_cb);        
  m_nh_srv_set_robot_speed_mode       = get_node()->create_service<dsr_msgs2::srv::SetRobotSpeedMode>("system/set_robot_speed_mode", set_robot_speed_mode_cb);    
  m_nh_srv_get_robot_speed_mode       = get_node()->create_service<dsr_msgs2::srv::GetRobotSpeedMode>("system/get_robot_speed_mode", get_robot_speed_mode_cb);       
  m_nh_srv_get_current_pose           = get_node()->create_service<dsr_msgs2::srv::GetCurrentPose>("system/get_current_pose", get_current_pose_cb);   
  m_nh_srv_set_safe_stop_reset_type   = get_node()->create_service<dsr_msgs2::srv::SetSafeStopResetType>("system/set_safe_stop_reset_type", set_safe_stop_reset_type_cb);           
  m_nh_srv_get_last_alarm             = get_node()->create_service<dsr_msgs2::srv::GetLastAlarm>("system/get_last_alarm", get_last_alarm_cb);   
  m_nh_srv_servo_off                  = get_node()->create_service<dsr_msgs2::srv::ServoOff>("system/servo_off", servo_off_cb);   
  m_nh_srv_set_robot_control          = get_node()->create_service<dsr_msgs2::srv::SetRobotControl>("system/set_robot_control", set_robot_control_cb);
  m_nh_srv_change_collision_sensitivity = get_node()->create_service<dsr_msgs2::srv::ChangeCollisionSensitivity>("system/change_collision_sensitivity", change_collision_sensitivity_cb);   
  m_nh_srv_set_safety_mode          = get_node()->create_service<dsr_msgs2::srv::SetSafetyMode>("system/set_safety_mode", set_safety_mode_cb);
	

  //  motion Operations
  m_nh_srv_move_joint                 = get_node()->create_service<dsr_msgs2::srv::MoveJoint>("motion/move_joint", movej_cb);                                
  m_nh_srv_move_line                  = get_node()->create_service<dsr_msgs2::srv::MoveLine>("motion/move_line", movel_cb);                        
  m_nh_srv_move_jointx                = get_node()->create_service<dsr_msgs2::srv::MoveJointx>("motion/move_jointx", movejx_cb);               
  m_nh_srv_move_circle                = get_node()->create_service<dsr_msgs2::srv::MoveCircle>("motion/move_circle", movec_cb);       
  m_nh_srv_move_spline_joint          = get_node()->create_service<dsr_msgs2::srv::MoveSplineJoint>("motion/move_spline_joint", movesj_cb);      
  m_nh_srv_move_spline_task           = get_node()->create_service<dsr_msgs2::srv::MoveSplineTask>("motion/move_spline_task", movesx_cb);          
  m_nh_srv_move_blending              = get_node()->create_service<dsr_msgs2::srv::MoveBlending>("motion/move_blending", moveb_cb);          
  m_nh_srv_move_spiral                = get_node()->create_service<dsr_msgs2::srv::MoveSpiral>("motion/move_spiral", movespiral_cb);              
  m_nh_srv_move_periodic              = get_node()->create_service<dsr_msgs2::srv::MovePeriodic>("motion/move_periodic", moveperiodic_cb);              
  m_nh_srv_move_wait                  = get_node()->create_service<dsr_msgs2::srv::MoveWait>("motion/move_wait", movewait_cb);                  
  m_nh_srv_jog                        = get_node()->create_service<dsr_msgs2::srv::Jog>("motion/jog", jog_cb);              
  m_nh_srv_jog_multi                  = get_node()->create_service<dsr_msgs2::srv::JogMulti>("motion/jog_multi", jog_multi_cb);                      
  m_nh_srv_move_pause                 = get_node()->create_service<dsr_msgs2::srv::MovePause>("motion/move_pause", move_pause_cb);                      
  m_nh_srv_move_stop                  = get_node()->create_service<dsr_msgs2::srv::MoveStop>("motion/move_stop", move_stop_cb, rmw_qos_profile_services_default, cb_group_);
  m_nh_srv_move_resume                = get_node()->create_service<dsr_msgs2::srv::MoveResume>("motion/move_resume", move_resume_cb);                  
  m_nh_srv_trans                      = get_node()->create_service<dsr_msgs2::srv::Trans>("motion/trans", trans_cb);                  
  m_nh_srv_fkin                       = get_node()->create_service<dsr_msgs2::srv::Fkin>("motion/fkin", fkin_cb);              
  m_nh_srv_ikin                       = get_node()->create_service<dsr_msgs2::srv::Ikin>("motion/ikin", ikin_cb);              
  m_nh_srv_set_ref_coord              = get_node()->create_service<dsr_msgs2::srv::SetRefCoord>("motion/set_ref_coord", set_ref_coord_cb);                  
  m_nh_srv_move_home                  = get_node()->create_service<dsr_msgs2::srv::MoveHome>("motion/move_home", move_home_cb);                  
  m_nh_srv_check_motion               = get_node()->create_service<dsr_msgs2::srv::CheckMotion>("motion/check_motion", check_motion_cb);                  
  m_nh_srv_change_operation_speed     = get_node()->create_service<dsr_msgs2::srv::ChangeOperationSpeed>("motion/change_operation_speed", change_operation_speed_cb);                  
  m_nh_srv_enable_alter_motion        = get_node()->create_service<dsr_msgs2::srv::EnableAlterMotion>("motion/enable_alter_motion", enable_alter_motion_cb);                      
  m_nh_srv_alter_motion               = get_node()->create_service<dsr_msgs2::srv::AlterMotion>("motion/alter_motion", alter_motion_cb);              
  m_nh_srv_disable_alter_motion       = get_node()->create_service<dsr_msgs2::srv::DisableAlterMotion>("motion/disable_alter_motion", disable_alter_motion_cb);                  
  m_nh_srv_set_singularity_handling   = get_node()->create_service<dsr_msgs2::srv::SetSingularityHandling>("motion/set_singularity_handling", set_singularity_handling_cb);                      

  //  auxiliary_control
  m_nh_srv_get_control_mode               = get_node()->create_service<dsr_msgs2::srv::GetControlMode>("aux_control/get_control_mode", get_control_mode_cb);                           
  m_nh_srv_get_control_space              = get_node()->create_service<dsr_msgs2::srv::GetControlSpace>("aux_control/get_control_space", get_control_space_cb);                          
  m_nh_srv_get_current_posj               = get_node()->create_service<dsr_msgs2::srv::GetCurrentPosj>("aux_control/get_current_posj", get_current_posj_cb);                                            
  m_nh_srv_get_current_velj               = get_node()->create_service<dsr_msgs2::srv::GetCurrentVelj>("aux_control/get_current_velj", get_current_velj_cb);                                
  m_nh_srv_get_desired_posj               = get_node()->create_service<dsr_msgs2::srv::GetDesiredPosj>("aux_control/get_desired_posj", get_desired_posj_cb);         
  m_nh_srv_get_desired_velj               = get_node()->create_service<dsr_msgs2::srv::GetDesiredVelj>("aux_control/get_desired_velj", get_desired_velj_cb);                                   
  m_nh_srv_get_current_posx               = get_node()->create_service<dsr_msgs2::srv::GetCurrentPosx>("aux_control/get_current_posx", get_current_posx_cb);                               
  m_nh_srv_get_current_tool_flange_posx   = get_node()->create_service<dsr_msgs2::srv::GetCurrentToolFlangePosx>("aux_control/get_current_tool_flange_posx", get_current_tool_flange_posx_cb);                 
  m_nh_srv_get_current_velx               = get_node()->create_service<dsr_msgs2::srv::GetCurrentVelx>("aux_control/get_current_velx", get_current_velx_cb);                         
  m_nh_srv_get_desired_posx               = get_node()->create_service<dsr_msgs2::srv::GetDesiredPosx>("aux_control/get_desired_posx", get_desired_posx_cb);     
  m_nh_srv_get_desired_velx               = get_node()->create_service<dsr_msgs2::srv::GetDesiredVelx>("aux_control/get_desired_velx", get_desired_velx_cb);                             
  m_nh_srv_get_current_solution_space     = get_node()->create_service<dsr_msgs2::srv::GetCurrentSolutionSpace>("aux_control/get_current_solution_space", get_current_solution_space_cb);                     
  m_nh_srv_get_current_rotm               = get_node()->create_service<dsr_msgs2::srv::GetCurrentRotm>("aux_control/get_current_rotm", get_current_rotm_cb);                     
  m_nh_srv_get_joint_torque               = get_node()->create_service<dsr_msgs2::srv::GetJointTorque>("aux_control/get_joint_torque", get_joint_torque_cb);                     
  m_nh_srv_get_external_torque            = get_node()->create_service<dsr_msgs2::srv::GetExternalTorque>("aux_control/get_external_torque", get_external_torque_cb);                
  m_nh_srv_get_tool_force                 = get_node()->create_service<dsr_msgs2::srv::GetToolForce>("aux_control/get_tool_force", get_tool_force_cb);                               
  m_nh_srv_get_solution_space             = get_node()->create_service<dsr_msgs2::srv::GetSolutionSpace>("aux_control/get_solution_space", get_solution_space_cb);       
  m_nh_srv_get_orientation_error          = get_node()->create_service<dsr_msgs2::srv::GetOrientationError>("aux_control/get_orientation_error", get_orientation_error_cb);              
  
  //  force/stiffness
  m_nh_srv_parallel_axis1                 = get_node()->create_service<dsr_msgs2::srv::ParallelAxis1>("force/parallel_axis1", parallel_axis1_cb);  
  m_nh_srv_parallel_axis2                 = get_node()->create_service<dsr_msgs2::srv::ParallelAxis2>("force/parallel_axis2", parallel_axis2_cb);  
  m_nh_srv_align_axis1                    = get_node()->create_service<dsr_msgs2::srv::AlignAxis1>("force/align_axis1", align_axis1_cb);          
  m_nh_srv_align_axis2                    = get_node()->create_service<dsr_msgs2::srv::AlignAxis2>("force/align_axis2", align_axis2_cb);      
  m_nh_srv_is_done_bolt_tightening        = get_node()->create_service<dsr_msgs2::srv::IsDoneBoltTightening>("force/is_done_bolt_tightening", is_done_bolt_tightening_cb);      
  m_nh_srv_release_compliance_ctrl        = get_node()->create_service<dsr_msgs2::srv::ReleaseComplianceCtrl>("force/release_compliance_ctrl", release_compliance_ctrl_cb);          
  m_nh_srv_task_compliance_ctrl           = get_node()->create_service<dsr_msgs2::srv::TaskComplianceCtrl>("force/task_compliance_ctrl", task_compliance_ctrl_cb);          
  m_nh_srv_set_stiffnessx                 = get_node()->create_service<dsr_msgs2::srv::SetStiffnessx>("force/set_stiffnessx", set_stiffnessx_cb);          
  m_nh_srv_calc_coord                     = get_node()->create_service<dsr_msgs2::srv::CalcCoord>("force/calc_coord", calc_coord_cb);      
  m_nh_srv_set_user_cart_coord1           = get_node()->create_service<dsr_msgs2::srv::SetUserCartCoord1>("force/set_user_cart_coord1", set_user_cart_coord1_cb);          
  m_nh_srv_set_user_cart_coord2           = get_node()->create_service<dsr_msgs2::srv::SetUserCartCoord2>("force/set_user_cart_coord2", set_user_cart_coord2_cb);              
  m_nh_srv_set_user_cart_coord3           = get_node()->create_service<dsr_msgs2::srv::SetUserCartCoord3>("force/set_user_cart_coord3", set_user_cart_coord3_cb);      
  m_nh_srv_overwrite_user_cart_coord      = get_node()->create_service<dsr_msgs2::srv::OverwriteUserCartCoord>("force/overwrite_user_cart_coord", overwrite_user_cart_coord_cb);      
  m_nh_srv_get_user_cart_coord            = get_node()->create_service<dsr_msgs2::srv::GetUserCartCoord>("force/get_user_cart_coord", get_user_cart_coord_cb);          
  m_nh_srv_set_desired_force              = get_node()->create_service<dsr_msgs2::srv::SetDesiredForce>("force/set_desired_force", set_desired_force_cb);          
  m_nh_srv_release_force                  = get_node()->create_service<dsr_msgs2::srv::ReleaseForce>("force/release_force", release_force_cb);      
  m_nh_srv_check_position_condition       = get_node()->create_service<dsr_msgs2::srv::CheckPositionCondition>("force/check_position_condition", check_position_condition_cb);          
  m_nh_srv_check_force_condition          = get_node()->create_service<dsr_msgs2::srv::CheckForceCondition>("force/check_force_condition", check_force_condition_cb);      
  m_nh_srv_check_orientation_condition1   = get_node()->create_service<dsr_msgs2::srv::CheckOrientationCondition1>("force/check_orientation_condition1", check_orientation_condition1_cb);             
  m_nh_srv_check_orientation_condition2   = get_node()->create_service<dsr_msgs2::srv::CheckOrientationCondition2>("force/check_orientation_condition2", check_orientation_condition2_cb);            
  m_nh_srv_coord_transform                = get_node()->create_service<dsr_msgs2::srv::CoordTransform>("force/coord_transform", coord_transform_cb);          
  m_nh_srv_get_workpiece_weight           = get_node()->create_service<dsr_msgs2::srv::GetWorkpieceWeight>("force/get_workpiece_weight", get_workpiece_weight_cb);          
  m_nh_srv_reset_workpiece_weight         = get_node()->create_service<dsr_msgs2::srv::ResetWorkpieceWeight>("force/reset_workpiece_weight", reset_workpiece_weight_cb);          

  //  IO
  m_nh_srv_set_ctrl_box_digital_output        = get_node()->create_service<dsr_msgs2::srv::SetCtrlBoxDigitalOutput>("io/set_ctrl_box_digital_output", set_digital_output_cb);    
  m_nh_srv_get_ctrl_box_digital_input         = get_node()->create_service<dsr_msgs2::srv::GetCtrlBoxDigitalInput>("io/get_ctrl_box_digital_input", get_digital_input_cb);   
  m_nh_srv_set_tool_digital_output            = get_node()->create_service<dsr_msgs2::srv::SetToolDigitalOutput>("io/set_tool_digital_output", set_tool_digital_output_cb);   
  m_nh_srv_get_tool_digital_input             = get_node()->create_service<dsr_msgs2::srv::GetToolDigitalInput>("io/get_tool_digital_input", get_tool_digital_input_cb);   
  m_nh_srv_set_ctrl_box_analog_output         = get_node()->create_service<dsr_msgs2::srv::SetCtrlBoxAnalogOutput>("io/set_ctrl_box_analog_output", set_analog_output_cb);   
  m_nh_srv_get_ctrl_box_analog_input          = get_node()->create_service<dsr_msgs2::srv::GetCtrlBoxAnalogInput>("io/get_ctrl_box_analog_input", get_analog_input_cb);   
  m_nh_srv_set_ctrl_box_analog_output_type    = get_node()->create_service<dsr_msgs2::srv::SetCtrlBoxAnalogOutputType>("io/set_ctrl_box_analog_output_type", set_analog_output_type_cb);   
  m_nh_srv_set_ctrl_box_analog_input_type     = get_node()->create_service<dsr_msgs2::srv::SetCtrlBoxAnalogInputType>("io/set_ctrl_box_analog_input_type", set_analog_input_type_cb);       
  m_nh_srv_get_ctrl_box_digital_output        = get_node()->create_service<dsr_msgs2::srv::GetCtrlBoxDigitalOutput>("io/get_ctrl_box_digital_output", get_digital_output_cb);   
  m_nh_srv_get_tool_digital_output            = get_node()->create_service<dsr_msgs2::srv::GetToolDigitalOutput>("io/get_tool_digital_output", get_tool_digital_output_cb);   

  //  Modbus
  m_nh_srv_set_modbus_output      = get_node()->create_service<dsr_msgs2::srv::SetModbusOutput>("modbus/set_modbus_output", set_modbus_output_cb);    
  m_nh_srv_get_modbus_input       = get_node()->create_service<dsr_msgs2::srv::GetModbusInput>("modbus/get_modbus_input", get_modbus_input_cb);    
  m_nh_srv_config_create_modbus   = get_node()->create_service<dsr_msgs2::srv::ConfigCreateModbus>("modbus/config_create_modbus", config_create_modbus_cb);
  m_nh_srv_config_delete_modbus   = get_node()->create_service<dsr_msgs2::srv::ConfigDeleteModbus>("modbus/config_delete_modbus", config_delete_modbus_cb);

  //  TCP
  m_nh_srv_config_create_tcp      = get_node()->create_service<dsr_msgs2::srv::ConfigCreateTcp>("tcp/config_create_tcp", config_create_tcp_cb);    
  m_nh_srv_config_delete_tcp      = get_node()->create_service<dsr_msgs2::srv::ConfigDeleteTcp>("tcp/config_delete_tcp", config_delete_tcp_cb);  
  m_nh_srv_get_current_tcp        = get_node()->create_service<dsr_msgs2::srv::GetCurrentTcp>("tcp/get_current_tcp", get_current_tcp_cb);       
  m_nh_srv_set_current_tcp        = get_node()->create_service<dsr_msgs2::srv::SetCurrentTcp>("tcp/set_current_tcp", set_current_tcp_cb);       

  //  Tool
  m_nh_srv_config_create_tool     = get_node()->create_service<dsr_msgs2::srv::ConfigCreateTool>("tool/config_create_tool", config_create_tool_cb); 
  m_nh_srv_config_delete_tool     = get_node()->create_service<dsr_msgs2::srv::ConfigDeleteTool>("tool/config_delete_tool", config_delete_tool_cb);    
  m_nh_srv_get_current_tool       = get_node()->create_service<dsr_msgs2::srv::GetCurrentTool>("tool/get_current_tool", get_current_tool_cb);     
  m_nh_srv_set_current_tool       = get_node()->create_service<dsr_msgs2::srv::SetCurrentTool>("tool/set_current_tool", set_current_tool_cb);     
  m_nh_srv_set_tool_shape         = get_node()->create_service<dsr_msgs2::srv::SetToolShape>("tool/set_tool_shape", set_tool_shape_cb); 

  //  DRL
  m_nh_srv_drl_pause              = get_node()->create_service<dsr_msgs2::srv::DrlPause>("drl/drl_pause", drl_pause_cb);                         
  m_nh_srv_drl_start              = get_node()->create_service<dsr_msgs2::srv::DrlStart>("drl/drl_start", drl_start_cb);    
  m_nh_srv_drl_stop               = get_node()->create_service<dsr_msgs2::srv::DrlStop>("drl/drl_stop", drl_stop_cb);    
  m_nh_srv_drl_resume             = get_node()->create_service<dsr_msgs2::srv::DrlResume>("drl/drl_resume", drl_resume_cb);        
  m_nh_srv_get_drl_state          = get_node()->create_service<dsr_msgs2::srv::GetDrlState>("drl/get_drl_state", get_drl_state_cb);       

  // RT
  m_nh_connect_rt_control = get_node()->create_service<dsr_msgs2::srv::ConnectRtControl>("realtime/connect_rt_control", connect_rt_control_cb);
  m_nh_disconnect_rt_control = get_node()->create_service<dsr_msgs2::srv::DisconnectRtControl>("realtime/disconnect_rt_control", disconnect_rt_control_cb);
  m_nh_get_rt_control_output_version_list = get_node()->create_service<dsr_msgs2::srv::GetRtControlOutputVersionList>("realtime/get_rt_control_output_version_list", get_rt_control_output_version_list_cb);
  m_nh_get_rt_control_input_version_list = get_node()->create_service<dsr_msgs2::srv::GetRtControlInputVersionList>("realtime/get_rt_control_input_version_list", get_rt_control_input_version_list_cb);
  m_nh_get_rt_control_input_data_list = get_node()->create_service<dsr_msgs2::srv::GetRtControlInputDataList>("realtime/get_rt_control_input_data_list", get_rt_control_input_data_list_cb);
  m_nh_get_rt_control_output_data_list = get_node()->create_service<dsr_msgs2::srv::GetRtControlOutputDataList>("realtime/get_rt_control_output_data_list", get_rt_control_output_data_list_cb);
  m_nh_set_rt_control_input = get_node()->create_service<dsr_msgs2::srv::SetRtControlInput>("realtime/set_rt_control_input", set_rt_control_input_cb);
  m_nh_set_rt_control_output = get_node()->create_service<dsr_msgs2::srv::SetRtControlOutput>("realtime/set_rt_control_output", set_rt_control_output_cb);
  m_nh_start_rt_control = get_node()->create_service<dsr_msgs2::srv::StartRtControl>("realtime/start_rt_control", start_rt_control_cb);
  m_nh_stop_rt_control = get_node()->create_service<dsr_msgs2::srv::StopRtControl>("realtime/stop_rt_control", stop_rt_control_cb);
  m_nh_set_velj_rt = get_node()->create_service<dsr_msgs2::srv::SetVeljRt>("realtime/set_velj_rt", set_velj_rt_cb);
  m_nh_set_accj_rt = get_node()->create_service<dsr_msgs2::srv::SetAccjRt>("realtime/set_accj_rt", set_accj_rt_cb);
  m_nh_set_velx_rt = get_node()->create_service<dsr_msgs2::srv::SetVelxRt>("realtime/set_velx_rt", set_velx_rt_cb);
  m_nh_set_accx_rt = get_node()->create_service<dsr_msgs2::srv::SetAccxRt>("realtime/set_accx_rt", set_accx_rt_cb);
  m_nh_read_data_rt = get_node()->create_service<dsr_msgs2::srv::ReadDataRt>("realtime/read_data_rt", read_data_rt_cb);
  m_nh_write_data_rt = get_node()->create_service<dsr_msgs2::srv::WriteDataRt>("realtime/write_data_rt", write_data_rt_cb);

  memset(&g_stDrState, 0x00, sizeof(DR_STATE)); 

  // // create threads     

  g_nAnalogOutputModeCh1 = -1;
  g_nAnalogOutputModeCh2 = -1;
  

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"on deactivate");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_error(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"on deactivate");
  return CallbackReturn::SUCCESS;
}





}  // namespace dsr_controller2


namespace DRFL_CALLBACKS{
	//----- register the call-back functions ----------------------------------------
void OnTpInitializingCompletedCB()
{
    // request control authority after TP initialized
    // cout << "[callback OnTpInitializingCompletedCB] tp initializing completed" << endl;
    g_bTpInitailizingComplted = TRUE;
    //Drfl->manage_access_control(MANAGE_ACCESS_CONTROL_REQUEST);
    // Drfl->manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

    g_stDrState.bTpInitialized = TRUE;
}


void OnHommingCompletedCB()
{
    g_bHommingCompleted = TRUE;
    // Only work within 50msec
    // cout << "[callback OnHommingCompletedCB] homming completed" << endl;

    g_stDrState.bHommingCompleted = TRUE;
}

void OnProgramStoppedCB(const PROGRAM_STOP_CAUSE /*iStopCause*/)
{
    // cout << "[callback OnProgramStoppedCB] Program Stop: " << (int)iStopCause << endl;
    g_stDrState.bDrlStopped = TRUE;
}
// M2.4 or lower
void OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO)
{
    for (int i = 0; i < NUM_DIGITAL; i++){
        if(pCtrlIO){  
            g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];  
            g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];  
        }
    }
}

// M3.0 or higher
// void OnMonitoringCtrlIOEx2CB (const LPMONITORING_CTRLIO_EX2 pCtrlIO) 
// {
//     //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"OnMonitoringCtrlIOExCB");

//     for (int i = 0; i < NUM_DIGITAL; i++){
//         if(pCtrlIO){  
//             g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];  
//             g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];  
//         }
//     }

//     //----- In M3.0 version or higher The following variables were added -----
//     for (int i = 0; i < 3; i++)
//         g_stDrState.bActualSW[i] = pCtrlIO->_tInput._iActualSW[i];

//     for (int i = 0; i < 2; i++){
//         g_stDrState.bActualSI[i] = pCtrlIO->_tInput._iActualSI[i];
//         g_stDrState.fActualAI[i] = pCtrlIO->_tInput._fActualAI[i];
//         g_stDrState.iActualAT[i] = pCtrlIO->_tInput._iActualAT[i];
//         g_stDrState.fTargetAO[i] = pCtrlIO->_tOutput._fTargetAO[i];
//         g_stDrState.iTargetAT[i] = pCtrlIO->_tOutput._iTargetAT[i];
//         g_stDrState.bActualES[i] = pCtrlIO->_tEncoder._iActualES[i];
//         g_stDrState.iActualED[i] = pCtrlIO->_tEncoder._iActualED[i];
//         g_stDrState.bActualER[i] = pCtrlIO->_tEncoder._iActualER[i];
//     }  
//     //-------------------------------------------------------------------------
// }

// M2.5 or higher
void OnMonitoringCtrlIOExCB (const LPMONITORING_CTRLIO_EX pCtrlIO) 
{
    //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"OnMonitoringCtrlIOExCB");

    for (int i = 0; i < NUM_DIGITAL; i++){
        if(pCtrlIO){  
            g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];  
            g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];  
        }
    }

    //----- In M2.5 version or higher The following variables were added -----
    for (int i = 0; i < 3; i++)
        g_stDrState.bActualSW[i] = pCtrlIO->_tInput._iActualSW[i];

    for (int i = 0; i < 2; i++){
        g_stDrState.bActualSI[i] = pCtrlIO->_tInput._iActualSI[i];
        g_stDrState.fActualAI[i] = pCtrlIO->_tInput._fActualAI[i];
        g_stDrState.iActualAT[i] = pCtrlIO->_tInput._iActualAT[i];
        g_stDrState.fTargetAO[i] = pCtrlIO->_tOutput._fTargetAO[i];
        g_stDrState.iTargetAT[i] = pCtrlIO->_tOutput._iTargetAT[i];
        g_stDrState.bActualES[i] = pCtrlIO->_tEncoder._iActualES[i];
        g_stDrState.iActualED[i] = pCtrlIO->_tEncoder._iActualED[i];
        g_stDrState.bActualER[i] = pCtrlIO->_tEncoder._iActualER[i];
    }  
    //-------------------------------------------------------------------------
}

// M2.4 or lower
void OnMonitoringDataCB(const LPMONITORING_DATA pData)
{
    // This function is called every 100 msec
    // Only work within 50msec
    //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"OnMonitoringDataCB");

    g_stDrState.nActualMode  = pData->_tCtrl._tState._iActualMode;                  // position control: 0, torque control: 1 ?????
    g_stDrState.nActualSpace = pData->_tCtrl._tState._iActualSpace;                 // joint space: 0, task space: 1    

    for (int i = 0; i < NUM_JOINT; i++){
        if(pData){  
            // joint         
            g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];     // Position Actual Value in INC     
            g_stDrState.fCurrentVelj[i] = pData->_tCtrl._tJoint._fActualVel[i];     // Velocity Actual Value
            g_stDrState.fJointAbs[i]    = pData->_tCtrl._tJoint._fActualAbs[i];     // Position Actual Value in ABS
            g_stDrState.fJointErr[i]    = pData->_tCtrl._tJoint._fActualErr[i];     // Joint Error
            g_stDrState.fTargetPosj[i]  = pData->_tCtrl._tJoint._fTargetPos[i];     // Target Position
            g_stDrState.fTargetVelj[i]  = pData->_tCtrl._tJoint._fTargetVel[i];     // Target Velocity
            // task
            g_stDrState.fCurrentPosx[i]     = pData->_tCtrl._tTask._fActualPos[0][i];   //????? <---------이것 2개다 확인할 것  
            g_stDrState.fCurrentToolPosx[i] = pData->_tCtrl._tTask._fActualPos[1][i];   //????? <---------이것 2개다 확인할 것  
            g_stDrState.fCurrentVelx[i] = pData->_tCtrl._tTask._fActualVel[i];      // Velocity Actual Value
            g_stDrState.fTaskErr[i]     = pData->_tCtrl._tTask._fActualErr[i];      // Task Error
            g_stDrState.fTargetPosx[i]  = pData->_tCtrl._tTask._fTargetPos[i];      // Target Position
            g_stDrState.fTargetVelx[i]  = pData->_tCtrl._tTask._fTargetVel[i];      // Target Velocity
            // Torque
            g_stDrState.fDynamicTor[i]  = pData->_tCtrl._tTorque._fDynamicTor[i];   // Dynamics Torque
            g_stDrState.fActualJTS[i]   = pData->_tCtrl._tTorque._fActualJTS[i];    // Joint Torque Sensor Value
            g_stDrState.fActualEJT[i]   = pData->_tCtrl._tTorque._fActualEJT[i];    // External Joint Torque
            g_stDrState.fActualETT[i]   = pData->_tCtrl._tTorque._fActualETT[i];    // External Task Force/Torque

            g_stDrState.nActualBK[i]    = pData->_tMisc._iActualBK[i];              // brake state     
            g_stDrState.fActualMC[i]    = pData->_tMisc._fActualMC[i];              // motor input current
            g_stDrState.fActualMT[i]    = pData->_tMisc._fActualMT[i];              // motor current temperature
        }
    }
    g_stDrState.nSolutionSpace  = pData->_tCtrl._tTask._iSolutionSpace;             // Solution Space
    g_stDrState.dSyncTime       = pData->_tMisc._dSyncTime;                         // inner clock counter  

    for (int i = 5; i < NUM_BUTTON; i++){
        if(pData){
            g_stDrState.nActualBT[i]    = pData->_tMisc._iActualBT[i];              // robot button state
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(pData){
                g_stDrState.fRotationMatrix[j][i] = pData->_tCtrl._tTask._fRotationMatrix[j][i];    // Rotation Matrix
            }
        }
    }

    for (int i = 0; i < NUM_FLANGE_IO; i++){
        if(pData){
            g_stDrState.bFlangeDigitalInput[i]  = pData->_tMisc._iActualDI[i];      // Digital Input data             
            g_stDrState.bFlangeDigitalOutput[i] = pData->_tMisc._iActualDO[i];      // Digital output data
        }
    }
}

// M2.5 or higher    
void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData)
{
    // This function is called every 100 msec
    // Only work within 50msec
    // RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    OnMonitoringDataExCB");

    g_stDrState.nActualMode  = pData->_tCtrl._tState._iActualMode;                  // position control: 0, torque control: 1 ?????
    g_stDrState.nActualSpace = pData->_tCtrl._tState._iActualSpace;                 // joint space: 0, task space: 1    

    for (int i = 0; i < NUM_JOINT; i++){
        if(pData){  
            // joint         
            g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];     // Position Actual Value in INC     
            g_stDrState.fCurrentVelj[i] = pData->_tCtrl._tJoint._fActualVel[i];     // Velocity Actual Value
            g_stDrState.fJointAbs[i]    = pData->_tCtrl._tJoint._fActualAbs[i];     // Position Actual Value in ABS
            g_stDrState.fJointErr[i]    = pData->_tCtrl._tJoint._fActualErr[i];     // Joint Error
            g_stDrState.fTargetPosj[i]  = pData->_tCtrl._tJoint._fTargetPos[i];     // Target Position
            g_stDrState.fTargetVelj[i]  = pData->_tCtrl._tJoint._fTargetVel[i];     // Target Velocity
            // task
            g_stDrState.fCurrentPosx[i]     = pData->_tCtrl._tTask._fActualPos[0][i];   //????? <---------이것 2개다 확인할 것  
            g_stDrState.fCurrentToolPosx[i] = pData->_tCtrl._tTask._fActualPos[1][i];   //????? <---------이것 2개다 확인할 것  
            g_stDrState.fCurrentVelx[i] = pData->_tCtrl._tTask._fActualVel[i];      // Velocity Actual Value
            g_stDrState.fTaskErr[i]     = pData->_tCtrl._tTask._fActualErr[i];      // Task Error
            g_stDrState.fTargetPosx[i]  = pData->_tCtrl._tTask._fTargetPos[i];      // Target Position
            g_stDrState.fTargetVelx[i]  = pData->_tCtrl._tTask._fTargetVel[i];      // Target Velocity
            // Torque
            g_stDrState.fDynamicTor[i]  = pData->_tCtrl._tTorque._fDynamicTor[i];   // Dynamics Torque
            g_stDrState.fActualJTS[i]   = pData->_tCtrl._tTorque._fActualJTS[i];    // Joint Torque Sensor Value
            g_stDrState.fActualEJT[i]   = pData->_tCtrl._tTorque._fActualEJT[i];    // External Joint Torque
            g_stDrState.fActualETT[i]   = pData->_tCtrl._tTorque._fActualETT[i];    // External Task Force/Torque

            g_stDrState.nActualBK[i]    = pData->_tMisc._iActualBK[i];              // brake state     
            g_stDrState.fActualMC[i]    = pData->_tMisc._fActualMC[i];              // motor input current
            g_stDrState.fActualMT[i]    = pData->_tMisc._fActualMT[i];              // motor current temperature
        }
    }
    g_stDrState.nSolutionSpace  = pData->_tCtrl._tTask._iSolutionSpace;             // Solution Space
    g_stDrState.dSyncTime       = pData->_tMisc._dSyncTime;                         // inner clock counter  

    for (int i = 5; i < NUM_BUTTON; i++){
        if(pData){
            g_stDrState.nActualBT[i]    = pData->_tMisc._iActualBT[i];              // robot button state
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(pData){
                g_stDrState.fRotationMatrix[j][i] = pData->_tCtrl._tTask._fRotationMatrix[j][i];    // Rotation Matrix
            }
        }
    }

    for (int i = 0; i < NUM_FLANGE_IO; i++){
        if(pData){
            g_stDrState.bFlangeDigitalInput[i]  = pData->_tMisc._iActualDI[i];      // Digital Input data             
            g_stDrState.bFlangeDigitalOutput[i] = pData->_tMisc._iActualDO[i];      // Digital output data
        }
    }

    //----- In M2.5 version or higher The following variables were added -----
    for (int i = 0; i < NUM_JOINT; i++){
        g_stDrState.fActualW2B[i] = pData->_tCtrl._tWorld._fActualW2B[i];
        g_stDrState.fCurrentVelW[i] = pData->_tCtrl._tWorld._fActualVel[i];
        g_stDrState.fWorldETT[i] = pData->_tCtrl._tWorld._fActualETT[i];
        g_stDrState.fTargetPosW[i] = pData->_tCtrl._tWorld._fTargetPos[i];
        g_stDrState.fTargetVelW[i] = pData->_tCtrl._tWorld._fTargetVel[i];
        g_stDrState.fCurrentVelU[i] = pData->_tCtrl._tWorld._fActualVel[i];
        g_stDrState.fUserETT[i] = pData->_tCtrl._tUser._fActualETT[i];
        g_stDrState.fTargetPosU[i] = pData->_tCtrl._tUser._fTargetPos[i];
        g_stDrState.fTargetVelU[i] = pData->_tCtrl._tUser._fTargetVel[i];
    }    

    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 6; j++){
            g_stDrState.fCurrentPosW[i][j] = pData->_tCtrl._tWorld._fActualPos[i][j];
            g_stDrState.fCurrentPosU[i][j] = pData->_tCtrl._tUser._fActualPos[i][j];
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            g_stDrState.fRotationMatrixWorld[j][i] = pData->_tCtrl._tWorld._fRotationMatrix[j][i];
            g_stDrState.fRotationMatrixUser[j][i] = pData->_tCtrl._tUser._fRotationMatrix[j][i];
        }
    }

    g_stDrState.iActualUCN = pData->_tCtrl._tUser._iActualUCN;
    g_stDrState.iParent    = pData->_tCtrl._tUser._iParent;
    //-------------------------------------------------------------------------
}

void OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus)
{
    g_stDrState.nRegCount = pModbus->_iRegCount;
    for (int i = 0; i < pModbus->_iRegCount; i++){
        // cout << "[callback OnMonitoringModbusCB] " << pModbus->_tRegister[i]._szSymbol <<": " << pModbus->_tRegister[i]._iValue<< endl;
        g_stDrState.strModbusSymbol[i] = pModbus->_tRegister[i]._szSymbol;
        g_stDrState.nModbusValue[i]    = pModbus->_tRegister[i]._iValue;
    }
}

void OnMonitoringStateCB(const ROBOT_STATE eState)
{

    switch((unsigned char)eState)
    {  
    case STATE_EMERGENCY_STOP:
        // popup
        break;
    case STATE_STANDBY:
    case STATE_MOVING:
    case STATE_TEACHING:
        break;
    case STATE_SAFE_STOP:
        if (g_bHasControlAuthority) {
            Drfl->set_safe_stop_reset_type(SAFE_STOP_RESET_TYPE_DEFAULT);
            Drfl->set_robot_control(CONTROL_RESET_SAFET_STOP);
            // Drfl->set_safety_mode(SAFETY_MODE_AUTONOMOUS,SAFETY_MODE_EVENT_MOVE);
        }
        break;
    case STATE_SAFE_OFF:
        if (g_bHasControlAuthority){
            if (init_state){
            Drfl->set_robot_control(CONTROL_SERVO_ON);
            Drfl->set_robot_mode(ROBOT_MODE_AUTONOMOUS);   //Idle Servo Off 후 servo on 하는 상황 발생 시 set_robot_mode 명령을 전송해 manual 로 전환. add 2020/04/28
            // Drfl->set_safety_mode(SAFETY_MODE_AUTONOMOUS,SAFETY_MODE_EVENT_MOVE);
            init_state = FALSE;
            }
        } 
        break;
    case STATE_SAFE_STOP2:
        if (g_bHasControlAuthority) Drfl->set_robot_control(CONTROL_RECOVERY_SAFE_STOP);
        break;
    case STATE_SAFE_OFF2:
        if (g_bHasControlAuthority) {
            Drfl->set_robot_control(CONTROL_RECOVERY_SAFE_OFF);
        }
        break;
    case STATE_RECOVERY:
        Drfl->set_robot_control(CONTROL_RESET_RECOVERY);
        break;
    default:
        break;
    }

    // cout << "[callback OnMonitoringStateCB] current state: " << GetRobotStateString((int)eState) << endl;
    g_stDrState.nRobotState = (int)eState;
    strncpy(g_stDrState.strRobotState, GetRobotStateString((int)eState), MAX_SYMBOL_SIZE); 
}

void OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl)
{
    // Only work within 50msec

    // cout << "[callback OnMonitoringAccessControlCB] eAccCtrl: " << eAccCtrl << endl;
    switch(eAccCtrl)
    {
    case MONITORING_ACCESS_CONTROL_REQUEST:
        Drfl->ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO);
        //Drfl->TransitControlAuth(MANaGE_ACCESS_CONTROL_RESPONSE_YES);
        break;
    case MONITORING_ACCESS_CONTROL_GRANT:
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"_______________________________________________\n");   
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    Access control granted ");
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"_______________________________________________\n");   
        g_bHasControlAuthority = TRUE;
        OnMonitoringStateCB(Drfl->GetRobotState());
        break;
    case MONITORING_ACCESS_CONTROL_DENY:
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"_______________________________________________\n");   
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    Access control deny ");
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"_______________________________________________\n");   
        break;
    case MONITORING_ACCESS_CONTROL_LOSS:
        g_bHasControlAuthority = FALSE;
        if (g_bTpInitailizingComplted) {
            Drfl->ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST);
        }
        break;
    default:
        break;
    }
    g_stDrState.nAccessControl = (int)eAccCtrl;
}

void OnLogAlarm(LPLOG_ALARM pLogAlarm)
{
	dsr_msgs2::msg::RobotError msg;
	switch(pLogAlarm->_iLevel)
	{
	case LOG_LEVEL_SYSINFO:
			RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"[callback OnLogAlarm]");
			RCLCPP_INFO(rclcpp::get_logger("dsr_controller2")," level : %d",(unsigned int)pLogAlarm->_iLevel);
			RCLCPP_INFO(rclcpp::get_logger("dsr_controller2")," group : %d",(unsigned int)pLogAlarm->_iGroup);
			RCLCPP_INFO(rclcpp::get_logger("dsr_controller2")," index : %d", pLogAlarm->_iIndex);
			RCLCPP_INFO(rclcpp::get_logger("dsr_controller2")," param : %s", pLogAlarm->_szParam[0] );
			RCLCPP_INFO(rclcpp::get_logger("dsr_controller2")," param : %s", pLogAlarm->_szParam[1] );
			RCLCPP_INFO(rclcpp::get_logger("dsr_controller2")," param : %s", pLogAlarm->_szParam[2] );
			break;
	case LOG_LEVEL_SYSWARN:
			RCLCPP_WARN(rclcpp::get_logger("dsr_controller2"),"[callback OnLogAlarm]");
			RCLCPP_WARN(rclcpp::get_logger("dsr_controller2")," level : %d",(unsigned int)pLogAlarm->_iLevel);
			RCLCPP_WARN(rclcpp::get_logger("dsr_controller2")," group : %d",(unsigned int)pLogAlarm->_iGroup);
			RCLCPP_WARN(rclcpp::get_logger("dsr_controller2")," index : %d", pLogAlarm->_iIndex);
			RCLCPP_WARN(rclcpp::get_logger("dsr_controller2")," param : %s", pLogAlarm->_szParam[0] );
			RCLCPP_WARN(rclcpp::get_logger("dsr_controller2")," param : %s", pLogAlarm->_szParam[1] );
			RCLCPP_WARN(rclcpp::get_logger("dsr_controller2")," param : %s", pLogAlarm->_szParam[2] );
			break;
	case LOG_LEVEL_SYSERROR:
	default:
			RCLCPP_ERROR(rclcpp::get_logger("dsr_controller2"),"[callback OnLogAlarm]");
			RCLCPP_ERROR(rclcpp::get_logger("dsr_controller2")," level : %d",(unsigned int)pLogAlarm->_iLevel);
			RCLCPP_ERROR(rclcpp::get_logger("dsr_controller2")," group : %d",(unsigned int)pLogAlarm->_iGroup);
			RCLCPP_ERROR(rclcpp::get_logger("dsr_controller2")," index : %d", pLogAlarm->_iIndex);
			RCLCPP_ERROR(rclcpp::get_logger("dsr_controller2")," param : %s", pLogAlarm->_szParam[0] );
			RCLCPP_ERROR(rclcpp::get_logger("dsr_controller2")," param : %s", pLogAlarm->_szParam[1] );
			RCLCPP_ERROR(rclcpp::get_logger("dsr_controller2")," param : %s", pLogAlarm->_szParam[2] );
			break;
	}
	g_stDrError.nLevel=(unsigned int)pLogAlarm->_iLevel;
	g_stDrError.nGroup=(unsigned int)pLogAlarm->_iGroup;
	g_stDrError.nCode=pLogAlarm->_iIndex;
	strncpy(g_stDrError.strMsg1, pLogAlarm->_szParam[0], MAX_STRING_SIZE);
	strncpy(g_stDrError.strMsg2, pLogAlarm->_szParam[1], MAX_STRING_SIZE);
	strncpy(g_stDrError.strMsg3, pLogAlarm->_szParam[2], MAX_STRING_SIZE);

	msg.level=g_stDrError.nLevel;
	msg.group=g_stDrError.nGroup;
	msg.code=g_stDrError.nCode;
	msg.msg1=g_stDrError.strMsg1;
	msg.msg2=g_stDrError.strMsg2;
	msg.msg3=g_stDrError.strMsg3;

	if(0 == instance->error_log_pub_.use_count())	return;
	instance->error_log_pub_->publish(msg);
}

void OnDisConnected(){
	RCLCPP_ERROR(rclcpp::get_logger("dsr_controller2"),"Disconnected.. Please check out Ethernet Cable.. ");
	Drfl->stop_rt_control();
    // To-do : Update disconnection function in controller version v3.6
    // Drfl->disconnect_rt_control();
	Drfl->close_connection(); // clean-up

	
	if(0 == instance->disconnect_pub_.use_count())	return;
	dsr_msgs2::msg::RobotDisconnection msg;
	instance->disconnect_pub_->publish(msg);
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dsr_controller2::RobotController, controller_interface::ControllerInterface)
