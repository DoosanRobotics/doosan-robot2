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
#include "../../common2/include/DRFLEx.h"

using config_type = controller_interface::interface_configuration_type;
using namespace DRAFramework;

extern void* get_drfl();
CDRFLEx *Drfl = (CDRFLEx*)get_drfl();
rclcpp::Node::SharedPtr m_node_ = nullptr;

bool g_bIsEmulatorMode = FALSE;
bool g_bHasControlAuthority = FALSE;
bool g_bTpInitailizingComplted = FALSE;
bool g_bHommingCompleted = FALSE;

ROBOT_JOINT_DATA g_joints[NUM_JOINT];

DR_STATE    g_stDrState;
DR_ERROR    g_stDrError;

int g_nAnalogOutputModeCh1;
int g_nAnalogOutputModeCh2;
int m_nVersionDRCF;


namespace dsr_controller2
{
RobotController::RobotController() : controller_interface::ControllerInterface() {}


controller_interface::CallbackReturn RobotController::on_init()
{
    // YAML 파일 경로 설정
    // std::string package_directory = ament_index_cpp::get_package_share_directory("dsr_hardware2");
    // std::string yaml_file_path = package_directory + "/config/parameters.yaml";
    
    // std::ifstream fin(yaml_file_path);
    // if (!fin) {
    //     RCLCPP_ERROR(rclcpp::get_logger("dsr_controller2"), "Failed to open YAML file: %s", yaml_file_path.c_str());
    // }

    // // YAML 파일 파싱
    // YAML::Node yaml_node = YAML::Load(fin);
    // fin.close();

    // // 파싱된 YAML 노드에서 파라미터 읽기
    // if (yaml_node["name"]) {
    //     m_name = yaml_node["name"].as<std::string>();
    //     RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"), "name: %s", m_name.c_str());
    // }
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
    

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
{
//   m_node_ = rclcpp::Node::make_shared("dsr_test");
//   auto callback =
//     [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
//   {
//     traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
//     new_msg_ = true;
//   };

    auto callback =
        [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
    {
        traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
        new_msg_ = true;
    };

    joint_command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>("~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);
      
auto set_robot_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetRobotMode::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotMode::Response> res) -> bool
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_robot_mode_cb() called and calling Drfl->set_robot_mode(%d)",req->robot_mode);

    res->success = Drfl->set_robot_mode((ROBOT_MODE)req->robot_mode); 
    return true;
};

auto get_robot_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRobotMode::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetRobotMode::Response> res) -> bool
{       
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_robot_mode_cb() called and calling Drfl->get_robot_mode()");
    res->robot_mode = Drfl->get_robot_mode();
    res->success = true;
    return true;
};    
auto set_robot_system_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetRobotSystem::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotSystem::Response> res) -> bool
{ 
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_robot_system_cb() called and calling Drfl->set_robot_system(%d)",req->robot_system);

    res->success = Drfl->set_robot_system((ROBOT_SYSTEM)req->robot_system);
    return true;
};        
auto get_robot_system_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRobotSystem::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetRobotSystem::Response> res) -> bool
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_robot_system_cb() called and calling Drfl->get_robot_system()");

    res->robot_system = Drfl->get_robot_system();
    res->success = true;
    return true;
};
auto get_robot_state_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRobotState::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetRobotState::Response> res) -> bool        
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_robot_state_cb() called and calling Drfl->get_robot_state()");

    res->robot_state = Drfl->get_robot_state();
    res->success = true;
    return true;
};
auto set_robot_speed_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetRobotSpeedMode::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotSpeedMode::Response> res) -> bool    
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_robot_speed_mode_cb() called and calling Drfl->set_robot_speed_mode(%d)",req->speed_mode);

    res->success = Drfl->set_robot_speed_mode((SPEED_MODE)req->speed_mode);
    return true;
};
auto get_robot_speed_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRobotSpeedMode::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetRobotSpeedMode::Response> res) -> bool       
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_robot_speed_mode_cb() called and calling Drfl->get_robot_speed_mode()");

    res->speed_mode = Drfl->get_robot_speed_mode();
    res->success = true;
    return true;
};
auto get_current_pose_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentPose::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentPose::Response> res) -> bool   
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_current_pose_cb() called and calling Drfl->get_current_pose(%d)",req->space_type);

    LPROBOT_POSE robot_pos = Drfl->get_current_pose((ROBOT_SPACE)req->space_type);
    for(int i = 0; i < NUM_TASK; i++){
        res->pos[i] = robot_pos->_fPosition[i];
    }
    res->success = true;
    return true;
};
auto set_safe_stop_reset_type_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetSafeStopResetType::Request> req, std::shared_ptr<dsr_msgs2::srv::SetSafeStopResetType::Response> res) -> bool     
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_safe_stop_reset_type_cb() called and calling Drfl->SetSafeStopResetType(%d)",req->reset_type);
    Drfl->SetSafeStopResetType((SAFE_STOP_RESET_TYPE)req->reset_type);
    res->success = true;                           
    return true;
};
auto get_last_alarm_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetLastAlarm::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetLastAlarm::Response> res) -> bool   
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
    return true;
};


//----- MOTION Service Call-back functions ------------------------------------------------------------
auto movej_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveJoint::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveJoint::Response> res) -> bool
{   
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movej_cb() %p", Drfl);
    res->success = false;
    std::array<float, NUM_JOINT> target_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
    if(req->sync_type == 0){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movej_cb() called and calling Drfl->movej");
        res->success = Drfl->movej(target_pos.data(), req->vel, req->acc, req->time, (MOVE_MODE)req->mode, req->radius, (BLENDING_SPEED_TYPE)req->blend_type);   
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movej_cb() called and calling Drfl->amovej");
        res->success = Drfl->amovej(target_pos.data(), req->vel, req->acc, req->time, (MOVE_MODE)req->mode, (BLENDING_SPEED_TYPE)req->blend_type);
    }
    return true;
};

auto movel_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveLine::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveLine::Response> res) -> bool
{
    res->success = false;
    std::array<float, NUM_JOINT> target_pos;
    std::array<float, 2> target_vel;
    std::array<float, 2> target_acc;
    std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
    std::copy(req->vel.cbegin(), req->vel.cend(), target_vel.begin());
    std::copy(req->acc.cbegin(), req->acc.cend(), target_acc.begin());

    if(req->sync_type == 0){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movel_cb() called and calling Drfl->movel");
        res->success = Drfl->movel(target_pos.data(), target_vel.data(), target_acc.data(), req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, req->radius, (BLENDING_SPEED_TYPE)req->blend_type);   
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movel_cb() called and calling Drfl->amovel");
        res->success = Drfl->amovel(target_pos.data(), target_vel.data(), target_acc.data(), req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (BLENDING_SPEED_TYPE)req->blend_type);
    }
    return true;
};
auto movejx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveJointx::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveJointx::Response> res) -> bool               
{
    res->success = false;
    std::array<float, NUM_TASK> target_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
    if(req->sync_type == 0){
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movejx_cb() called and calling Drfl->movejx");
        res->success = Drfl->movejx(target_pos.data(), req->sol, req->vel, req->acc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, req->radius, (BLENDING_SPEED_TYPE)req->blend_type);    
    }
    else{
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movejx_cb() called and calling Drfl->amovejx");
        res->success = Drfl->amovejx(target_pos.data(), req->sol, req->vel, req->acc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (BLENDING_SPEED_TYPE)req->blend_type);    
    }
    return true;
};
auto movec_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveCircle::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveCircle::Response> res) -> bool       
{
    res->success = false;
    float fTargetPos[2][NUM_TASK];
    float fTargetVel[2];
    float fTargetAcc[2];
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < NUM_TASK; j++){
            std_msgs::msg::Float64MultiArray pos = req->pos.at(i);
            fTargetPos[i][j] = pos.data[j];
        }
        fTargetVel[i] = req->vel[i];
        fTargetAcc[i] = req->acc[i];
    }
    ///RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"  <xxx pos1> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[0][0],fTargetPos[0][1],fTargetPos[0][2],fTargetPos[0][3],fTargetPos[0][4],fTargetPos[0][5]);
    ///RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"  <xxx pos2> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[1][0],fTargetPos[1][1],fTargetPos[1][2],fTargetPos[1][3],fTargetPos[1][4],fTargetPos[1][5]);
    if(req->sync_type == 0){   
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movec_cb() called and calling Drfl->movec");
        res->success = Drfl->movec(fTargetPos, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, req->radius, (BLENDING_SPEED_TYPE)req->blend_type);      
    }
    else{
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movec_cb() called and calling Drfl->amovec");
        res->success = Drfl->amovec(fTargetPos, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (BLENDING_SPEED_TYPE)req->blend_type);  
    }
    return true;
};
auto movesj_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveSplineJoint::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveSplineJoint::Response> res) -> bool      
{
    res->success = false;
    float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT];
    float fTargetVel[NUM_JOINT]= {0.0, };
    float fTargetAcc[NUM_JOINT]= {0.0, };

    for(int i=0; i<req->pos_cnt; i++){
        for(int j=0; j<NUM_JOINT; j++){
            std_msgs::msg::Float64MultiArray pos = req->pos.at(i);
            fTargetPos[i][j] = pos.data[j];
        }
    }
    if(req->sync_type == 0){
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movejx_cb() called and calling Drfl->movesj");
        //res->success = Drfl->MoveSJ(fTargetPos, req->pos_cnt, req->vel, req->acc, req->time, (MOVE_MODE)req->mode);
        res->success = Drfl->movesj(fTargetPos, req->pos_cnt, fTargetVel[0], fTargetAcc[0], req->time, (MOVE_MODE)req->mode); //need updata API
    }
    else{
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movejx_cb() called and calling Drfl->amovesj");
        //res->success = Drfl->MoveSJAsync(fTargetPos, req->pos_cnt, req->vel, req->acc, req->time, (MOVE_MODE)req->mode);
        res->success = Drfl->amovesj(fTargetPos, req->pos_cnt, fTargetVel[0], fTargetAcc[0], req->time, (MOVE_MODE)req->mode); //need updata API
    }
    return true;
};
auto movesx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveSplineTask::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveSplineTask::Response> res) -> bool          
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
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movesx_cb() called and calling Drfl->movesx");
        res->success = Drfl->movesx(fTargetPos, req->pos_cnt, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (SPLINE_VELOCITY_OPTION)req->opt);
    }
    else{
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movesx_cb() called and calling Drfl->amovesx");
        res->success = Drfl->amovesx(fTargetPos, req->pos_cnt, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (SPLINE_VELOCITY_OPTION)req->opt);
    }
    return true;
};
auto moveb_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveBlending::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveBlending::Response> res) -> bool          
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
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"moveb_cb() called and calling Drfl->moveb");
        res->success = Drfl->moveb(posb, req->pos_cnt, target_vel.data(), target_acc.data(), req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref);
    }
    else{
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"moveb_cb() called and calling Drfl->amoveb");
        res->success = Drfl->amoveb(posb, req->pos_cnt, target_vel.data(), target_acc.data(), req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref);
    }
    return true;
};
auto movespiral_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveSpiral::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveSpiral::Response> res) -> bool              
{
    res->success = false;
    std::array<float, 2> target_vel;
    
    std::array<float, 2> target_acc;
    std::copy(req->vel.cbegin(), req->vel.cend(), target_vel.begin());
    std::copy(req->acc.cbegin(), req->acc.cend(), target_acc.begin());

    if(req->sync_type == 0){
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movespiral_cb() called and calling Drfl->move_spiral");
        res->success = Drfl->move_spiral((TASK_AXIS)req->task_axis, req->revolution, req->max_radius, req->max_length, target_vel.data(), target_acc.data(), req->time, (MOVE_REFERENCE)req->ref);
    }
    else{
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movespiral_cb() called and calling Drfl->amove_spiral");
        res->success = Drfl->amove_spiral((TASK_AXIS)req->task_axis, req->revolution, req->max_radius, req->max_length, target_vel.data(), target_acc.data(), req->time, (MOVE_REFERENCE)req->ref);
    }
    return true;
};
auto moveperiodic_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MovePeriodic::Request> req, std::shared_ptr<dsr_msgs2::srv::MovePeriodic::Response> res) -> bool              
{
    res->success = false;
    std::array<float, NUM_TASK> target_amp;
    std::array<float, NUM_TASK> target_periodic;
    std::copy(req->amp.cbegin(), req->amp.cend(), target_amp.begin());
    std::copy(req->periodic.cbegin(), req->periodic.cend(), target_periodic.begin());
    if(req->sync_type == 0){
        //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"moveperiodic_cb() called and calling Drfl->move_periodic");
        res->success = Drfl->move_periodic(target_amp.data(), target_periodic.data(), req->acc, req->repeat, (MOVE_REFERENCE)req->ref);
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"moveperiodic_cb() called and calling Drfl->amove_periodic");
        res->success = Drfl->amove_periodic(target_amp.data(), target_periodic.data(), req->acc, req->repeat, (MOVE_REFERENCE)req->ref);
    }
    return true;
};
auto movewait_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveWait::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::MoveWait::Response> res) -> bool                  
{
    res->success = false;
    //RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"movewait_cb() called and calling Drfl->mwait");
    res->success = Drfl->mwait();
    return true;
};
auto jog_cb = [this](const std::shared_ptr<dsr_msgs2::srv::Jog::Request> req, std::shared_ptr<dsr_msgs2::srv::Jog::Response> res) -> bool              
{
    res->success = false;
    res->success = Drfl->jog((JOG_AXIS)req->jog_axis, (MOVE_REFERENCE)req->move_reference, req->speed);
    return true;
};

auto jog_multi_cb = [this](const std::shared_ptr<dsr_msgs2::srv::JogMulti::Request> req, std::shared_ptr<dsr_msgs2::srv::JogMulti::Response> res) -> bool                      
{
    res->success = false;

    std::array<float, NUM_JOINT> target_jog;
    std::copy(req->jog_axis.cbegin(), req->jog_axis.cend(), target_jog.begin());

    res->success = Drfl->multi_jog(target_jog.data(), (MOVE_REFERENCE)req->move_reference, req->speed);
    return true;
};
auto move_stop_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveStop::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveStop::Response> res) -> bool                  
{
    res->success = false;
    res->success = Drfl->stop((STOP_TYPE)req->stop_mode);
    return true;
};
auto move_resume_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveResume::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::MoveResume::Response> res) -> bool                  
{
    res->success = false;
    res->success = Drfl->move_resume();
    return true;
};
auto move_pause_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MovePause::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::MovePause::Response> res) -> bool                      
{
    res->success = false;
    res->success = Drfl->move_pause();
    return true;
};
auto trans_cb = [this](const std::shared_ptr<dsr_msgs2::srv::Trans::Request> req, std::shared_ptr<dsr_msgs2::srv::Trans::Response> res) -> bool                  
{
    res->success = false;
    std::array<float, NUM_TASK> target_pos;
    std::array<float, NUM_TASK> delta_pos;

    std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
    std::copy(req->delta.cbegin(), req->delta.cend(), delta_pos.begin());

    LPROBOT_POSE robot_pos = Drfl->trans(target_pos.data(), delta_pos.data(), (COORDINATE_SYSTEM)req->ref, (COORDINATE_SYSTEM)req->ref_out);
    for(int i=0; i<NUM_TASK; i++){
        res->trans_pos[i] = robot_pos->_fPosition[i];
    }
    res->success = true;
    return true;
};
auto fkin_cb = [this](const std::shared_ptr<dsr_msgs2::srv::Fkin::Request> req, std::shared_ptr<dsr_msgs2::srv::Fkin::Response> res) -> bool              
{
    res->success = false;
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
    return true;
};
auto ikin_cb = [this](const std::shared_ptr<dsr_msgs2::srv::Ikin::Request> req, std::shared_ptr<dsr_msgs2::srv::Ikin::Response> res) -> bool              
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
    return true;
};
auto set_ref_coord_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetRefCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRefCoord::Response> res) -> bool                  
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< set_ref_coord_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    coord = %d",req->coord);      
#endif

    res->success = Drfl->set_ref_coord((COORDINATE_SYSTEM)req->coord);
    return true;
};
auto move_home_cb = [this](const std::shared_ptr<dsr_msgs2::srv::MoveHome::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveHome::Response> res) -> bool                  
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< move_home_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    target = %d",req->target);      
#endif

    if(0 == req->target) 
        res->success = Drfl->move_home(MOVE_HOME_MECHANIC);
    else 
        res->success = Drfl->move_home(MOVE_HOME_USER);
    return true;
};
auto check_motion_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CheckMotion::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::CheckMotion::Response> res) -> bool                  
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< check_motion_cb >");
#endif

    res->status = Drfl->check_motion();
    res->success = true;
    return true;
};
auto change_operation_speed_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ChangeOperationSpeed::Request> req, std::shared_ptr<dsr_msgs2::srv::ChangeOperationSpeed::Response> res) -> bool        
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< change_operation_speed_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    speed = %f",(float)req->speed);
#endif

    res->success = Drfl->change_operation_speed((float)req->speed);
    return true;
};
auto enable_alter_motion_cb = [this](const std::shared_ptr<dsr_msgs2::srv::EnableAlterMotion::Request> req, std::shared_ptr<dsr_msgs2::srv::EnableAlterMotion::Response> res) -> bool                 
{
    res->success = false;
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
    return true;        
};
auto alter_motion_cb = [this](const std::shared_ptr<dsr_msgs2::srv::AlterMotion::Request> req, std::shared_ptr<dsr_msgs2::srv::AlterMotion::Response> res) -> bool              
{
    res->success = false;
    std::array<float, NUM_TASK> pos_alter;
    std::copy(req->pos.cbegin(), req->pos.cend(), pos_alter.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< alter_motion_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos_alter = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",pos_alter[0],pos_alter[1],pos_alter[2],pos_alter[3],pos_alter[4],pos_alter[5]);
#endif

    res->success = Drfl->alter_motion(pos_alter.data());       
    return true;
};
auto disable_alter_motion_cb = [this](const std::shared_ptr<dsr_msgs2::srv::DisableAlterMotion::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::DisableAlterMotion::Response> res) -> bool              
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< disable_alter_motion_cb >");
#endif

    res->success = Drfl->disable_alter_motion();
    return true;
};
auto set_singularity_handling_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetSingularityHandling::Request> req, std::shared_ptr<dsr_msgs2::srv::SetSingularityHandling::Response> res) -> bool  
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< set_singularity_handling_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    mode = %d",req->mode);
#endif

    res->success = Drfl->set_singularity_handling((SINGULARITY_AVOIDANCE)req->mode);      
    return true;
};



//----- AUXILIARY_CONTROL Service Call-back functions ------------------------------------------------------------
auto get_control_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetControlMode::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetControlMode::Response> res) -> bool                           
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_control_mode_cb >");
#endif
    //NO API , get mon_data      
    res->control_mode = g_stDrState.nActualMode;
    res->success = true;       
    return true;
};
auto get_control_space_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetControlSpace::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetControlSpace::Response> res) -> bool                          
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_control_space_cb >");
#endif
    //NO API , get mon_data
    res->space = g_stDrState.nActualSpace;
    res->success = true;
    return true;
};
auto get_current_posj_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentPosj::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentPosj::Response> res) -> bool                                            
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_current_posj_cb >");
#endif
    LPROBOT_POSE robot_pos = Drfl->GetCurrentPose((ROBOT_SPACE)ROBOT_SPACE_JOINT);
    for(int i = 0; i < NUM_TASK; i++){
        res->pos[i] = robot_pos->_fPosition[i];
    }
    res->success = true;        
    return true;
};
auto get_current_velj_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentVelj::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentVelj::Response> res) -> bool                                
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_current_velj_cb >");
#endif

    //NO API , get mon_data
    for(int i=0; i<NUM_TASK; i++){
        res->joint_speed[i] = g_stDrState.fCurrentVelj[i];
    }
    res->success = true;
    return true;
};
auto get_desired_posj_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetDesiredPosj::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetDesiredPosj::Response> res) -> bool         
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_desired_posj_cb >");
#endif
    //NO API , get mon_data
    for(int i=0; i<NUM_TASK; i++){
        res->pos[i] = g_stDrState.fTargetPosj[i];
    }
    res->success = true;        
    return true;
};
auto get_desired_velj_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetDesiredVelj::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetDesiredVelj::Response> res) -> bool                                   
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_desired_velj_cb >");
#endif
    //NO API , get mon_data
    for(int i=0; i<NUM_TASK; i++){
        res->joint_vel[i] = g_stDrState.fTargetVelj[i];
    }
    res->success = true;        
    return true;
};
auto get_current_posx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentPosx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentPosx::Response> res) -> bool                               
{
    res->success = false;
    std_msgs::msg::Float64MultiArray arr;

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_current_posx_cb >");
#endif

    LPROBOT_TASK_POSE cur_posx = Drfl->get_current_posx((COORDINATE_SYSTEM)req->ref);
    arr.data.clear();
    for (int i = 0; i < NUM_TASK; i++){
        arr.data.push_back(cur_posx->_fTargetPos[i]);
    }
    arr.data.push_back(cur_posx->_iTargetSol);
    res->task_pos_info.push_back(arr);
    res->success = true;
    return true;
};
auto get_current_velx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentVelx::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentVelx::Response> res) -> bool                         
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_current_velx_cb >");
#endif

    //NO API , get mon_data
    for(int i=0; i<NUM_TASK; i++){
        res->vel[i] = g_stDrState.fCurrentVelx[i];
    }
    res->success = true;            
    return true;
};
auto get_desired_posx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetDesiredPosx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetDesiredPosx::Response> res) -> bool     
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_desired_posx_cb >");
#endif
    LPROBOT_POSE task_pos = Drfl->get_desired_posx((COORDINATE_SYSTEM)req->ref);
    for(int i=0; i<NUM_TASK; i++){
        res->pos[i] = task_pos->_fPosition[i];
    }
    res->success = true;        
    return true;
};
auto get_desired_velx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetDesiredVelx::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetDesiredVelx::Response> res) -> bool                             
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_desired_velx_cb >");
#endif
    //NO API , get mon_data
    for(int i=0; i<NUM_TASK; i++){
        res->vel[i] = g_stDrState.fTargetVelx[i];
    }
    res->success = true;                    
    return true;
};
auto get_current_tool_flange_posx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentToolFlangePosx::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentToolFlangePosx::Response> res) -> bool                 
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
    return true;
};
auto get_current_solution_space_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentSolutionSpace::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentSolutionSpace::Response> res) -> bool                    
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_current_solution_space_cb >");
#endif
    res->sol_space = Drfl->get_current_solution_space();
    res->success = true;
    return true;
};
auto get_current_rotm_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentRotm::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentRotm::Response> res) -> bool                     
{
    res->success = false;
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
    return true;
};
auto get_joint_torque_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetJointTorque::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetJointTorque::Response> res) -> bool                     
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_joint_torque_cb >");
#endif
    //NO API , get mon_data
    for(int i = 0; i < NUM_TASK; i++){
        res->jts[i] = g_stDrState.fActualJTS[i];
    }
    res->success = true;        
    return true;
};
auto get_external_torque_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetExternalTorque::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetExternalTorque::Response> res) -> bool                
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_external_torque_cb >");
#endif
    //NO API , get mon_data
    for(int i = 0; i < NUM_TASK; i++){
        res->ext_torque[i] = g_stDrState.fActualEJT[i];
    }
    res->success = true;        
    return true;
};
auto get_tool_force_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetToolForce::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetToolForce::Response> res) -> bool                               
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_tool_force_cb >");
#endif
    //NO API , get mon_data
    for(int i = 0; i < NUM_TASK; i++){
        res->tool_force[i] = g_stDrState.fActualETT[i];
    }
    res->success = true;        
    return true;
};
auto get_solution_space_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetSolutionSpace::Request> req, std::shared_ptr<dsr_msgs2::srv::GetSolutionSpace::Response> res) -> bool       
{    
    res->success = false;
    std::array<float, NUM_TASK> task_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), task_pos.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_solution_space_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
#endif
    res->sol_space = Drfl->get_solution_space(task_pos.data());
    res->success = true;        
    return true;
};
auto get_orientation_error_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetOrientationError::Request> req, std::shared_ptr<dsr_msgs2::srv::GetOrientationError::Response> res) -> bool              
{        
    res->success = false;
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
    return true;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

auto get_workpiece_weight_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetWorkpieceWeight::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetWorkpieceWeight::Response> res) -> bool          
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< get_workpiece_weight_cb >");
#endif
    res->weight = Drfl->get_workpiece_weight();
    res->success = true;
    return true;
};
auto reset_workpiece_weight_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ResetWorkpieceWeight::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::ResetWorkpieceWeight::Response> res) -> bool          
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< reset_workpiece_weight_cb >");
#endif
    res->success = Drfl->reset_workpiece_weight();

    return true;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//----- FORCE/STIFFNESS Service Call-back functions ------------------------------------------------------------
auto parallel_axis1_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ParallelAxis1::Request> req, std::shared_ptr<dsr_msgs2::srv::ParallelAxis1::Response> res) -> bool  
{
    res->success = false;
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
    return true;
};
auto parallel_axis2_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ParallelAxis2::Request> req, std::shared_ptr<dsr_msgs2::srv::ParallelAxis2::Response> res) -> bool  
{
    res->success = false;
    std::array<float, 3> vector;

    std::copy(req->vect.cbegin(), req->vect.cend(), vector.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< parallel_axis2_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    vect = %7.3f,%7.3f,%7.3f",vector[0],vector[1],vector[2]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    axis = %d",req->axis);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref  = %d",req->ref);
#endif
    res->success = Drfl->parallel_axis(vector.data(), (TASK_AXIS)req->axis, (COORDINATE_SYSTEM)req->ref);        
    return true;
};
auto align_axis1_cb = [this](const std::shared_ptr<dsr_msgs2::srv::AlignAxis1::Request> req, std::shared_ptr<dsr_msgs2::srv::AlignAxis1::Response> res) -> bool          
{
    res->success = false;
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
    return true;
};
auto align_axis2_cb = [this](const std::shared_ptr<dsr_msgs2::srv::AlignAxis2::Request> req, std::shared_ptr<dsr_msgs2::srv::AlignAxis2::Response> res) -> bool      
{
    res->success = false;
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
    return true;
};
auto is_done_bolt_tightening_cb = [this](const std::shared_ptr<dsr_msgs2::srv::IsDoneBoltTightening::Request> req, std::shared_ptr<dsr_msgs2::srv::IsDoneBoltTightening::Response> res) -> bool      
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< is_done_bolt_tightening_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    m       = %f",req->m);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    timeout = %f",req->timeout);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    axis    = %d",req->axis);
#endif
    res->success = Drfl->is_done_bolt_tightening((FORCE_AXIS)req->axis, req->m, req->timeout);
    return true;
};
auto release_compliance_ctrl_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ReleaseComplianceCtrl::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::ReleaseComplianceCtrl::Response> res) -> bool         
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< release_compliance_ctrl_cb >");
#endif
    res->success = Drfl->release_compliance_ctrl();
    return true;
};
auto task_compliance_ctrl_cb = [this](const std::shared_ptr<dsr_msgs2::srv::TaskComplianceCtrl::Request> req, std::shared_ptr<dsr_msgs2::srv::TaskComplianceCtrl::Response> res) -> bool          
{
    res->success = false;
    std::array<float, NUM_TASK> stiffnesses;

    std::copy(req->stx.cbegin(), req->stx.cend(), stiffnesses.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< task_compliance_ctrl_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    stx     = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",stiffnesses[0],stiffnesses[1],stiffnesses[2],stiffnesses[3],stiffnesses[4],stiffnesses[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref     = %d",req->ref);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    timeout = %f",req->time);
#endif
    res->success = Drfl->task_compliance_ctrl(stiffnesses.data(), (COORDINATE_SYSTEM)req->ref, req->time);
    return true;
};
auto set_stiffnessx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetStiffnessx::Request> req, std::shared_ptr<dsr_msgs2::srv::SetStiffnessx::Response> res) -> bool          
{
    res->success = false;
    std::array<float, NUM_TASK> stiffnesses;

    std::copy(req->stx.cbegin(), req->stx.cend(), stiffnesses.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< set_stiffnessx_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    stx     = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",stiffnesses[0],stiffnesses[1],stiffnesses[2],stiffnesses[3],stiffnesses[4],stiffnesses[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref     = %d",req->ref);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    timeout = %f",req->time);
#endif
    res->success = Drfl->set_stiffnessx(stiffnesses.data(), (COORDINATE_SYSTEM)req->ref, req->time);
    return true;
};
auto calc_coord_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CalcCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::CalcCoord::Response> res) -> bool      
{
    res->success = false;
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
    return true;
};
auto set_user_cart_coord1_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord1::Request> req, std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord1::Response> res) -> bool          
{
    res->success = false;
    std::array<float, NUM_TASK> task_pos;

    std::copy(req->pos.cbegin(), req->pos.cend(), task_pos.begin());

#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< set_user_cart_coord1_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    pos = %7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",task_pos[0],task_pos[1],task_pos[2],task_pos[3],task_pos[4],task_pos[5]);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref = %d",req->ref);
#endif
    res->id = Drfl->set_user_cart_coord(0, task_pos.data(), (COORDINATE_SYSTEM)req->ref);  
    res->success = true;
    return true;
};
auto set_user_cart_coord2_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord2::Request> req, std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord2::Response> res) -> bool              
{
    res->success = false;
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
    return true;
};
auto set_user_cart_coord3_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord3::Request> req, std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord3::Response> res) -> bool      
{
    res->success = false;
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
    return true;
};
auto overwrite_user_cart_coord_cb = [this](const std::shared_ptr<dsr_msgs2::srv::OverwriteUserCartCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::OverwriteUserCartCoord::Response> res) -> bool      
{
    res->success = false;
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
    return true;
};
auto get_user_cart_coord_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetUserCartCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::GetUserCartCoord::Response> res) -> bool          
{
    res->success = false;
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
    return true;
};
auto set_desired_force_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetDesiredForce::Request> req, std::shared_ptr<dsr_msgs2::srv::SetDesiredForce::Response> res) -> bool          
{
    res->success = false;
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
    return true;
};
auto release_force_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ReleaseForce::Request> req, std::shared_ptr<dsr_msgs2::srv::ReleaseForce::Response> res) -> bool      
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< release_force_cb >");
#endif
    res->success = Drfl->release_force(req->time);
    return true;
};
auto check_position_condition_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CheckPositionCondition::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckPositionCondition::Response> res) -> bool          
{
    res->success = false;
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
    return true;
};
auto check_force_condition_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CheckForceCondition::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckForceCondition::Response> res) -> bool      
{
    res->success = false;
#if (_DEBUG_DSR_CTL)
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"< check_force_condition_cb >");
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    axis = %d", req->axis);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    min  = %f", req->min);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    max  = %f", req->max);
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"    ref  = %d", req->ref);
#endif
    res->success = Drfl->check_force_condition((FORCE_AXIS)req->axis, req->min, req->max, (COORDINATE_SYSTEM)req->ref);
    return true;
};
auto check_orientation_condition1_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition1::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition1::Response> res) -> bool             
{
    res->success = false;
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
    return true;
};
auto check_orientation_condition2_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition2::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition2::Response> res) -> bool            
{
    res->success = false;
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
    return true;
};
auto coord_transform_cb = [this](const std::shared_ptr<dsr_msgs2::srv::CoordTransform::Request> req, std::shared_ptr<dsr_msgs2::srv::CoordTransform::Response> res) -> bool          
{
    res->success = false;
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
    return true;
};

//----- IO Service Call-back functions ------------------------------------------------------------
auto set_digital_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxDigitalOutput::Response> res) -> bool    
{
    //ROS_INFO("set_digital_output_cb() called and calling Drfl->SetCtrlBoxDigitalOutput");
    res->success = false;

    if((req->index < DR_DIO_MIN_INDEX) || (req->index > DR_DIO_MAX_INDEX)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_digital_output(index=%d, value=%d): index(%d) is out of range. (normal range: %d ~ %d)",req->index ,req->value ,req->index, DR_DIO_MIN_INDEX, DR_DIO_MAX_INDEX);
    }       
    else if((req->value < 0) || (req->value > 1)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_digital_output(index=%d, value=%d): value(%d) is out of range. [normal range: 0 or 1]",req->index ,req->value ,req->value);
    }       
    else{
        req->index -=1;
        res->success = Drfl->SetCtrlBoxDigitalOutput((GPIO_CTRLBOX_DIGITAL_INDEX)req->index, req->value);
    }

    return true;
};
auto get_digital_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalOutput::Response> res) -> bool   
{
    //ROS_INFO("get_digital_output_cb() called and calling Drfl->GetCtrlBoxDigitalOutput");
    res->success = false;

    if((req->index < DR_DIO_MIN_INDEX) || (req->index > DR_DIO_MAX_INDEX)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_digital_output(index=%d): index(%d) is out of range. (normal range: %d ~ %d)",req->index ,req->index, DR_DIO_MIN_INDEX, DR_DIO_MAX_INDEX);
    }       
    else{
        req->index -=1;
        res->value = Drfl->get_digital_output((GPIO_CTRLBOX_DIGITAL_INDEX)req->index);
        res->success = true;
    }

    return true;
};
auto get_digital_input_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalInput::Response> res) -> bool   
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

    return true;
};
auto set_tool_digital_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetToolDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetToolDigitalOutput::Response> res) -> bool   
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

    return true;
};
auto get_tool_digital_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetToolDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetToolDigitalOutput::Response> res) -> bool   
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

    return true;
};
auto get_tool_digital_input_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetToolDigitalInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetToolDigitalInput::Response> res) -> bool   
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

    return true;
};
auto set_analog_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutput::Response> res) -> bool   
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

    return true;
};
auto get_analog_input_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxAnalogInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxAnalogInput::Response> res) -> bool   
{
    //ROS_INFO("get_analog_input_cb() called and calling Drfl->get_analog_input");
    res->success = false;
    bool bIsError = 0;   

    if((req->channel < 1) || (req->channel > 2)){
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_analog_input(channel=%d): channel(%d) is out of range. [normal range: 1 or 2]",req->channel ,req->channel);
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

    return true;
};
auto set_analog_output_type_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutputType::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutputType::Response> res) -> bool   
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

    return true;
};
auto set_analog_input_type_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogInputType::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogInputType::Response> res) -> bool       
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
        if(req->channel == 1) g_nAnalogOutputModeCh1 = req->mode;    
        if(req->channel == 2) g_nAnalogOutputModeCh2 = req->mode;    

        req->channel -=1;
        res->success = Drfl->set_mode_analog_input((GPIO_CTRLBOX_ANALOG_INDEX)req->channel, (GPIO_ANALOG_TYPE)req->mode);
    }

    return true;
};

//----- MODBUS Service Call-back functions ------------------------------------------------------------
auto set_modbus_output_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetModbusOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetModbusOutput::Response> res) -> bool    
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"set_modbus_output_cb() called and calling Drfl->set_modbus_output");
    res->success = false;
    res->success = Drfl->set_modbus_output(req->name, (unsigned short)req->value);
    return true;
};
auto get_modbus_input_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetModbusInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetModbusInput::Response> res) -> bool    
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"get_modbus_input_cb() called and calling Drfl->get_modbus_input");
    res->value = Drfl->get_modbus_input(req->name);
    return true;
};
auto config_create_modbus_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConfigCreateModbus::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigCreateModbus::Response> res) -> bool
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"config_create_modbus_cb() called and calling Drfl->add_modbus_signal");
    res->success = false;
    res->success = Drfl->add_modbus_signal(req->name, req->ip, (unsigned short)req->port, (MODBUS_REGISTER_TYPE)req->reg_type, (unsigned short)req->index, (unsigned short)req->value, (int)req->slave_id);
    return true;
};
auto config_delete_modbus_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConfigDeleteModbus::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigDeleteModbus::Response> res) -> bool
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"config_delete_modbus_cb() called and calling Drfl->del_modbus_signal");
    res->success = false;
    res->success = Drfl->del_modbus_signal(req->name);
    return true;
};

//----- DRL Service Call-back functions ------------------------------------------------------------
auto drl_pause_cb = [this](const std::shared_ptr<dsr_msgs2::srv::DrlPause::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::DrlPause::Response> res) -> bool                         
{
    //ROS_INFO("drl_pause_cb() called and calling Drfl->DrlPause");
    res->success = false;

    if(g_bIsEmulatorMode)
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"The drl service cannot be used in emulator mode (available in real mode).");
    else 
        res->success = Drfl->PlayDrlPause();

    return true;
};
auto drl_start_cb = [this](const std::shared_ptr<dsr_msgs2::srv::DrlStart::Request> req, std::shared_ptr<dsr_msgs2::srv::DrlStart::Response> res) -> bool    
{
    //ROS_INFO("drl_start_cb() called and calling Drfl->drl_start");
    res->success = false;

    if(g_bIsEmulatorMode)
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"The drl service cannot be used in emulator mode (available in real mode).");
    else 
        res->success = Drfl->drl_start((ROBOT_SYSTEM)req->robot_system, req->code);

    return true;
};
auto drl_stop_cb = [this](const std::shared_ptr<dsr_msgs2::srv::DrlStop::Request> req, std::shared_ptr<dsr_msgs2::srv::DrlStop::Response> res) -> bool    
{
    //ROS_INFO("drl_stop_cb() called and calling Drfl->drl_stop");
    res->success = false;

    if(g_bIsEmulatorMode)
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"The drl service cannot be used in emulator mode (available in real mode).");
    else 
        res->success = Drfl->drl_stop((STOP_TYPE)req->stop_mode);

    return true;
};
auto drl_resume_cb = [this](const std::shared_ptr<dsr_msgs2::srv::DrlResume::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::DrlResume::Response> res) -> bool        
{
    //ROS_INFO("drl_resume_cb() called and calling Drfl->drl_resume");
    res->success = false;

    if(g_bIsEmulatorMode)
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"The drl service cannot be used in emulator mode (available in real mode).");
    else 
        res->success = Drfl->drl_resume();

    return true;
};
auto get_drl_state_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetDrlState::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetDrlState::Response> res) -> bool       
{
    res->success = false;

    if(g_bIsEmulatorMode)
        RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"),"The drl service cannot be used in emulator mode (available in real mode).");
    else{ 
        res->drl_state = Drfl->get_program_state();
        res->success = true;
    }    

    return true;
};


//----- TCP Service Call-back functions -------------------------------------------------------------
auto set_current_tcp_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetCurrentTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCurrentTcp::Response> res) -> bool       
{
    //ROS_INFO("set_current_tcp_cb() called and calling Drfl->set_tcp");
    res->success = false;
    res->success = Drfl->set_tcp(req->name);
    return true;
};
auto get_current_tcp_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentTcp::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentTcp::Response> res) -> bool       
{
    //ROS_INFO("get_current_tcp_cb() called and calling Drfl->get_tcp");
    res->success = false;
    res->info = Drfl->get_tcp();
    res->success = true;
    return true;
};
auto config_create_tcp_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConfigCreateTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigCreateTcp::Response> res) -> bool    
{
    //ROS_INFO("config_create_tcp_cb() called and calling Drfl->add_tcp");
    res->success = false;
    std::array<float, 6> target_pos;
    std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
    res->success = Drfl->add_tcp(req->name, target_pos.data());
    return true;
};
auto config_delete_tcp_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTcp::Response> res) -> bool  
{
    //ROS_INFO("config_delete_tcp_cb() called and calling Drfl->del_tcp");
    res->success = false;
    res->success = Drfl->del_tcp(req->name);
    return true;
};

//----- TOOL Service Call-back functions ------------------------------------------------------------
auto set_current_tool_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetCurrentTool::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCurrentTool::Response> res) -> bool     
{
    //ROS_INFO("set_current_tool_cb() called and calling Drfl->set_tool");
    res->success = false;
    res->success = Drfl->set_tool(req->name);
    return true;
};
auto get_current_tool_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentTool::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentTool::Response> res) -> bool     
{
    //ROS_INFO("get_current_tool_cb() called and calling Drfl->get_tool %s", Drfl->GetCurrentTool().c_str());
    res->success = false;
    res->info = Drfl->get_tool();
    res->success = true;
    return true;
};
auto config_create_tool_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConfigCreateTool::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigCreateTool::Response> res) -> bool 
{
    //ROS_INFO("config_create_tool_cb() called and calling Drfl->add_tool");
    res->success = false;
    std::array<float, 3> target_cog;
    std::array<float, 6> target_inertia;
    std::copy(req->cog.cbegin(), req->cog.cend(), target_cog.begin());
    std::copy(req->inertia.cbegin(), req->inertia.cend(), target_inertia.begin());
    res->success = Drfl->add_tool(req->name, req->weight, target_cog.data(), target_inertia.data());
    return true;
};
auto config_delete_tool_cb = [this](const std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTool::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTool::Response> res) -> bool    
{
    //ROS_INFO("config_delete_tool_cb() called and calling Drfl->del_tool");
    res->success = false;
    res->success = Drfl->del_tool(req->name);
    return true;
};
auto set_tool_shape_cb = [this](const std::shared_ptr<dsr_msgs2::srv::SetToolShape::Request> req, std::shared_ptr<dsr_msgs2::srv::SetToolShape::Response> res) -> bool 
{
    res->success = false;
    res->success = Drfl->set_tool(req->name);
    return true;
};
  
  
  m_nh_srv_set_robot_mode             = get_node()->create_service<dsr_msgs2::srv::SetRobotMode>("dsr_test/system/set_robot_mode", set_robot_mode_cb);
  m_nh_srv_get_robot_mode             = get_node()->create_service<dsr_msgs2::srv::GetRobotMode>("system/get_robot_mode", get_robot_mode_cb);     
  m_nh_srv_set_robot_system           = get_node()->create_service<dsr_msgs2::srv::SetRobotSystem>("system/set_robot_system", set_robot_system_cb);         
  m_nh_srv_get_robot_system           = get_node()->create_service<dsr_msgs2::srv::GetRobotSystem>("system/get_robot_system", get_robot_system_cb);         
  m_nh_srv_get_robot_state            = get_node()->create_service<dsr_msgs2::srv::GetRobotState>("system/get_robot_state", get_robot_state_cb);        
  m_nh_srv_set_robot_speed_mode       = get_node()->create_service<dsr_msgs2::srv::SetRobotSpeedMode>("system/set_robot_speed_mode", set_robot_speed_mode_cb);    
  m_nh_srv_get_robot_speed_mode       = get_node()->create_service<dsr_msgs2::srv::GetRobotSpeedMode>("system/get_robot_speed_mode", get_robot_speed_mode_cb);       
  m_nh_srv_get_current_pose           = get_node()->create_service<dsr_msgs2::srv::GetCurrentPose>("system/get_current_pose", get_current_pose_cb);   
  m_nh_srv_set_safe_stop_reset_type   = get_node()->create_service<dsr_msgs2::srv::SetSafeStopResetType>("system/set_safe_stop_reset_type", set_safe_stop_reset_type_cb);           
  m_nh_srv_get_last_alarm             = get_node()->create_service<dsr_msgs2::srv::GetLastAlarm>("system/get_last_alarm", get_last_alarm_cb);   

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
  m_nh_srv_move_stop                  = get_node()->create_service<dsr_msgs2::srv::MoveStop>("motion/move_stop", move_stop_cb);                  
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

  memset(&g_joints,    0x00, sizeof(ROBOT_JOINT_DATA)*NUM_JOINT);
  memset(&g_stDrState, 0x00, sizeof(DR_STATE)); 
  memset(&g_stDrError, 0x00, sizeof(DR_ERROR)); 

  // // create threads     

  g_nAnalogOutputModeCh1 = -1;
  g_nAnalogOutputModeCh2 = -1;
  

  return CallbackReturn::SUCCESS;
}
void interpolate_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & point_1,
  const trajectory_msgs::msg::JointTrajectoryPoint & point_2,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp, double delta)
{
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_2.positions[i];
  }
  for (size_t i = 0; i < point_1.positions.size(); i++)
  {
    point_interp.velocities[i] =
      delta * point_2.velocities[i] + (1.0 - delta) * point_2.velocities[i];
  }
}

void interpolate_trajectory_point(
  const trajectory_msgs::msg::JointTrajectory & traj_msg, const rclcpp::Duration & cur_time,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp)
{
  double traj_len = traj_msg.points.size();
  auto last_time = traj_msg.points[traj_len - 1].time_from_start;
  double total_time = last_time.sec + last_time.nanosec * 1E-9;

  size_t ind = cur_time.seconds() * (traj_len / total_time);
  ind = std::min(static_cast<double>(ind), traj_len - 2);
  double delta = cur_time.seconds() - ind * (total_time / traj_len);
  interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
}

controller_interface::return_type RobotController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
    
    // auto now = this->now();
    //printf("Current time: %lf seconds since the Epoch\n", now_sec);

    
    

    // Publish regularly for smooth looking frames in rviz
    // if (new_msg_)
    // {
    //     trajectory_msg_ = *traj_msg_external_point_ptr_.readFromRT();
    //     start_time_ = time;
    //     new_msg_ = false;
    // }

    // if (trajectory_msg_ != nullptr)
    // {
    //     interpolate_trajectory_point(*trajectory_msg_, time - start_time_, point_interp_);
    //     for (size_t i = 0; i < joint_position_command_interface_.size(); i++)
    //     {
    //     joint_position_command_interface_[i].get().set_value(point_interp_.positions[i]);
    //     }
    //     for (size_t i = 0; i < joint_velocity_command_interface_.size(); i++)
    //     {
    //     joint_velocity_command_interface_[i].get().set_value(point_interp_.velocities[i]);
    //     }
    // }
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (m_node_) {
    RCLCPP_INFO(rclcpp::get_logger("dsr_test"),"on deactivate");
    m_node_.reset();
    m_node_ = nullptr;
    if (m_node_) {
        m_node_.reset();
        m_node_ = nullptr;
        RCLCPP_INFO(rclcpp::get_logger("dsr_test"),"on deactivate");
        // m_node_가 유효한 경우에 실행할 코드
    } else {
        // m_node_가 유효하지 않은 경우에 실행할 코드
    }
  }
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_test"),"on deactivate");
    m_node_.reset();
    m_node_ = nullptr;
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_error(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("dsr_test"),"on deactivate");
    m_node_.reset();
    m_node_ = nullptr;
  return CallbackReturn::SUCCESS;
}

}  // namespace dsr_controller2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dsr_controller2::RobotController, controller_interface::ControllerInterface)
