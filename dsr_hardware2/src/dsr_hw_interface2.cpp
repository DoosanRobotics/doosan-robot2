/*********************************************************************
 * 
 * dsr_hardware2
 * Copyright (c) 2025 Doosan Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *********************************************************************/
#include <boost/thread/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <unistd.h>     
#include <math.h>

#include "dsr_hardware2/dsr_hw_interface2.h"
#include "dsr_hardware2/util.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "../../dsr_common2/include/DRFLEx.h"

using namespace std;
using namespace chrono;
using namespace DRAFramework;

CDRFLEx Drfl;

bool g_bIsEmulatorMode = FALSE;
std::string g_model;
int m_nVersionDRCF;
constexpr size_t g_k_default_num_joint = 6;
constexpr size_t g_k_p3020_num_joint = 5;
constexpr size_t g_k_p3020_fixed_joint_index = 3;

void* get_drfl(){
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[DRFL address] %p", &Drfl);
    return &Drfl;
}

namespace dsr_hardware2{

CallbackReturn DRHWInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    for(auto parameter : info.hardware_parameters) {
        if("host" == parameter.first) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "host : %s", parameter.second.c_str());
            drcf_ip_ = parameter.second;
        } else if("rt_host" == parameter.first) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "rt_host : %s", parameter.second.c_str());
            drcf_rt_ip_ = parameter.second;
        } else if("port" == parameter.first) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "port : %s", parameter.second.c_str());
            drcf_port_ = std::stoi(parameter.second);
        } else if("mode" == parameter.first) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "mode : %s", parameter.second.c_str());
            mode_ = parameter.second;
        } else if("model" == parameter.first) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "model : %s", parameter.second.c_str());
            model_ = parameter.second;
            std::transform(model_.begin(), model_.end(), model_.begin(), ::tolower);
            g_model = model_;
        } else if ("update_rate" == parameter.first) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"update_rate : %s", parameter.second.c_str());
            update_rate_ = std::stoi(parameter.second);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"), "Unexpected Parameter....\
                 key : %s, value : %s",parameter.first.c_str(), parameter.second.c_str());
        }
    }
    const size_t expected_num_joints = (model_ == "p3020") ? g_k_p3020_num_joint : g_k_default_num_joint;
    if (model_ == "p3020") {
        ignored_joints_.insert(g_k_p3020_fixed_joint_index); // ignore joint 4 for p3020
    }

    // host, rt_host, port, mode, model, update_rate
    if(info.hardware_parameters.size() != 6) {
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"), "Unexpected Parameter Size ...");
        return CallbackReturn::ERROR;
    }

    // robot has 6 (or 5) joints and 2 interfaces
    joint_position_.assign(expected_num_joints, 0);
    joint_velocities_.assign(expected_num_joints, 0);
    joint_position_command_.assign(expected_num_joints, 0);
    joint_velocities_command_.assign(expected_num_joints, 0);

    if(expected_num_joints != info_.joints.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), 
                "[on_init] Hardware joint size : %zu, expected : %zu", info.joints.size(), expected_num_joints);
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), 
                    "[on_init] Hardware name : %s, type : %s, plugin name : %s",
                    info_.name.c_str(), info_.type.c_str(), info_.hardware_plugin_name.c_str());

    for (const auto & joint : info_.joints)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("dsr_hw_interface2"), 
            "[on_init] joint name : %s, type : %s,",
            joint.name.c_str(), joint.type.c_str());
        for (const auto & interface : joint.state_interfaces)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("dsr_hw_interface2"), 
                    "[on_init] joint state interface name : %s ", 
                    interface.name.c_str());
            if(interface.name == "effort") {
                RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), 
                    "[on_init] Not Implemented effort interface.. ignored");
                continue;
            }
            joint_interfaces[interface.name].push_back(joint.name);
        }
        for (const auto & interface : joint.command_interfaces)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("dsr_hw_interface2"),
                "[on_init] joint command_interfaces name : %s ",
                interface.name.c_str());
            if(interface.name == "effort") {
                RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),
                            "[on_init] Not Implemented effort interface.. ignored");
                continue;
            }
            joint_comm_interfaces[interface.name].push_back(joint.name);
        }
    }


//-----------------------------------------------------------------------------------------------------
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    INITAILIZE");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");

    // Try to connect to DRCF for 10 (20 * 0.5) sec. 
    bool is_connected = false;
    for (size_t retry = 0; retry < 20; ++retry) {
        is_connected = Drfl.open_connection(drcf_ip_, drcf_port_);
        if(!is_connected) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"Connecting failure.. retry...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"Connected to DRCF");
        break;
    }
    if(!is_connected)
    {
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"),"    DSRInterface::init() DRCF connecting ERROR!!!");
            return CallbackReturn::ERROR;
    }
    // Check whether DRCF loaded successfully for 10 sec..
    // Even thought, the server connected,
    // The drcf could still be in the booting process. 
    // Need to make sure it loaded successfully.
    // By making sure AUTHORITY and STANDBY_STATE.
    static bool get_control_access = false;
    static bool is_standby = false;
    Drfl.set_on_monitoring_access_control([](const MONITORING_ACCESS_CONTROL access) {
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"AUTHORITY : %s", to_str(access).c_str());
        if(MONITORING_ACCESS_CONTROL_GRANT == access) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"INITIAL AUTHORITY GRANTED !!!");
            get_control_access = true;
            is_standby = false; // previous standby state before getting authority is definitely useless.
        }
        if(MONITORING_ACCESS_CONTROL_LOSS == access) {
            get_control_access = false;
            is_standby = false; // previous standby state after losing authority is definitely useless.
        }
    });
    Drfl.set_on_monitoring_state([](const ROBOT_STATE state) {
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"ROBOT_STATE : %s", to_str(state).c_str());
        if(STATE_STANDBY == state) {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"INITIAL STATE_STANDBY !!!");
            is_standby = true;
        }else {
            is_standby = false;
        }
    });
    for (size_t retry = 0; retry < 10; ++retry, std::this_thread::sleep_for(std::chrono::milliseconds(1000))) {
        if(!get_control_access) {
            Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"INITIAL MANAGE_ACCESS_CONTROL_FORCE_REQUEST called");
            continue;
        }
        if(!is_standby) {
            Drfl.set_robot_control(CONTROL_SERVO_ON);
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"INITIAL CONTROL_SERVO_ON called");
            continue;
        }
        if(get_control_access && is_standby)   break;
    }
    if(!(get_control_access && is_standby)) {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"),"INITIAL STATE CALL FAILURE !!");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n"); 
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    OPEN CONNECTION");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n"); 

    //--- connect Emulator ? ------------------------------    
    if(mode_ == "virtual") {
        g_bIsEmulatorMode = true;
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Emulator Mode");
    } else {
        g_bIsEmulatorMode = false;
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Real Robot Mode");
    }

    //--- Get version -------------------------------------            
    SYSTEM_VERSION tSysVerion;
    memset(&tSysVerion, 0, sizeof(tSysVerion));
    assert(Drfl.get_system_version(&tSysVerion));

    //--- Get DRCF version & convert to integer  ----------            
    m_nVersionDRCF = 0; 
    int k=0;
    for(int i=strlen(tSysVerion._szController); i>0; i--)
            if(tSysVerion._szController[i]>='0' && tSysVerion._szController[i]<='9')
                    m_nVersionDRCF += (tSysVerion._szController[i]-'0')*pow(10.0,k++);
    if(m_nVersionDRCF < 100000) m_nVersionDRCF += 100000; 
                 
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    DRCF version = %s",tSysVerion._szController);
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    DRFL version = %s",Drfl.get_library_version());
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    m_nVersionDRCF = %d", m_nVersionDRCF);  //ex> M2.40 = 120400, M2.50 = 120500  
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");

    Drfl.setup_monitoring_version(1); //Enabling extended monitoring functions 

    if(Drfl.GetRobotState() != STATE_STANDBY)    {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "Expected State : Stanby, \
            but Actual State : %s ", to_str(Drfl.GetRobotState()).c_str()); 
        return CallbackReturn::ERROR;
    }

    //--- Set Robot mode : MANUAL or AUTO
    if(!Drfl.SetRobotMode(ROBOT_MODE_AUTONOMOUS)) {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "ROBOT_MODE_AUTONOMOUS Setting Failure !!"); 
        return CallbackReturn::ERROR;
    }

    //--- Set Robot mode : virual or real 
    ROBOT_SYSTEM eTargetSystem = ROBOT_SYSTEM_VIRTUAL;
    if(mode_ == "real") eTargetSystem = ROBOT_SYSTEM_REAL;
    if(!Drfl.SetRobotSystem(eTargetSystem)) {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "SetRobotSystem {%s} Setting Failure !!",
                mode_.c_str()); 
        return CallbackReturn::ERROR;
    }

    // Basically, Controller automatically servo-off after elapse time (5 min)
    // Deactivate it.
    Drfl.set_auto_servo_off(0, 5.0);

    // Virtual controller doesn't support real time connection.
    if(mode_ != "virtual") {
        if(m_nVersionDRCF >= 3000000) {
            drcf_ip_ = drcf_rt_ip_;
        }
        if (!Drfl.connect_rt_control(drcf_ip_)) {
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "Unable to connect RT control stream");
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Connected RT control stream");
        const std::string version   = "v1.0";
        const float       period    = 0.001;
        const int         losscount = 4;
        if (!Drfl.set_rt_control_output(version, period, losscount)) {
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "Unable to connect RT control stream");
            return CallbackReturn::FAILURE;
        }

        if (!Drfl.start_rt_control()) {
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "Unable to start RT control");
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Setting velocity and acceleration limits");
        float limit[6] = {70.0f,70.0f,70.0f,70.0f,70.0f,70.0f};
        if (!Drfl.set_velj_rt(limit)) return CallbackReturn::ERROR;
        if (!Drfl.set_accj_rt(limit)) return CallbackReturn::ERROR;
    }

    Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS,SAFETY_MODE_EVENT_MOVE);
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DRHWInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

    for(size_t i=0; i<joint_interfaces["position"].size(); i++) {
        state_interfaces.emplace_back(joint_interfaces["position"][i], "position", &joint_position_[i]);
    }
    // TODO(songms, yurirocha15) support velocity control.
    for(size_t i=0; i<joint_interfaces["velocity"].size(); i++) {
        state_interfaces.emplace_back(joint_interfaces["velocity"][i], "velocity", &joint_velocities_[i]);
    }
    // TODO(songms, yurirocha15) support effort control.
    for(size_t i=0; i<joint_interfaces["effort"].size(); i++) {
        state_interfaces.emplace_back(joint_interfaces["effort"][i], "effort", &joint_effort_[i]);
    }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DRHWInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
    pre_joint_position_command_ = joint_position_command_;
    for(size_t i=0; i<joint_comm_interfaces["position"].size(); i++) {
        command_interfaces.emplace_back(joint_comm_interfaces["position"][i], "position", &joint_position_command_[i]);
    }
    for(size_t i=0; i<joint_comm_interfaces["velocity"].size(); i++) {
        command_interfaces.emplace_back(joint_comm_interfaces["velocity"][i], "velocity", &joint_velocities_command_[i]);
    }
    // TODO(songms, yurirocha15) support effort control.
    for(size_t i=0; i<joint_comm_interfaces["effort"].size(); i++) {
        command_interfaces.emplace_back(joint_comm_interfaces["effort"][i], "effort", &joint_effort_command_[i]);
    }
  return command_interfaces;
}


return_type DRHWInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    const size_t expected_num_joints = joint_position_.size();
    if(mode_ == "real") {
        const LPRT_OUTPUT_DATA_LIST data = Drfl.read_data_rt();
        if(nullptr == data) {
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"),
                                    "[read] read_data_rt retrieves nullptr");
            return return_type::ERROR;
        }
        for(size_t idx_control=0, idx_ros=0; idx_control < g_k_default_num_joint; idx_control++) {
            if (ignored_joints_.find(idx_control) != ignored_joints_.end()) {
                continue;
            }
            joint_position_[idx_ros] = static_cast<float>(data->actual_joint_position[idx_control] * (M_PI / 180.0f));
            joint_velocities_[idx_ros] = static_cast<float>(data->actual_joint_velocity[idx_control] * (M_PI / 180.0f));
            idx_ros++;
        }
    }else if(mode_ == "virtual") {
        LPROBOT_POSE pose = Drfl.GetCurrentPose();
        if(nullptr == pose) {
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"),
                                    "[read] GetCurrentPose retrieves nullptr");
            return return_type::ERROR; //? what effection of this to control node 
        }
        for(size_t idx_control=0, idx_ros=0; idx_control < g_k_default_num_joint; idx_control++) {
            if (ignored_joints_.find(idx_control) != ignored_joints_.end()) {
                continue;
            }
            joint_position_[idx_ros++] = deg2rad(pose->_fPosition[idx_control]);
        }
    }else {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), 
                "'mode' is neither 'real' nor 'virtual.'" );
    }
    // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "[READ] joint_position_  : {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f}"
    //     ,joint_position_[0]
    //     ,joint_position_[1]
    //     ,joint_position_[2]
    //     ,joint_position_[3]
    //     ,joint_position_[4]
    //     ,joint_position_[5]);
  return return_type::OK;
}

bool positionCommandRunning(const std::vector<double>& lhs, const std::vector<double>& rhs) {
    double var = 0;
    for(size_t i=0; i<lhs.size(); i++) {
        var += abs(lhs[i] - rhs[i]);
    }
    return var >= 0.0001;
}

vector<vector<float>> joint_position_commands;

return_type DRHWInterface::write(const rclcpp::Time &, const rclcpp::Duration &dt)
{
    // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "[WRITE] dt  : %.3f", float(dt.seconds()) );
    // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "[WRITE] joint_position_command_  : {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f}"
    //         ,joint_position_command_[0]
    //         ,joint_position_command_[1]
    //         ,joint_position_command_[2]
    //         ,joint_position_command_[3]
    //         ,joint_position_command_[4]
    //         ,joint_position_command_[5]);
    // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "[WRITE] joint_velocities_command_  : {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f}"
    //         ,joint_velocities_command_[0]
    //         ,joint_velocities_command_[1]
    //         ,joint_velocities_command_[2]
    //         ,joint_velocities_command_[3]
    //         ,joint_velocities_command_[4]
    //         ,joint_velocities_command_[5]);

    // Measure CPU loop duration for REAL servo timing
    static auto last_tick = std::chrono::steady_clock::now();
    auto now_cpu = std::chrono::steady_clock::now();
    double real_loop_dt = std::chrono::duration<double>(now_cpu - last_tick).count();
    last_tick = now_cpu;

    // dt provided by controller_manager
    const double dt_sec = dt.seconds();

    // Expected control period from update_rate_ (Hz → seconds)
    double desired_period = 0.0;
    if (update_rate_ > 0)
        desired_period = 1.0 / static_cast<double>(update_rate_);

    // REAL mode: filter unstable dt cycles
    if (mode_ == "real" && desired_period > 0.0)
    {
        double min_dt = desired_period * 0.3;
        double max_dt = desired_period * 1.5;

        if (dt_sec < min_dt || dt_sec > max_dt)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("dsr_hw_interface2"),
                "[REAL] Skip dt=%.6f (expected=%.6f, allowed=[%.6f, %.6f])",
                dt_sec, desired_period, min_dt, max_dt
            );
            return return_type::OK;
        }
    }

    double effective_dt = dt_sec;
    if (mode_ == "virtual")
    {
        if (update_rate_ > 10)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("dsr_hw_interface2"),"[DEBUG] update_rate_=%d Hz exceeds recommended 10 Hz",update_rate_);
        }

        double desired_period_virtual = (update_rate_ > 0) ? desired_period : 0.1;        // If update_rate_ is invalid, fallback to 10Hz (0.1s)
        double min_dt = desired_period_virtual * 0.3;
        double max_dt = desired_period_virtual * 1.5;

        if (dt_sec < min_dt || dt_sec > max_dt)
        {
            RCLCPP_DEBUG(
                rclcpp::get_logger("dsr_hw_interface2"),
                "[VIRTUAL] Skip dt=%.6f (expected=%.6f, allowed=[%.6f, %.6f])",
                dt_sec, desired_period_virtual, min_dt, max_dt
            );
            return return_type::OK;
        }
        effective_dt = dt_sec;
    }

    // Accumulate simulated time
    static double total_time_sec = 0.0;
    total_time_sec += effective_dt;

    static bool idle = false;

    if (positionCommandRunning(pre_joint_position_command_, joint_position_command_))
    {
        if (idle)
        {
            Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
            idle = false;
        }

        // Convert rad → deg
        float pos[g_k_default_num_joint];
        float vel[g_k_default_num_joint];
        for (size_t idx_control = 0, idx_ros = 0; idx_control < g_k_default_num_joint; idx_control++)
        {
            if (ignored_joints_.find(idx_control) != ignored_joints_.end()) {
                pos[idx_control] = 0.0f;
                vel[idx_control] = 0.0f;
            }
            else {
                pos[idx_control] = static_cast<float>(joint_position_command_[idx_ros] * (180.0 / M_PI));
                vel[idx_control] = static_cast<float>(joint_velocities_command_[idx_ros] * (180.0 / M_PI));
                idx_ros++;
            }
        }

        // Select control API
        std::string cmd_type;
        if (mode_ == "real")
        {
            float acc[g_k_default_num_joint] = {0,0,0,0,0,0};
            const float margin = 20.0f;
            float servo_time = static_cast<float>(real_loop_dt * margin);

            Drfl.servoj_rt(pos, vel, acc, servo_time);
            cmd_type = "servoj_rt";
        }
        else  // virtual
        {
            float target_vel_acc[g_k_default_num_joint] = {70,70,70,70,70,70};
            Drfl.amovej(pos, target_vel_acc, target_vel_acc);
            cmd_type = "amovej";
        }

        // Debug logging
        // RCLCPP_INFO(
        //     rclcpp::get_logger("dsr_hw_interface2"),
        //     "[WRITE] t=%.6f | mode_=%s | dt=%.6f → eff=%.6f\n"
        //     "        update_rate_=%d (period=%.6f)\n"
        //     "        pos={%.3f %.3f %.3f %.3f %.3f %.3f} deg\n"
        //     "        vel={%.3f %.3f %.3f %.3f %.3f %.3f} deg/s\n"
        //     "        cmd=%s",
        //     total_time_sec,
        //     mode_.c_str(),
        //     dt_sec, effective_dt,
        //     update_rate_, desired_period,
        //     pos[0], pos[1], pos[2], pos[3], pos[4], pos[5],
        //     vel[0], vel[1], vel[2], vel[3], vel[4], vel[5],
        //     cmd_type.c_str()
        // );

        pre_joint_position_command_ = joint_position_command_;
        return return_type::OK;
    }
    idle = true;
    pre_joint_position_command_ = joint_position_command_;
    return return_type::OK;
}


DRHWInterface::~DRHWInterface()
{
    Drfl.stop_rt_control();
    // To-do : Update disconnection function in controller version v3.6
    // Drfl.disconnect_rt_control();
    Drfl.close_connection();

    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n"); 
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    CONNECTION IS CLOSED");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n"); 
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dsr_hardware2::DRHWInterface, hardware_interface::SystemInterface)
