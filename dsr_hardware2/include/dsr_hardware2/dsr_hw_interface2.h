/*********************************************************************
 *
 * Inferfaces for doosan robot controllor 
 * Author: Minsoo Song (minsoo.song@doosan.com)
 *
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
#pragma once

#include "rclcpp/rclcpp.hpp"
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/thread/thread.hpp>
#include <array>
#include <algorithm>  // std::copy
#include <unordered_map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "../../../dsr_common2/include/DRFLEx.h"


using namespace DRAFramework;
using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace dsr_hardware2{

class DRHWInterface : public hardware_interface::SystemInterface
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

    std::vector<int> hw_mapping_;   // URDF joints -> hardware joints mapping
private:
    std::string drcf_ip_;
    std::string drcf_rt_ip_;
    std::string mode_;
    std::string model_;
    int drcf_port_;
    int update_rate_;
    std::unordered_set<size_t> ignored_joints_;
};
}
