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
#pragma once

#include "rclcpp/rclcpp.hpp"
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/thread/thread.hpp>
#include <array>
#include <algorithm>  // std::copy

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

class HARDWARE_INTERFACE_PUBLIC DRHWInterface : public hardware_interface::SystemInterface
{
public:
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

private:
		std::string drcf_ip;	
		std::string drcf_rt_ip;
		std::string mode;
		std::string model;
		int drcf_port;
};

}
