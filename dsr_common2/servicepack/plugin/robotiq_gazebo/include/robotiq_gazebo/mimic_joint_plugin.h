/*********************************************************************
 * 
 *  Copyright (c) 2014, Konstantinos Chatzilygeroudis
 *  Copyright (c) 2016, CRI Lab at Nanyang Technological University
 *  All rights reserved.
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

#ifndef GAZEBO_PLUGINS_MIMIC_JOINT_PLUGIN
#define GAZEBO_PLUGINS_MIMIC_JOINT_PLUGIN

// ROS includes
#include <ros/ros.h>

// ros_control
#include <control_toolbox/pid.h>

// Boost includes
#include <boost/bind.hpp>

// Gazebo includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class MimicJointPlugin : public ModelPlugin
  {
    public:
      MimicJointPlugin();
      ~MimicJointPlugin();

      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void UpdateChild();

    private:
      // Parameters
      std::string joint_name_, mimic_joint_name_, robot_namespace_;
      double multiplier_, offset_, sensitiveness_, max_effort_;
      bool has_pid_;

      bool kill_sim;

      // PID controller if needed
      control_toolbox::Pid pid_;

      // Pointers to the joints
      physics::JointPtr joint_, mimic_joint_;

      // Pointer to the model
      physics::ModelPtr model_;

      // Pointer to the world
      physics::WorldPtr world_;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;

  };
}

#endif
