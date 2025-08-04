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

#ifndef GAZEBO_PLUGINS_DISABLE_LINK_PLUGIN
#define GAZEBO_PLUGINS_DISABLE_LINK_PLUGIN

// ROS includes
#include <ros/ros.h>

// Boost includes
#include <boost/bind.hpp>

// Gazebo includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class DisableLinkPlugin : public ModelPlugin
  {
    public:
      DisableLinkPlugin();
      ~DisableLinkPlugin();

      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void UpdateChild();

    private:
      // Parameters
      std::string link_name_;

      bool kill_sim;

      // Pointers to the joints
      physics::LinkPtr link_;

      // Pointer to the model
      physics::ModelPtr model_;

      // Pointer to the world
      physics::WorldPtr world_;

  };
}

#endif
