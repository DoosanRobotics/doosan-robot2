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

#include <robotiq_gazebo/mimic_joint_plugin.h>

namespace gazebo
{

MimicJointPlugin::MimicJointPlugin()
{
  kill_sim = false;

  joint_.reset();
  mimic_joint_.reset();
}

MimicJointPlugin::~MimicJointPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);

  kill_sim = true;
}

void MimicJointPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  ros::NodeHandle model_nh;
  model_ = _parent;
  world_ = model_->GetWorld();

  // Error message if the model couldn't be found
  if (!model_)
  {
    ROS_ERROR("Parent model is NULL! MimicJointPlugin could not be loaded.");
    return;
  }

  // Check that ROS has been initialized
  if(!ros::isInitialized())
  {
    ROS_ERROR("A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return;
  }

  // Check for robot namespace
  robot_namespace_ = "/";
  if(_sdf->HasElement("robotNamespace"))
  {
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }

  // Check for joint element
  if(!_sdf->HasElement("joint"))
  {
    ROS_ERROR("No joint element present. MimicJointPlugin could not be loaded.");
    return;
  }
  
  joint_name_ = _sdf->GetElement("joint")->Get<std::string>();

  // Check for mimicJoint element
  if(!_sdf->HasElement("mimicJoint"))
  {
    ROS_ERROR("No mimicJoint element present. MimicJointPlugin could not be loaded.");
    return;
  }
  
  mimic_joint_name_ = _sdf->GetElement("mimicJoint")->Get<std::string>();

  has_pid_ = false;
  // Check if PID controller wanted
  if(_sdf->HasElement("hasPID"))
  {
    has_pid_ = true;

    const ros::NodeHandle nh(model_nh, std::string(robot_namespace_+"/gazebo_ros_control/pid_gains/")+mimic_joint_name_);
    double p, i,d ;
    // TODO: include i_clamp e.t.c.
    nh.param("p", p, 0.0);
    nh.param("i", i, 0.0);
    nh.param("d", d, 0.0);

    pid_ = control_toolbox::Pid(p,i,d);
  }

  // Check for multiplier element
  multiplier_ = 1.0;
  if(_sdf->HasElement("multiplier"))
    multiplier_ = _sdf->GetElement("multiplier")->Get<double>();

  // Check for offset element
  offset_ = 0.0;
  if (_sdf->HasElement("offset"))
    offset_ = _sdf->GetElement("offset")->Get<double>();

  // Check for sensitiveness element
  sensitiveness_ = 0.0;
  if (_sdf->HasElement("sensitiveness"))
    sensitiveness_ = _sdf->GetElement("sensitiveness")->Get<double>();

  // Check for max effort
  max_effort_ = 1.0;
  if (_sdf->HasElement("maxEffort"))
  {
    max_effort_ = _sdf->GetElement("maxEffort")->Get<double>();
  }

  // Get pointers to joints
  joint_ = model_->GetJoint(joint_name_);
  if(!joint_)
  {
    ROS_ERROR("No joint named %s. MimicJointPlugin could not be loaded.", joint_name_.c_str());
    return;
  }
  mimic_joint_ = model_->GetJoint(mimic_joint_name_);
  if(!mimic_joint_)
  {
    ROS_ERROR("No (mimic) joint named %s. MimicJointPlugin could not be loaded.", mimic_joint_name_.c_str());
    return;
  }
  
  // Set max effort
  if(!has_pid_)
  {
    #if GAZEBO_MAJOR_VERSION > 2
      mimic_joint_->SetParam("fmax", 0, max_effort_);
    #else
      mimic_joint_->SetMaxForce(0, max_effort_);
    #endif
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MimicJointPlugin::UpdateChild, this));
}

void MimicJointPlugin::UpdateChild()
{
  static ros::Duration period(world_->GetPhysicsEngine()->GetMaxStepSize());

  // Set mimic joint's angle based on joint's angle
  double angle = joint_->GetAngle(0).Radian()*multiplier_+offset_;
  
  if(abs(angle-mimic_joint_->GetAngle(0).Radian())>=sensitiveness_)
  {
    if(has_pid_)
    {
      double a = mimic_joint_->GetAngle(0).Radian();
      if(a!=a)
        a = angle;
      double error = angle-a;
      double effort = gazebo::math::clamp(pid_.computeCommand(error, period), -max_effort_, max_effort_);
    }
    else
    {
      #if GAZEBO_MAJOR_VERSION >= 4
        mimic_joint_->SetPosition(0, angle);
      #else
        mimic_joint_->SetAngle(0, angle);
      #endif
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin);

}
