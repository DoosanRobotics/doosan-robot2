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

#include <robotiq_gazebo/disable_link_plugin.h>

namespace gazebo
{

DisableLinkPlugin::DisableLinkPlugin()
{
  kill_sim = false;
  link_.reset();
}

DisableLinkPlugin::~DisableLinkPlugin()
{
  kill_sim = true;
}

void DisableLinkPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  model_ = _parent;
  world_ = model_->GetWorld();

  // Check for link element
  if (!_sdf->HasElement("link"))
  {
    ROS_ERROR("No link element present. DisableLinkPlugin could not be loaded.");
    return;
  }
  
  link_name_ = _sdf->GetElement("link")->Get<std::string>();

  // Get pointers to joints
  link_ = model_->GetLink(link_name_);
  if(link_)
    link_->SetEnabled(false);
  else
    ROS_WARN("Link %s not found!", link_name_.c_str());
}

GZ_REGISTER_MODEL_PLUGIN(DisableLinkPlugin);

}
