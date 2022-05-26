/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Metro Robots
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
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
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

/* Author: David V. Lu!! */

#include <moveit_setup_controllers/urdf_modifications.hpp>

namespace moveit_setup
{
namespace controllers
{
void UrdfModifications::onInit()
{
  config_data_->registerType("ros2_control", "moveit_setup::controllers::ROS2ControlConfig");
  ros2_control_config_ = config_data_->get<ROS2ControlConfig>("ros2_control");
  srdf_config_ = config_data_->get<SRDFConfig>("srdf");
}

void UrdfModifications::loadDefault()
{
  for (const std::string& joint_name : ros2_control_config_->getUnconfiguredJoints())
  {
    ControlInterfaces ci;
    ci.command_interfaces.push_back("position");
    ci.state_interfaces.push_back("position");
    ci.state_interfaces.push_back("velocity");
    ros2_control_config_->addControlInterfaces(joint_name, ci);
  }
}

bool UrdfModifications::needsWork() const
{
  return !ros2_control_config_->getUnconfiguredJoints().empty();
}

}  // namespace controllers
}  // namespace moveit_setup
