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

#include <moveit_setup_controllers/ros2_control_config.hpp>

namespace moveit_setup
{
namespace controllers
{
void ROS2ControlConfig::onInit()
{
  urdf_config_ = config_data_->get<URDFConfig>("urdf");
  srdf_config_ = config_data_->get<SRDFConfig>("srdf");
}

std::vector<std::string> ROS2ControlConfig::getCommandInterfaces(const std::string& /*joint_name*/) const
{
  return { "position" };
}

std::vector<std::string> ROS2ControlConfig::getStateInterfaces(const std::string& /*joint_name*/) const
{
  return { "position", "velocity" };
}

void ROS2ControlConfig::collectVariables(std::vector<TemplateVariable>& variables)
{
  std::string joints = "";
  const std::string TAB = "            ";
  for (const std::string& joint_name : {})
  {
    joints += TAB;
    joints += "<joint name=\"" + joint_name + "\">\n";
    for (const std::string& command_interface : getCommandInterfaces(joint_name))
    {
      joints += TAB;
      joints += "    <command_interface name=\"";
      joints += command_interface;
      joints += "\"/>\n";
    }
    for (const std::string& state_interface : getStateInterfaces(joint_name))
    {
      joints += TAB;
      joints += "    <state_interface name=\"";
      joints += state_interface;
      if (state_interface == "position")
      {
        joints += "\">\n";
        joints += TAB;
        joints += "      <param name=\"initial_value\">${initial_positions['";
        joints += joint_name;
        joints += "']}</param>\n";
        joints += TAB;
        joints += "    </state_interface>\n";
      }
      else
      {
        joints += "\"/>\n";
      }
      joints += TAB;
      joints += "</joint>";
    }
  }

  variables.push_back(TemplateVariable("ROS2_CONTROL_JOINTS", joints));
}

}  // namespace controllers
}  // namespace moveit_setup
