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
#include <tinyxml2.h>
#include <algorithm>

namespace moveit_setup
{
namespace controllers
{
void ROS2ControlConfig::onInit()
{
  urdf_config_ = config_data_->get<URDFConfig>("urdf");
  srdf_config_ = config_data_->get<SRDFConfig>("srdf");
}

bool ROS2ControlConfig::findJointInterface(const std::string& joint_name, ControlInterfaceIterator& it) const
{
  it = original_joint_interfaces_.find(joint_name);
  if (it != original_joint_interfaces_.end())
  {
    return true;
  }
  it = new_joint_interfaces_.find(joint_name);
  return it != new_joint_interfaces_.end();
}

std::vector<std::string> ROS2ControlConfig::getCommandInterfaces(const std::string& joint_name) const
{
  ControlInterfaceIterator it;
  if (findJointInterface(joint_name, it))
  {
    return it->second.command_interfaces;
  }
  else
  {
    return std::vector<std::string>();
  }
}

std::vector<std::string> ROS2ControlConfig::getStateInterfaces(const std::string& joint_name) const
{
  ControlInterfaceIterator it;
  if (findJointInterface(joint_name, it))
  {
    return it->second.state_interfaces;
  }
  else
  {
    return std::vector<std::string>();
  }
}

std::vector<std::string> ROS2ControlConfig::getAllJoints() const
{
  std::vector<std::string> joint_names;
  for (const std::string& group_name : srdf_config_->getGroupNames())
  {
    for (const std::string& joint_name : srdf_config_->getJointNames(group_name, true, false))  // exclude passive
    {
      joint_names.push_back(joint_name);
    }
  }
  return joint_names;
}

void getInterfaceNames(const tinyxml2::XMLElement* joint_el, const std::string& element_name,
                       std::vector<std::string>& interface_names)
{
  for (const tinyxml2::XMLElement* el = joint_el->FirstChildElement(element_name.c_str()); el != NULL;
       el = el->NextSiblingElement())
  {
    interface_names.push_back(el->Attribute("name"));
  }
}

void ROS2ControlConfig::loadFromOriginalURDF()
{
  using tinyxml2::XMLElement;
  original_joint_interfaces_.clear();
  missing_interfaces_ = getAllJoints();

  tinyxml2::XMLDocument urdf_xml;
  urdf_xml.Parse(urdf_config_->getURDFContents().c_str());
  for (XMLElement* control_el = urdf_xml.FirstChildElement("ros2_control"); control_el != NULL;
       control_el = control_el->NextSiblingElement())
  {
    for (XMLElement* joint_el = control_el->FirstChildElement("joint"); joint_el != NULL;
         joint_el = joint_el->NextSiblingElement())
    {
      std::string joint_name = joint_el->Attribute("name");

      // remove from missing_interfaces_
      auto it = std::find(missing_interfaces_.begin(), missing_interfaces_.end(), joint_name);
      if (it != missing_interfaces_.end())
      {
        missing_interfaces_.erase(it);
      }

      // Parse the interfaces
      ControlInterfaces ci;
      getInterfaceNames(joint_el, "command_interface", ci.command_interfaces);
      getInterfaceNames(joint_el, "state_interface", ci.state_interfaces);
      original_joint_interfaces_[joint_name] = ci;
    }
  }
}

void ROS2ControlConfig::collectVariables(std::vector<TemplateVariable>& variables)
{
  std::string joints = "";
  const std::string TAB = "            ";
  // Loop through all joints to preserve joint ordering
  for (const std::string& joint_name : getAllJoints())
  {
    auto pair = new_joint_interfaces_.find(joint_name);
    if (pair == new_joint_interfaces_.end())
    {
      continue;
    }

    const ControlInterfaces& ci = pair->second;

    joints += TAB;
    joints += "<joint name=\"" + joint_name + "\">\n";
    for (const std::string& command_interface : ci.command_interfaces)
    {
      joints += TAB;
      joints += "    <command_interface name=\"";
      joints += command_interface;
      joints += "\"/>\n";
    }
    for (const std::string& state_interface : ci.state_interfaces)
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
    }
    joints += TAB;
    joints += "</joint>\n";
  }

  variables.push_back(TemplateVariable("ROS2_CONTROL_JOINTS", joints));
}

}  // namespace controllers
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::IncludedHeaderConfig, moveit_setup::SetupConfig)
PLUGINLIB_EXPORT_CLASS(moveit_setup::controllers::ROS2ControlConfig, moveit_setup::SetupConfig)
