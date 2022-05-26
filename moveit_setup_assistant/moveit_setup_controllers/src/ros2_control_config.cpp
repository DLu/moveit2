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
  available_ci_.command_interfaces = getAvailableInterfaceNames();
  available_ci_.state_interfaces = getAvailableInterfaceNames();
  default_ci_.command_interfaces = { "position" };
  default_ci_.state_interfaces = { "position", "velocity" };
}

void ROS2ControlConfig::loadPrevious(const std::filesystem::path& /*config_package_path*/, const YAML::Node& node)
{
  if (!node.IsDefined())
  {
    return;
  }

  getYamlProperty(node, "command", default_ci_.command_interfaces);
  getYamlProperty(node, "state", default_ci_.state_interfaces);
  setControlInterfaces(default_ci_);
  changed_ = false;
}

YAML::Node ROS2ControlConfig::saveToYaml() const
{
  YAML::Node node;
  node["command"] = default_ci_.command_interfaces;
  node["state"] = default_ci_.state_interfaces;
  return node;
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

void ROS2ControlConfig::setControlInterfaces(const ControlInterfaces& ci)
{
  default_ci_ = ci;
  for (const std::string& joint_name : missing_interfaces_)
  {
    new_joint_interfaces_[joint_name] = default_ci_;
  }
  changed_ = true;
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

  // Set up initial group state
  if (!initial_group_state_.joint_values_.empty())
  {
    return;
  }
  for (const std::string& joint_name : missing_interfaces_)
  {
    std::vector<double> joint_value;
    // TODO: For now, just assume one DOF, and it is zero
    joint_value.push_back(0.0);
    initial_group_state_.joint_values_[joint_name] = joint_value;
  }
}

std::string ROS2ControlConfig::getJointsXML() const
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
  return joints;
}

bool ROS2ControlConfig::GeneratedInitialPositions::writeYaml(YAML::Emitter& emitter)
{
  emitter << YAML::Comment("Default initial positions for " + parent_.urdf_config_->getRobotName() +
                           "'s ros2_control fake system");
  emitter << YAML::Newline;
  emitter << YAML::BeginMap;
  {
    emitter << YAML::Key << "initial_positions";
    emitter << YAML::Value;
    emitter << YAML::BeginMap;
    for (const std::string& joint : parent_.missing_interfaces_)
    {
      const auto it = parent_.initial_group_state_.joint_values_.find(joint);
      if (it == parent_.initial_group_state_.joint_values_.end())
        continue;

      emitter << YAML::Key << joint;

      const std::vector<double>& jv = it->second;

      emitter << YAML::Value;
      if (jv.size() == 1)
      {
        emitter << jv[0];
      }
      else
      {
        emitter << jv;
      }
    }
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndMap;

  return true;
}

bool ROS2ControlConfig::GeneratedControllerConfig::writeYaml(YAML::Emitter& emitter)
{
  std::string controller_name = parent_.urdf_config_->getRobotName() + "_controller";

  emitter << YAML::Comment("This config file is used by ros2_control");
  emitter << YAML::BeginMap;
  {
    // Output controller Manager Config
    emitter << YAML::Key << "controller_manager";
    emitter << YAML::Value;
    emitter << YAML::BeginMap;
    {
      emitter << YAML::Key << "ros__parameters";
      emitter << YAML::Value;
      emitter << YAML::BeginMap;
      {
        emitter << YAML::Key << "update_rate" << YAML::Value << 100;  // Hz
        emitter << YAML::Comment("Hz");
        emitter << YAML::Newline;
        emitter << YAML::Newline;
        emitter << YAML::Key << controller_name;
        emitter << YAML::Value;
        emitter << YAML::BeginMap;
        {
          emitter << YAML::Key << "type" << YAML::Value << "joint_trajectory_controller/JointTrajectoryController";
        }
        emitter << YAML::Newline << YAML::Newline;
        emitter << YAML::EndMap;
        emitter << YAML::Key << "joint_state_broadcaster";
        emitter << YAML::Value;
        emitter << YAML::BeginMap;
        {
          emitter << YAML::Key << "type" << YAML::Value << "joint_state_broadcaster/JointStateBroadcaster";
        }
        emitter << YAML::EndMap;
      }
      emitter << YAML::EndMap;
    }
    emitter << YAML::EndMap;
    emitter << YAML::Newline;
  }
  // Output Controller Configs
  {
    emitter << YAML::Key << controller_name;
    emitter << YAML::Value;
    emitter << YAML::BeginMap;
    {
      emitter << YAML::Key << "ros__parameters";
      emitter << YAML::Value;
      emitter << YAML::BeginMap;
      {
        emitter << YAML::Key << "command_interfaces" << YAML::Value << parent_.default_ci_.command_interfaces;
        emitter << YAML::Key << "state_interfaces" << YAML::Value << parent_.default_ci_.state_interfaces;
        emitter << YAML::Key << "joints" << YAML::Value << parent_.missing_interfaces_;
      }
      emitter << YAML::EndMap;
    }
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndMap;

  return true;
}

void ROS2ControlConfig::collectVariables(std::vector<TemplateVariable>& variables)
{
  variables.push_back(TemplateVariable("ROS2_CONTROL_JOINTS", getJointsXML()));
}

}  // namespace controllers
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::IncludedHeaderConfig, moveit_setup::SetupConfig)
PLUGINLIB_EXPORT_CLASS(moveit_setup::controllers::ROS2ControlConfig, moveit_setup::SetupConfig)
