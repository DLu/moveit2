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

#pragma once

#include <moveit_setup_controllers/modified_urdf_config.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>

namespace moveit_setup
{
namespace controllers
{
struct ControlInterfaces
{
  std::vector<std::string> command_interfaces;
  std::vector<std::string> state_interfaces;
};

class ROS2ControlConfig : public IncludedHeaderConfig
{
public:
  void onInit() override;

  std::vector<std::pair<std::string, std::string>> getArguments() const override
  {
    std::vector<std::pair<std::string, std::string>> arguments;
    arguments.push_back(std::make_pair("initial_positions_file", "initial_positions.yaml"));
    return arguments;
  }

  std::string getFilename() const override
  {
    return urdf_config_->getRobotName() + ".ros2_control.xacro";
  }

  std::string getCommands() const override
  {
    std::string commands = "    <xacro:";
    commands += urdf_config_->getRobotName();
    commands += "_ros2_control name=\"FakeSystem\" initial_positions_file=\"$(arg initial_positions_file)\"/>";
    return commands;
  }

  void loadFromOriginalURDF();

  std::vector<std::string> getAllJoints() const;
  const std::vector<std::string>& getUnconfiguredJoints() const
  {
    return missing_interfaces_;
  }

  void addControlInterfaces(const std::string& joint_name, const ControlInterfaces& ci)
  {
    new_joint_interfaces_[joint_name] = ci;
    changed_ = true;
  }

  std::vector<std::string> getCommandInterfaces(const std::string& joint_name) const;
  std::vector<std::string> getStateInterfaces(const std::string& joint_name) const;

  bool isConfigured() const override
  {
    return !new_joint_interfaces_.empty();
  }

  bool hasChanges() const override
  {
    return changed_;
  }

  class GeneratedControlHeader : public TemplatedGeneratedFile
  {
  public:
    GeneratedControlHeader(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                           ROS2ControlConfig& parent)
      : TemplatedGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    std::filesystem::path getRelativePath() const override
    {
      return std::filesystem::path("config") / (parent_.urdf_config_->getRobotName() + ".ros2_control.xacro");
    }

    std::filesystem::path getTemplatePath() const override
    {
      return getSharePath("moveit_setup_controllers") / "templates" / "config" / "ros2_control.xacro";
    }

    std::string getDescription() const override
    {
      return "Macro definition for required ros2_control xacro additions.";
    }

    bool hasChanges() const override
    {
      return parent_.hasChanges();
    }

  protected:
    ROS2ControlConfig& parent_;
  };

  void collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                    std::vector<GeneratedFilePtr>& files) override
  {
    files.push_back(std::make_shared<GeneratedControlHeader>(package_path, last_gen_time, *this));
  }

  void collectVariables(std::vector<TemplateVariable>& variables) override;

protected:
  using ControlInterfaceIterator = std::unordered_map<std::string, ControlInterfaces>::const_iterator;
  bool findJointInterface(const std::string& joint_name, ControlInterfaceIterator& it) const;

  std::shared_ptr<SRDFConfig> srdf_config_;

  std::unordered_map<std::string, ControlInterfaces> original_joint_interfaces_, new_joint_interfaces_;
  std::vector<std::string> missing_interfaces_;
  bool changed_;
};
}  // namespace controllers
}  // namespace moveit_setup
