/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Robotics
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
 *   * Neither the name of PickNik Robotics nor the names of its
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

#include <moveit_setup_srdf_plugins/planning_groups.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.hpp>  // for loading all avail kinematic planners

namespace moveit_setup_srdf_plugins
{
void PlanningGroups::onInit()
{
  config_data_->registerType("group_meta", "moveit_setup_srdf_plugins::GroupMetaConfig");
  group_meta_config_ = config_data_->get<GroupMetaConfig>("group_meta");
}

std::vector<std::string> PlanningGroups::getKinematicPlanners() const
{
  // load all avail kin planners
  std::unique_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> loader;
  try
  {
    loader = std::make_unique<pluginlib::ClassLoader<kinematics::KinematicsBase>>("moveit_core",
                                                                                  "kinematics::KinematicsBase");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    throw std::runtime_error(std::string("Exception while creating class loader for kinematic "
                                         "solver plugins: ") +
                             ex.what());
  }

  std::vector<std::string> planners(loader->getDeclaredClasses());

  // Warn if no plugins are found
  if (planners.empty())
  {
    throw std::runtime_error("No MoveIt-compatible kinematics solvers found. Try "
                             "installing moveit_kinematics (sudo apt-get install "
                             "ros-${ROS_DISTRO}-moveit-kinematics)");
  }
  return planners;
}

}  // namespace moveit_setup_srdf_plugins
