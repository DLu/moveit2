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
#pragma once

#include <moveit_setup_framework/setup_step.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>

namespace moveit_setup_srdf_plugins
{
class EndEffectors : public moveit_setup_framework::SetupStep
{
public:
  std::string getName() const override
  {
    return "End Effectors";
  }

  void onInit() override;

  bool isReady() const override
  {
    return srdf_config_->getGroups().size() > 0;
  }

  std::vector<std::string> getGroupNames() const
  {
    return srdf_config_->getGroupNames();
  }

  std::vector<srdf::Model::EndEffector>& getEndEffectors()
  {
    return srdf_config_->getEndEffectors();
  }

  /**
   * Find the associated data by name
   *
   * @param name - name of data to find in datastructure
   * @return pointer to data in datastructure
   */
  srdf::Model::EndEffector* findEffectorByName(const std::string& name);

  std::vector<std::string> getLinkNames() const
  {
    return srdf_config_->getLinkNames();
  }

  void removeEefByName(const std::string& eef_name)
  {
    srdf_config_->removeEefByName(eef_name);
    srdf_config_->updateRobotModel(true);
  }

  bool isLinkInGroup(const std::string& link, const std::string& group) const
  {
    const auto rm = srdf_config_->getRobotModel();
    const moveit::core::JointModelGroup* jmg = rm->getJointModelGroup(group);
    return jmg->hasLinkModel(link);
  }

  void setProperties(srdf::Model::EndEffector* eef, const std::string& eef_name, const std::string& parent_link,
                     const std::string& component_group, const std::string& parent_group)
  {
    eef->name_ = eef_name;
    eef->parent_link_ = parent_link;
    eef->component_group_ = component_group;
    eef->parent_group_ = parent_group;
    srdf_config_->updateRobotModel(true);
  }

protected:
  std::shared_ptr<moveit_setup_framework::SRDFConfig> srdf_config_;
  std::shared_ptr<moveit_setup_framework::URDFConfig> urdf_config_;
};
}  // namespace moveit_setup_srdf_plugins
