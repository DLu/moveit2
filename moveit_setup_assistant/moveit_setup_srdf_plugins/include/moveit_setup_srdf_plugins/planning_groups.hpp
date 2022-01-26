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
#include <moveit_setup_srdf_plugins/group_meta_config.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_srdf_plugins/tree_structure.hpp>
#include <moveit/robot_model/robot_model.h>

namespace moveit_setup_srdf_plugins
{
using LinkNameTree = GenericTree<std::string>;

inline LinkNameTree buildLinkNameTree(const moveit::core::LinkModel* link)
{
  LinkNameTree node;
  node.data = link->getName();

  for (const auto& child_joint : link->getChildJointModels())
  {
    node.children.push_back(buildLinkNameTree(child_joint->getChildLinkModel()));
  }
  return node;
}

class PlanningGroups : public moveit_setup_framework::SetupStep
{
public:
  std::string getName() const override
  {
    return "Planning Groups";
  }

  void onInit() override;

  bool isReady() const override
  {
    return srdf_config_->isConfigured();
  }

  std::vector<srdf::Model::Group>& getGroups()
  {
    return srdf_config_->getGroups();
  }

  std::vector<std::string> getGroupNames() const
  {
    return srdf_config_->getGroupNames();
  }

  /**
   * Find the associated group by name
   *
   * @param name - name of data to find in datastructure
   * @return pointer to data in datastructure
   */
  srdf::Model::Group* findGroupByName(const std::string& name);

  std::vector<std::string> getKinematicPlanners() const;

  const std::vector<std::string>& getJointNames() const
  {
    return srdf_config_->getRobotModel()->getJointModelNames();
  }

  const std::vector<std::string>& getLinkNames() const
  {
    return srdf_config_->getRobotModel()->getLinkModelNames();
  }

  std::string getChildOfJoint(const std::string& joint_name)
  {
    const moveit::core::JointModel* joint_model = srdf_config_->getRobotModel()->getJointModel(joint_name);
    // Check that a joint model was found
    if (!joint_model)
    {
      return "";
    }
    return joint_model->getChildLinkModel()->getName();
  }

  std::string getJointType(const std::string& joint_name) const
  {
    const moveit::core::JointModel* joint_model = srdf_config_->getRobotModel()->getJointModel(joint_name);
    if (!joint_model)
    {
      return "";
    }
    return joint_model->getTypeName();
  }

  /**
   * @brief Set the specified group's joint names
   */
  void setJoints(const std::string& group_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Set the specified group's link names
   */
  void setLinks(const std::string& group_name, const std::vector<std::string>& link_names);

  /**
   * @brief Set the specified group's kinematic chain
   * @throws runtime_error If base/tip are invalid
   */
  void setChain(const std::string& group_name, const std::string& base, const std::string& tip);

  /**
   * @brief Set the specified group's subgroups
   * @throws runtime_error If subgroup would result in a cycle
   */
  void setSubgroups(const std::string& selected_group_name, const std::vector<std::string>& subgroups);

  void createGroup(const std::string& group_name);

  void deleteGroup(const std::string& group_name);

  void renameGroup(const std::string& old_group_name, const std::string& new_group_name);

  const GroupMetaData& getMetaData(const std::string& group_name) const
  {
    return group_meta_config_->getMetaData(group_name);
  }

  void setMetaData(const std::string& group_name, const GroupMetaData& meta_data)
  {
    group_meta_config_->setMetaData(group_name, meta_data);
  }

  LinkNameTree getLinkNameTree() const
  {
    const moveit::core::JointModel* root_joint = srdf_config_->getRobotModel()->getRootJoint();
    return buildLinkNameTree(root_joint->getChildLinkModel());
  }

  std::vector<std::string> getPosesByGroup(const std::string& group_name) const
  {
    std::vector<std::string> pose_names;
    for (const srdf::Model::GroupState& pose : srdf_config_->getGroupStates())
    {
      if (pose.group_ == group_name)
      {
        pose_names.push_back(pose.name_);
      }
    }
    return pose_names;
  }

  std::vector<std::string> getEndEffectorsByGroup(const std::string& group_name) const
  {
    std::vector<std::string> eef_names;
    for (const srdf::Model::EndEffector& eef : srdf_config_->getEndEffectors())
    {
      if (eef.component_group_ == group_name)
      {
        eef_names.push_back(eef.name_);
      }
    }
    return eef_names;
  }

protected:
  std::shared_ptr<moveit_setup_framework::SRDFConfig> srdf_config_;
  std::shared_ptr<GroupMetaConfig> group_meta_config_;
};
}  // namespace moveit_setup_srdf_plugins
