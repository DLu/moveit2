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

//// Cycle checking
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>

namespace moveit_setup_srdf_plugins
{
// Used for checking for cycles in a subgroup hierarchy
struct CycleDetector : public boost::dfs_visitor<>
{
  CycleDetector(bool& has_cycle) : m_has_cycle(has_cycle)
  {
  }

  template <class Edge, class Graph>
  void backEdge(Edge /*unused*/, Graph& /*unused*/)
  {
    m_has_cycle = true;
  }

protected:
  bool& m_has_cycle;
};

void PlanningGroups::onInit()
{
  config_data_->registerType("group_meta", "moveit_setup_srdf_plugins::GroupMetaConfig");
  group_meta_config_ = config_data_->get<GroupMetaConfig>("group_meta");
  srdf_config_ = config_data_->get<moveit_setup_framework::SRDFConfig>("srdf");
}

srdf::Model::Group* PlanningGroups::findGroupByName(const std::string& name)
{
  // Find the group we are editing based on the group name string
  srdf::Model::Group* searched_group = nullptr;  // used for holding our search results

  for (srdf::Model::Group& group : srdf_config_->getGroups())
  {
    if (group.name_ == name)  // string match
    {
      searched_group = &group;  // convert to pointer from iterator
      break;                    // we are done searching
    }
  }

  // Check if subgroup was found
  if (searched_group == nullptr)  // not found
  {
    RCLCPP_ERROR_STREAM((*logger_), "An internal error has occurred while searching for groups. Group '"
                                        << name << "' was not found  in the SRDF.");
    throw std::runtime_error(name + " was not found in the SRDF");
  }

  return searched_group;
}

void PlanningGroups::setJoints(const std::string& group_name, const std::vector<std::string>& joint_names)
{
  // Find the group we are editing based on the group name string
  srdf::Model::Group* searched_group = findGroupByName(group_name);

  // save the data
  searched_group->joints_ = joint_names;

  // Update the kinematic model with changes
  srdf_config_->updateRobotModel(true);
}

void PlanningGroups::setLinks(const std::string& group_name, const std::vector<std::string>& link_names)
{
  // Find the group we are editing based on the group name string
  srdf::Model::Group* searched_group = findGroupByName(group_name);

  // save the data
  searched_group->links_ = link_names;

  // Update the kinematic model with changes
  srdf_config_->updateRobotModel(true);
}

void PlanningGroups::setChain(const std::string& group_name, const std::string& base, const std::string& tip)
{
  // Check that box the tip and base, or neither, have text
  if ((!tip.empty() && base.empty()) || (tip.empty() && !base.empty()))
  {
    throw std::runtime_error("You must specify a link for both the base and tip, or leave both "
                             "blank.");
  }

  // Check that both given links are valid links, unless they are both blank
  if (!tip.empty() && !base.empty())
  {
    // Check that they are not the same link
    if (tip.compare(base) == 0)  // they are same
    {
      throw std::runtime_error("Tip and base link cannot be the same link.");
    }

    bool found_tip = false;
    bool found_base = false;
    const std::vector<std::string>& links = getLinkNames();

    for (const std::string& link : links)
    {
      // Check if string matches either of user specified links
      if (link.compare(tip) == 0)  // they are same
        found_tip = true;
      else if (link.compare(base) == 0)  // they are same
        found_base = true;

      // Check if we are done searching
      if (found_tip && found_base)
        break;
    }

    // Check if we found both links
    if (!found_tip || !found_base)
    {
      throw std::runtime_error("Tip or base link(s) were not found in kinematic chain.");
    }
  }

  // Find the group we are editing based on the group name string
  srdf::Model::Group* searched_group = findGroupByName(group_name);

  // clear the old data
  searched_group->chains_.clear();

  // Save the data if there is data to save
  if (!tip.empty() && !base.empty())
  {
    searched_group->chains_.push_back(std::pair<std::string, std::string>(base, tip));
  }

  // Update the kinematic model with changes
  srdf_config_->updateRobotModel(true);
}

void PlanningGroups::createGroup(const std::string& group_name)
{
  srdf::Model::Group new_group;
  new_group.name_ = group_name;
  srdf_config_->getGroups().push_back(new_group);
  srdf_config_->updateRobotModel(true);
}

void PlanningGroups::setSubgroups(const std::string& selected_group_name, const std::vector<std::string>& subgroups)
{
  // Check for cycles -------------------------------

  // Create vector index of all nodes
  std::map<std::string, int> group_nodes;

  // Create vector of all nodes for use as id's
  int node_id = 0;
  for (const std::string& group_name : getGroupNames())
  {
    // Add string to vector
    group_nodes.insert(std::pair<std::string, int>(group_name, node_id));
    ++node_id;
  }

  // Create the empty graph
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS> Graph;
  Graph g(group_nodes.size());

  // Traverse the group list again, this time inserting subgroups into graph
  int from_id = 0;  // track the outer node we are on to reduce searches performed
  for (srdf::Model::Group& group : srdf_config_->getGroups())
  {
    // Check if group_it is same as current group
    if (group.name_ == selected_group_name)  // yes, same group
    {
      // add new subgroup list from widget, not old one. this way we can check for new cycles
      for (const std::string& to_string : subgroups)
      {
        // convert subgroup string to associated id
        int to_id = group_nodes[to_string];

        // Add edge from from_id to to_id
        add_edge(from_id, to_id, g);
      }
    }
    else  // this group is not the group we are editing, so just add subgroups from memory
    {
      // add new subgroup list from widget, not old one. this way we can check for new cycles
      for (const std::string& to_string : group.subgroups_)
      {
        // Get std::string of subgroup
        // convert subgroup string to associated id
        int to_id = group_nodes[to_string];

        // Add edge from from_id to to_id
        add_edge(from_id, to_id, g);
      }
    }

    ++from_id;
  }

  // Check for cycles
  bool has_cycle = false;
  CycleDetector vis(has_cycle);
  boost::depth_first_search(g, visitor(vis));

  if (has_cycle)
  {
    throw std::runtime_error("Depth first search reveals a cycle in the subgroups");
  }

  // Find the group we are editing based on the group name string
  srdf::Model::Group* searched_group = findGroupByName(selected_group_name);

  // save the data
  searched_group->subgroups_ = subgroups;

  // Update the kinematic model with changes
  srdf_config_->updateRobotModel(true);
}

void PlanningGroups::deleteGroup(const std::string& group_name)
{
  for (const std::string& pose_name : getPosesByGroup(group_name))
  {
    srdf_config_->removePoseByName(pose_name, group_name);
  }
  for (const std::string& eef_name : getEndEffectorsByGroup(group_name))
  {
    srdf_config_->removeEefByName(eef_name);
  }
  // delete actual group
  std::vector<srdf::Model::Group>& groups = srdf_config_->getGroups();
  std::vector<srdf::Model::Group>::iterator to_delete;
  for (std::vector<srdf::Model::Group>::iterator group_it = groups.begin(); group_it != groups.end(); ++group_it)
  {
    // check if this is the group we want to delete
    if (group_it->name_ == group_name)  // string match
    {
      to_delete = group_it;
    }
    else
    {
      auto& subgroups = group_it->subgroups_;
      std::vector<std::string>::iterator subgroup_it = std::find(subgroups.begin(), subgroups.end(), group_name);
      while (subgroup_it != subgroups.end())
      {
        subgroups.erase(subgroup_it);
        subgroup_it = std::find(subgroups.begin(), subgroups.end(), group_name);
      }
    }
  }
  groups.erase(to_delete);

  group_meta_config_->deleteGroup(group_name);

  srdf_config_->updateRobotModel(true);
}

void PlanningGroups::renameGroup(const std::string& old_group_name, const std::string& new_group_name)
{
  srdf::Model::Group* searched_group = findGroupByName(old_group_name);
  // Change group name
  searched_group->name_ = new_group_name;

  // Change all references to this group name in other subgroups
  // Loop through every group
  for (srdf::Model::Group& group : srdf_config_->getGroups())
  {
    // Loop through every subgroup
    for (std::string& subgroup : group.subgroups_)
    {
      // Check if that subgroup references old group name. if so, update it
      if (subgroup.compare(old_group_name) == 0)  // same name
      {
        subgroup.assign(new_group_name);  // updated
      }
    }
  }

  // Change all references to this group name in the end effectors screen
  for (srdf::Model::EndEffector& eef : srdf_config_->getEndEffectors())
  {
    // Check if this eef's parent group references old group name. if so, update it
    if (eef.parent_group_ == old_group_name)  // same name
    {
      RCLCPP_DEBUG_STREAM((*logger_), "Changed eef '" << eef.name_ << "' to new parent group name " << new_group_name);
      eef.parent_group_ = new_group_name;  // updated
    }

    // Check if this eef's group references old group name. if so, update it
    if (eef.component_group_.compare(old_group_name) == 0)  // same name
    {
      RCLCPP_DEBUG_STREAM((*logger_), "Changed eef '" << eef.name_ << "' to new group name " << new_group_name);
      eef.component_group_ = new_group_name;  // updated
    }
  }

  // Change all references to this group name in the robot poses screen
  for (srdf::Model::GroupState& gs : srdf_config_->getGroupStates())
  {
    // Check if this eef's parent group references old group name. if so, update it
    if (gs.group_ == old_group_name)  // same name
    {
      RCLCPP_DEBUG_STREAM((*logger_), "Changed group state group '" << gs.group_ << "' to new parent group name "
                                                                    << new_group_name);
      gs.group_ = new_group_name;  // updated
    }
  }

  group_meta_config_->renameGroup(old_group_name, new_group_name);

  // Now update the robot model based on our changed to the SRDF
  srdf_config_->updateRobotModel(true);
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
