/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Dave Coleman */

#include <moveit/setup_assistant/tools/moveit_config_data.hpp>
// Reading/Writing Files
#include <iostream>  // For writing yaml and launch files
#include <fstream>
#include <boost/filesystem/path.hpp>        // for creating folders/files
#include <boost/filesystem/operations.hpp>  // is_regular_file, is_directory, etc.
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/predicate.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>  // for getting file path for loading images

// OMPL version
#include <ompl/config.h>

namespace moveit_setup_assistant
{
// File system
namespace fs = boost::filesystem;

// ******************************************************************************************
// Constructor
// ******************************************************************************************
MoveItConfigData::MoveItConfigData() : config_pkg_generated_timestamp_(0)
{
  // Not in debug mode
  debug_ = false;

  // Get MoveIt Setup Assistant package path
  setup_assistant_path_ = ament_index_cpp::get_package_share_directory("moveit_setup_assistant");
  if (setup_assistant_path_.empty())
  {
    setup_assistant_path_ = ".";
  }
}

// ******************************************************************************************
// Destructor
// ******************************************************************************************
MoveItConfigData::~MoveItConfigData() = default;

// ******************************************************************************************
// Output OMPL Planning config files
// ******************************************************************************************
bool MoveItConfigData::outputOMPLPlanningYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // Output every available planner ---------------------------------------------------
  emitter << YAML::Key << "planner_configs";

  emitter << YAML::Value << YAML::BeginMap;

  std::vector<OMPLPlannerDescription> planner_des = getOMPLPlanners();

  // Add Planners with parameter values
  std::vector<std::string> pconfigs;
  for (OMPLPlannerDescription& planner_de : planner_des)
  {
    std::string defaultconfig = planner_de.name_;
    emitter << YAML::Key << defaultconfig;
    emitter << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "type" << YAML::Value << "geometric::" + planner_de.name_;
    for (OmplPlanningParameter& ompl_planner_param : planner_de.parameter_list_)
    {
      emitter << YAML::Key << ompl_planner_param.name;
      emitter << YAML::Value << ompl_planner_param.value;
      emitter << YAML::Comment(ompl_planner_param.comment);
    }
    emitter << YAML::EndMap;

    pconfigs.push_back(defaultconfig);
  }

  // End of every avail planner
  emitter << YAML::EndMap;

  // Output every group and the planners it can use ----------------------------------
  for (srdf::Model::Group& group : srdf_->groups_)
  {
    emitter << YAML::Key << group.name_;
    emitter << YAML::Value << YAML::BeginMap;
    // Output associated planners
    if (!group_meta_data_[group.name_].default_planner_.empty())
      emitter << YAML::Key << "default_planner_config" << YAML::Value << group_meta_data_[group.name_].default_planner_;
    emitter << YAML::Key << "planner_configs";
    emitter << YAML::Value << YAML::BeginSeq;
    for (const std::string& pconfig : pconfigs)
      emitter << pconfig;
    emitter << YAML::EndSeq;

    // Output projection_evaluator
    std::string projection_joints = decideProjectionJoints(group.name_);
    if (!projection_joints.empty())
    {
      emitter << YAML::Key << "projection_evaluator";
      emitter << YAML::Value << projection_joints;
      // OMPL collision checking discretization
      emitter << YAML::Key << "longest_valid_segment_fraction";
      emitter << YAML::Value << "0.005";
    }

    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to open file for writing " << file_path);
    return false;
  }

  output_stream << emitter.c_str() << '\n';
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Output STOMP Planning config files
// ******************************************************************************************
bool MoveItConfigData::outputSTOMPPlanningYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // Add STOMP default for every group
  for (srdf::Model::Group& group : srdf_->groups_)
  {
    emitter << YAML::Key << "stomp/" + group.name_;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "group_name";
    emitter << YAML::Value << group.name_;

    emitter << YAML::Key << "optimization";
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "num_timesteps";
    emitter << YAML::Value << "60";
    emitter << YAML::Key << "num_iterations";
    emitter << YAML::Value << "40";
    emitter << YAML::Key << "num_iterations_after_valid";
    emitter << YAML::Value << "0";
    emitter << YAML::Key << "num_rollouts";
    emitter << YAML::Value << "30";
    emitter << YAML::Key << "max_rollouts";
    emitter << YAML::Value << "30";
    emitter << YAML::Key << "initialization_method";
    emitter << YAML::Value << "1";
    emitter << YAML::Comment("[1 : LINEAR_INTERPOLATION, 2 : CUBIC_POLYNOMIAL, 3 : MININUM_CONTROL_COST]");
    emitter << YAML::Key << "control_cost_weight";
    emitter << YAML::Value << "0.0";
    emitter << YAML::EndMap;

    emitter << YAML::Key << "task";
    emitter << YAML::BeginMap;

    emitter << YAML::Key << "noise_generator";
    emitter << YAML::BeginSeq;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class";
    emitter << YAML::Value << "stomp_moveit/NormalDistributionSampling";
    emitter << YAML::Key << "stddev";
    emitter << YAML::Flow;
    const moveit::core::JointModelGroup* joint_model_group = getRobotModel()->getJointModelGroup(group.name_);
    const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getActiveJointModels();
    std::vector<float> stddev(joint_models.size(), 0.05);
    emitter << YAML::Value << stddev;
    emitter << YAML::EndMap;
    emitter << YAML::EndSeq;

    emitter << YAML::Key << "cost_functions";
    emitter << YAML::BeginSeq;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class";
    emitter << YAML::Value << "stomp_moveit/CollisionCheck";
    emitter << YAML::Key << "collision_penalty";
    emitter << YAML::Value << "1.0";
    emitter << YAML::Key << "cost_weight";
    emitter << YAML::Value << "1.0";
    emitter << YAML::Key << "kernel_window_percentage";
    emitter << YAML::Value << "0.2";
    emitter << YAML::Key << "longest_valid_joint_move";
    emitter << YAML::Value << "0.05";
    emitter << YAML::EndMap;
    emitter << YAML::EndSeq;

    emitter << YAML::Key << "noisy_filters";
    emitter << YAML::BeginSeq;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class";
    emitter << YAML::Value << "stomp_moveit/JointLimits";
    emitter << YAML::Key << "lock_start";
    emitter << YAML::Value << "True";
    emitter << YAML::Key << "lock_goal";
    emitter << YAML::Value << "True";
    emitter << YAML::EndMap;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class";
    emitter << YAML::Value << "stomp_moveit/MultiTrajectoryVisualization";
    emitter << YAML::Key << "line_width";
    emitter << YAML::Value << "0.02";
    emitter << YAML::Key << "rgb";
    emitter << YAML::Flow;
    std::vector<float> noisy_filters_rgb{ 255, 255, 0 };
    emitter << YAML::Value << noisy_filters_rgb;
    emitter << YAML::Key << "marker_array_topic";
    emitter << YAML::Value << "stomp_trajectories";
    emitter << YAML::Key << "marker_namespace";
    emitter << YAML::Value << "noisy";
    emitter << YAML::EndMap;
    emitter << YAML::EndSeq;

    emitter << YAML::Key << "update_filters";
    emitter << YAML::BeginSeq;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class";
    emitter << YAML::Value << "stomp_moveit/PolynomialSmoother";
    emitter << YAML::Key << "poly_order";
    emitter << YAML::Value << "6";
    emitter << YAML::EndMap;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class";
    emitter << YAML::Value << "stomp_moveit/TrajectoryVisualization";
    emitter << YAML::Key << "line_width";
    emitter << YAML::Value << "0.05";
    emitter << YAML::Key << "rgb";
    emitter << YAML::Flow;
    std::vector<float> update_filters_rgb{ 0, 191, 255 };
    emitter << YAML::Value << update_filters_rgb;
    emitter << YAML::Key << "error_rgb";
    emitter << YAML::Flow;
    std::vector<float> update_filters_error_rgb{ 255, 0, 0 };
    emitter << YAML::Value << update_filters_error_rgb;
    emitter << YAML::Key << "publish_intermediate";
    emitter << YAML::Value << "True";
    emitter << YAML::Key << "marker_topic";
    emitter << YAML::Value << "stomp_trajectory";
    emitter << YAML::Key << "marker_namespace";
    emitter << YAML::Value << "optimized";
    emitter << YAML::EndMap;
    emitter << YAML::EndSeq;

    emitter << YAML::EndMap;
    emitter << YAML::EndMap;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to open file for writing " << file_path);
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

bool MoveItConfigData::outputFakeControllersYAML(const std::string& file_path)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  emitter << YAML::Key << "controller_list";
  emitter << YAML::Value << YAML::BeginSeq;

  // Loop through groups
  for (srdf::Model::Group& group : srdf_->groups_)
  {
    // Get list of associated joints
    const moveit::core::JointModelGroup* joint_model_group = getRobotModel()->getJointModelGroup(group.name_);
    emitter << YAML::BeginMap;
    const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getActiveJointModels();
    emitter << YAML::Key << "name";
    emitter << YAML::Value << "fake_" + group.name_ + "_controller";
    emitter << YAML::Key << "type";
    emitter << YAML::Value << "$(arg fake_execution_type)";
    emitter << YAML::Key << "joints";
    emitter << YAML::Value << YAML::BeginSeq;

    // Iterate through the joints
    for (const moveit::core::JointModel* joint : joint_models)
    {
      if (joint->isPassive() || joint->getMimic() != nullptr || joint->getType() == moveit::core::JointModel::FIXED)
        continue;
      emitter << joint->getName();
    }
    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
  }

  emitter << YAML::EndSeq;

  // Add an initial pose for each group
  emitter << YAML::Key << "initial" << YAML::Comment("Define initial robot poses per group");

  bool poses_found = false;
  std::string default_group_name;
  for (const srdf::Model::Group& group : srdf_->groups_)
  {
    if (default_group_name.empty())
      default_group_name = group.name_;
    for (const srdf::Model::GroupState& group_state : srdf_->group_states_)
    {
      if (group.name_ == group_state.group_)
      {
        if (!poses_found)
        {
          poses_found = true;
          emitter << YAML::Value << YAML::BeginSeq;
        }
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "group";
        emitter << YAML::Value << group.name_;
        emitter << YAML::Key << "pose";
        emitter << YAML::Value << group_state.name_;
        emitter << YAML::EndMap;
        break;
      }
    }
  }
  if (poses_found)
    emitter << YAML::EndSeq;
  else
  {
    // Add commented lines to show how the feature can be used
    if (default_group_name.empty())
      default_group_name = "group";
    emitter << YAML::Newline;
    emitter << YAML::Comment(" - group: " + default_group_name) << YAML::Newline;
    emitter << YAML::Comment("   pose: home") << YAML::Newline;

    // Add empty list for valid yaml
    emitter << YAML::BeginSeq;
    emitter << YAML::EndSeq;
  }

  emitter << YAML::EndMap;

  std::ofstream output_stream(file_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to open file for writing " << file_path);
    return false;
  }

  output_stream << emitter.c_str();
  output_stream.close();

  return true;  // file created successfully
}

std::map<std::string, double> MoveItConfigData::getInitialJoints() const
{
  std::map<std::string, double> joints;
  for (const srdf::Model::Group& group : srdf_->groups_)
  {
    // use first pose of each group as initial pose
    for (const srdf::Model::GroupState& group_state : srdf_->group_states_)
    {
      if (group.name_ != group_state.group_)
        continue;
      for (const auto& pair : group_state.joint_values_)
      {
        if (pair.second.size() != 1)
          continue;  // only handle simple joints here
        joints[pair.first] = pair.second.front();
      }
      break;
    }
  }
  return joints;
}

std::vector<OMPLPlannerDescription> MoveItConfigData::getOMPLPlanners() const
{
  std::vector<OMPLPlannerDescription> planner_des;

  OMPLPlannerDescription aps("AnytimePathShortening", "geometric");
  aps.addParameter("shortcut", "true", "Attempt to shortcut all new solution paths");
  aps.addParameter("hybridize", "true", "Compute hybrid solution trajectories");
  aps.addParameter("max_hybrid_paths", "24", "Number of hybrid paths generated per iteration");
  aps.addParameter("num_planners", "4", "The number of default planners to use for planning");
// TODO: remove when ROS Melodic and older are no longer supported
#if OMPL_VERSION_VALUE >= 1005000
  // This parameter was added in OMPL 1.5.0
  aps.addParameter("planners", "",
                   "A comma-separated list of planner types (e.g., \"PRM,EST,RRTConnect\""
                   "Optionally, planner parameters can be passed to change the default:"
                   "\"PRM[max_nearest_neighbors=5],EST[goal_bias=.5],RRT[range=10. goal_bias=.1]\"");
#endif
  planner_des.push_back(aps);

  OMPLPlannerDescription sbl("SBL", "geometric");
  sbl.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  planner_des.push_back(sbl);

  OMPLPlannerDescription est("EST", "geometric");
  est.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0 setup()");
  est.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  planner_des.push_back(est);

  OMPLPlannerDescription lbkpiece("LBKPIECE", "geometric");
  lbkpiece.addParameter("range", "0.0",
                        "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                        "setup()");
  lbkpiece.addParameter("border_fraction", "0.9", "Fraction of time focused on boarder default: 0.9");
  lbkpiece.addParameter("min_valid_path_fraction", "0.5", "Accept partially valid moves above fraction. default: 0.5");
  planner_des.push_back(lbkpiece);

  OMPLPlannerDescription bkpiece("BKPIECE", "geometric");
  bkpiece.addParameter("range", "0.0",
                       "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                       "setup()");
  bkpiece.addParameter("border_fraction", "0.9", "Fraction of time focused on boarder default: 0.9");
  bkpiece.addParameter("failed_expansion_score_factor", "0.5",
                       "When extending motion fails, scale score by factor. "
                       "default: 0.5");
  bkpiece.addParameter("min_valid_path_fraction", "0.5", "Accept partially valid moves above fraction. default: 0.5");
  planner_des.push_back(bkpiece);

  OMPLPlannerDescription kpiece("KPIECE", "geometric");
  kpiece.addParameter("range", "0.0",
                      "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                      "setup()");
  kpiece.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  kpiece.addParameter("border_fraction", "0.9", "Fraction of time focused on boarder default: 0.9 (0.0,1.]");
  kpiece.addParameter("failed_expansion_score_factor", "0.5",
                      "When extending motion fails, scale score by factor. "
                      "default: 0.5");
  kpiece.addParameter("min_valid_path_fraction", "0.5", "Accept partially valid moves above fraction. default: 0.5");
  planner_des.push_back(kpiece);

  OMPLPlannerDescription rrt("RRT", "geometric");
  rrt.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  rrt.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  planner_des.push_back(rrt);

  OMPLPlannerDescription rrt_connect("RRTConnect", "geometric");
  rrt_connect.addParameter("range", "0.0",
                           "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                           "setup()");
  planner_des.push_back(rrt_connect);

  OMPLPlannerDescription rr_tstar("RRTstar", "geometric");
  rr_tstar.addParameter("range", "0.0",
                        "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                        "setup()");
  rr_tstar.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  rr_tstar.addParameter("delay_collision_checking", "1",
                        "Stop collision checking as soon as C-free parent found. "
                        "default 1");
  planner_des.push_back(rr_tstar);

  OMPLPlannerDescription trrt("TRRT", "geometric");
  trrt.addParameter("range", "0.0", "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()");
  trrt.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  trrt.addParameter("max_states_failed", "10", "when to start increasing temp. default: 10");
  trrt.addParameter("temp_change_factor", "2.0", "how much to increase or decrease temp. default: 2.0");
  trrt.addParameter("min_temperature", "10e-10", "lower limit of temp change. default: 10e-10");
  trrt.addParameter("init_temperature", "10e-6", "initial temperature. default: 10e-6");
  trrt.addParameter("frountier_threshold", "0.0",
                    "dist new state to nearest neighbor to disqualify as frontier. "
                    "default: 0.0 set in setup()");
  trrt.addParameter("frountierNodeRatio", "0.1", "1/10, or 1 nonfrontier for every 10 frontier. default: 0.1");
  trrt.addParameter("k_constant", "0.0", "value used to normalize expression. default: 0.0 set in setup()");
  planner_des.push_back(trrt);

  OMPLPlannerDescription prm("PRM", "geometric");
  prm.addParameter("max_nearest_neighbors", "10", "use k nearest neighbors. default: 10");
  planner_des.push_back(prm);

  OMPLPlannerDescription pr_mstar("PRMstar", "geometric");  // no declares in code
  planner_des.push_back(pr_mstar);

  OMPLPlannerDescription fmt("FMT", "geometric");
  fmt.addParameter("num_samples", "1000", "number of states that the planner should sample. default: 1000");
  fmt.addParameter("radius_multiplier", "1.1", "multiplier used for the nearest neighbors search radius. default: 1.1");
  fmt.addParameter("nearest_k", "1", "use Knearest strategy. default: 1");
  fmt.addParameter("cache_cc", "1", "use collision checking cache. default: 1");
  fmt.addParameter("heuristics", "0", "activate cost to go heuristics. default: 0");
  fmt.addParameter("extended_fmt", "1",
                   "activate the extended FMT*: adding new samples if planner does not finish "
                   "successfully. default: 1");
  planner_des.push_back(fmt);

  OMPLPlannerDescription bfmt("BFMT", "geometric");
  bfmt.addParameter("num_samples", "1000", "number of states that the planner should sample. default: 1000");
  bfmt.addParameter("radius_multiplier", "1.0",
                    "multiplier used for the nearest neighbors search radius. default: "
                    "1.0");
  bfmt.addParameter("nearest_k", "1", "use the Knearest strategy. default: 1");
  bfmt.addParameter("balanced", "0",
                    "exploration strategy: balanced true expands one tree every iteration. False will "
                    "select the tree with lowest maximum cost to go. default: 1");
  bfmt.addParameter("optimality", "1",
                    "termination strategy: optimality true finishes when the best possible path is "
                    "found. Otherwise, the algorithm will finish when the first feasible path is "
                    "found. default: 1");
  bfmt.addParameter("heuristics", "1", "activates cost to go heuristics. default: 1");
  bfmt.addParameter("cache_cc", "1", "use the collision checking cache. default: 1");
  bfmt.addParameter("extended_fmt", "1",
                    "Activates the extended FMT*: adding new samples if planner does not finish "
                    "successfully. default: 1");
  planner_des.push_back(bfmt);

  OMPLPlannerDescription pdst("PDST", "geometric");
  rrt.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability? default: 0.05");
  planner_des.push_back(pdst);

  OMPLPlannerDescription stride("STRIDE", "geometric");
  stride.addParameter("range", "0.0",
                      "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                      "setup()");
  stride.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  stride.addParameter("use_projected_distance", "0",
                      "whether nearest neighbors are computed based on distances in a "
                      "projection of the state rather distances in the state space "
                      "itself. default: 0");
  stride.addParameter("degree", "16",
                      "desired degree of a node in the Geometric Near-neightbor Access Tree (GNAT). "
                      "default: 16");
  stride.addParameter("max_degree", "18", "max degree of a node in the GNAT. default: 12");
  stride.addParameter("min_degree", "12", "min degree of a node in the GNAT. default: 12");
  stride.addParameter("max_pts_per_leaf", "6", "max points per leaf in the GNAT. default: 6");
  stride.addParameter("estimated_dimension", "0.0", "estimated dimension of the free space. default: 0.0");
  stride.addParameter("min_valid_path_fraction", "0.2", "Accept partially valid moves above fraction. default: 0.2");
  planner_des.push_back(stride);

  OMPLPlannerDescription bi_trrt("BiTRRT", "geometric");
  bi_trrt.addParameter("range", "0.0",
                       "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                       "setup()");
  bi_trrt.addParameter("temp_change_factor", "0.1", "how much to increase or decrease temp. default: 0.1");
  bi_trrt.addParameter("init_temperature", "100", "initial temperature. default: 100");
  bi_trrt.addParameter("frountier_threshold", "0.0",
                       "dist new state to nearest neighbor to disqualify as frontier. "
                       "default: 0.0 set in setup()");
  bi_trrt.addParameter("frountier_node_ratio", "0.1", "1/10, or 1 nonfrontier for every 10 frontier. default: 0.1");
  bi_trrt.addParameter("cost_threshold", "1e300",
                       "the cost threshold. Any motion cost that is not better will not be "
                       "expanded. default: inf");
  planner_des.push_back(bi_trrt);

  OMPLPlannerDescription lbtrrt("LBTRRT", "geometric");
  lbtrrt.addParameter("range", "0.0",
                      "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                      "setup()");
  lbtrrt.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  lbtrrt.addParameter("epsilon", "0.4", "optimality approximation factor. default: 0.4");
  planner_des.push_back(lbtrrt);

  OMPLPlannerDescription bi_est("BiEST", "geometric");
  bi_est.addParameter("range", "0.0",
                      "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                      "setup()");
  planner_des.push_back(bi_est);

  OMPLPlannerDescription proj_est("ProjEST", "geometric");
  proj_est.addParameter("range", "0.0",
                        "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                        "setup()");
  proj_est.addParameter("goal_bias", "0.05", "When close to goal select goal, with this probability. default: 0.05");
  planner_des.push_back(proj_est);

  OMPLPlannerDescription lazy_prm("LazyPRM", "geometric");
  lazy_prm.addParameter("range", "0.0",
                        "Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on "
                        "setup()");
  planner_des.push_back(lazy_prm);

  OMPLPlannerDescription lazy_pr_mstar("LazyPRMstar", "geometric");  // no declares in code
  planner_des.push_back(lazy_pr_mstar);

  OMPLPlannerDescription spars("SPARS", "geometric");
  spars.addParameter("stretch_factor", "3.0",
                     "roadmap spanner stretch factor. multiplicative upper bound on path "
                     "quality. It does not make sense to make this parameter more than 3. "
                     "default: 3.0");
  spars.addParameter("sparse_delta_fraction", "0.25",
                     "delta fraction for connection distance. This value represents "
                     "the visibility range of sparse samples. default: 0.25");
  spars.addParameter("dense_delta_fraction", "0.001", "delta fraction for interface detection. default: 0.001");
  spars.addParameter("max_failures", "1000", "maximum consecutive failure limit. default: 1000");
  planner_des.push_back(spars);

  OMPLPlannerDescription spar_stwo("SPARStwo", "geometric");
  spar_stwo.addParameter("stretch_factor", "3.0",
                         "roadmap spanner stretch factor. multiplicative upper bound on path "
                         "quality. It does not make sense to make this parameter more than 3. "
                         "default: 3.0");
  spar_stwo.addParameter("sparse_delta_fraction", "0.25",
                         "delta fraction for connection distance. This value represents "
                         "the visibility range of sparse samples. default: 0.25");
  spar_stwo.addParameter("dense_delta_fraction", "0.001", "delta fraction for interface detection. default: 0.001");
  spar_stwo.addParameter("max_failures", "5000", "maximum consecutive failure limit. default: 5000");
  planner_des.push_back(spar_stwo);

  return planner_des;
}

// ******************************************************************************************
// Helper function to get the default start pose for moveit_sim_hw_interface
// ******************************************************************************************
srdf::Model::GroupState MoveItConfigData::getDefaultStartPose()
{
  if (!srdf_->group_states_.empty())
    return srdf_->group_states_[0];
  else
    return srdf::Model::GroupState{ .name_ = "todo_state_name", .group_ = "todo_group_name", .joint_values_ = {} };
}

// ******************************************************************************************
// Decide the best two joints to be used for the projection evaluator
// ******************************************************************************************
std::string MoveItConfigData::decideProjectionJoints(const std::string& planning_group)
{
  std::string joint_pair = "";

  // Retrieve pointer to the shared kinematic model
  const moveit::core::RobotModelConstPtr& model = getRobotModel();

  // Error check
  if (!model->hasJointModelGroup(planning_group))
    return joint_pair;

  // Get the joint model group
  const moveit::core::JointModelGroup* group = model->getJointModelGroup(planning_group);

  // get vector of joint names
  const std::vector<std::string>& joints = group->getJointModelNames();

  if (joints.size() >= 2)
  {
    // Check that the first two joints have only 1 variable
    if (group->getJointModel(joints[0])->getVariableCount() == 1 &&
        group->getJointModel(joints[1])->getVariableCount() == 1)
    {
      // Just choose the first two joints.
      joint_pair = "joints(" + joints[0] + "," + joints[1] + ")";
    }
  }

  return joint_pair;
}

bool MoveItConfigData::inputOMPLYAML(const std::string& file_path)
{
  // Load file
  std::ifstream input_stream(file_path.c_str());
  if (!input_stream.good())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to open file for reading " << file_path);
    return false;
  }

  // Begin parsing
  try
  {
    YAML::Node doc = YAML::Load(input_stream);

    // Loop through all groups
    for (YAML::const_iterator group_it = doc.begin(); group_it != doc.end(); ++group_it)
    {
      // get group name
      const std::string group_name = group_it->first.as<std::string>();

      // compare group name found to list of groups in group_meta_data_
      std::map<std::string, GroupMetaData>::iterator group_meta_it;
      group_meta_it = group_meta_data_.find(group_name);
      if (group_meta_it != group_meta_data_.end())
      {
        std::string planner;
        getYamlProperty(group_it->second, "default_planner_config", planner);
        std::size_t pos = planner.find("kConfigDefault");
        if (pos != std::string::npos)
        {
          planner = planner.substr(0, pos);
        }
        group_meta_data_[group_name].default_planner_ = planner;
      }
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    RCLCPP_ERROR_STREAM(LOGGER, e.what());
    return false;
  }
  return true;
}

// ******************************************************************************************
// Input planning_context.launch file
// ******************************************************************************************
bool MoveItConfigData::inputPlanningContextLaunch(const std::string& file_path)
{
  TiXmlDocument launch_document(file_path);
  if (!launch_document.LoadFile())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed parsing " << file_path);
    return false;
  }

  // find the kinematics section
  TiXmlHandle doc(&launch_document);
  TiXmlElement* kinematics_group = doc.FirstChild("launch").FirstChild("group").ToElement();
  while (kinematics_group && kinematics_group->Attribute("ns") &&
         kinematics_group->Attribute("ns") != std::string("$(arg robot_description)_kinematics"))
  {
    kinematics_group = kinematics_group->NextSiblingElement("group");
  }
  if (!kinematics_group)
  {
    RCLCPP_ERROR(LOGGER, "<group ns=\"$(arg robot_description)_kinematics\"> not found");
    return false;
  }

  // iterate over all <rosparam namespace="group" file="..."/> elements
  // and if 'group' matches an existing group, copy the filename
  for (TiXmlElement* kinematics_parameter_file = kinematics_group->FirstChildElement("rosparam");
       kinematics_parameter_file; kinematics_parameter_file = kinematics_parameter_file->NextSiblingElement("rosparam"))
  {
    const char* ns = kinematics_parameter_file->Attribute("ns");
    if (ns && (group_meta_data_.find(ns) != group_meta_data_.end()))
    {
      group_meta_data_[ns].kinematics_parameters_file_ = kinematics_parameter_file->Attribute("file");
    }
  }

  return true;
}

// ******************************************************************************************
// Make the full URDF path using the loaded .setup_assistant data
// ******************************************************************************************
bool MoveItConfigData::createFullURDFPath()
{
  boost::trim(urdf_pkg_name_);

  // Check if a package name was provided
  if (urdf_pkg_name_.empty() || urdf_pkg_name_ == "\"\"")
  {
    urdf_path_ = urdf_pkg_relative_path_;
    urdf_pkg_name_.clear();
  }
  else
  {
    // Check that ROS can find the package
    std::string robot_desc_pkg_path = ament_index_cpp::get_package_share_directory(urdf_pkg_name_);

    if (robot_desc_pkg_path.empty())
    {
      urdf_path_.clear();
      return false;
    }

    // Append the relative URDF url path
    urdf_path_ = appendPaths(robot_desc_pkg_path, urdf_pkg_relative_path_);
  }

  // Check that this file exits -------------------------------------------------
  return fs::is_regular_file(urdf_path_);
}
}  // namespace moveit_setup_assistant
