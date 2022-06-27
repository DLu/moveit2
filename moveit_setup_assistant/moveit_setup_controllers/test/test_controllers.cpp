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
#include <moveit_setup_framework/testing_utils.hpp>
#include <moveit_setup_controllers/ros2_controllers_config.hpp>
#include <moveit_setup_controllers/moveit_controllers_config.hpp>
#include <moveit_setup_controllers/ros2_controllers.hpp>
#include <moveit_setup_controllers/urdf_modifications.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_controllers");

using moveit_setup::expectYamlEquivalence;
using moveit_setup::getSharePath;
using moveit_setup::controllers::ControllerInfo;
using moveit_setup::controllers::MoveItControllersConfig;
using moveit_setup::controllers::ROS2ControllersConfig;

class ControllersTest : public moveit_setup::MoveItSetupTest
{
protected:
  void SetUp() override
  {
    MoveItSetupTest::SetUp();
    config_data_->registerType("moveit_controllers", "moveit_setup::controllers::MoveItControllersConfig");
    config_data_->registerType("ros2_controllers", "moveit_setup::controllers::ROS2ControllersConfig");
    config_data_->registerType("modified_urdf", "moveit_setup::ModifiedUrdfConfig");
    config_data_->registerType("control_xacro", "moveit_setup::controllers::ControlXacroConfig");
  }

  void loadURDFAndSRDF(const std::string& package_name, const std::string& urdf_key = "urdf",
                       const std::string& srdf_key = "srdf")
  {
    auto config_dir = getSharePath(package_name);
    settings_ = YAML::LoadFile(config_dir / ".setup_assistant")["moveit_setup_assistant_config"];
    config_data_->get<moveit_setup::URDFConfig>("urdf")->loadPrevious(config_dir, settings_[urdf_key]);
    auto srdf_config = config_data_->get<moveit_setup::SRDFConfig>("srdf");
    srdf_config->loadPrevious(config_dir, settings_[srdf_key]);
  }

  YAML::Node settings_;
};

TEST_F(ControllersTest, ParseFanuc)
{
  config_data_->preloadWithFullConfig("moveit_resources_fanuc_moveit_config");
  auto ros2_controllers_config = config_data_->get<ROS2ControllersConfig>("ros2_controllers");
  const std::vector<ControllerInfo>& controllers = ros2_controllers_config->getControllers();
  ASSERT_EQ(1u, controllers.size());
  const ControllerInfo& ci = controllers[0];
  EXPECT_EQ("fanuc_controller", ci.name_);
  EXPECT_EQ("joint_trajectory_controller/JointTrajectoryController", ci.type_);
  EXPECT_EQ(6u, ci.joints_.size());

  auto moveit_controllers_config = config_data_->get<MoveItControllersConfig>("moveit_controllers");
  const std::vector<ControllerInfo>& mcontrollers = moveit_controllers_config->getControllers();
  ASSERT_EQ(1u, mcontrollers.size());
  const ControllerInfo& mci = mcontrollers[0];
  EXPECT_EQ("fanuc_controller", mci.name_);
  EXPECT_EQ("FollowJointTrajectory", mci.type_);
  EXPECT_EQ(6u, mci.joints_.size());
}

TEST_F(ControllersTest, ParsePanda)
{
  config_data_->preloadWithFullConfig("moveit_resources_panda_moveit_config");
  auto ros2_controllers_config = config_data_->get<ROS2ControllersConfig>("ros2_controllers");
  const std::vector<ControllerInfo>& controllers = ros2_controllers_config->getControllers();
  ASSERT_EQ(2u, controllers.size());

  int offset = controllers[0].name_ == "panda_arm_controller" ? 0 : 1;
  const ControllerInfo& ci1 = controllers[offset];
  EXPECT_EQ("panda_arm_controller", ci1.name_);
  EXPECT_EQ("joint_trajectory_controller/JointTrajectoryController", ci1.type_);
  EXPECT_EQ(7u, ci1.joints_.size());

  const ControllerInfo& ci2 = controllers[1 - offset];
  EXPECT_EQ("panda_hand_controller", ci2.name_);
  EXPECT_EQ("position_controllers/GripperActionController", ci2.type_);
  EXPECT_EQ(1u, ci2.joints_.size());

  auto moveit_controllers_config = config_data_->get<MoveItControllersConfig>("moveit_controllers");
  const std::vector<ControllerInfo>& mcontrollers = moveit_controllers_config->getControllers();
  ASSERT_EQ(1u, mcontrollers.size());
  const ControllerInfo& mci1 = mcontrollers[offset];
  EXPECT_EQ("panda_arm_controller", mci1.name_);
  EXPECT_EQ("FollowJointTrajectory", mci1.type_);
  EXPECT_EQ(7u, mci1.joints_.size());
}

TEST_F(ControllersTest, OutputFanuc)
{
  config_data_->preloadWithFullConfig("moveit_resources_fanuc_moveit_config");
  auto control_config = config_data_->get<moveit_setup::controllers::ControlXacroConfig>("control_xacro");
  control_config->loadFromDescription();
  control_config->setControlInterfaces(control_config->getDefaultControlInterfaces());
  generateFiles<ROS2ControllersConfig>("ros2_controllers");
  generateFiles<MoveItControllersConfig>("moveit_controllers");

  std::filesystem::path original_config = getSharePath("moveit_resources_fanuc_moveit_config");
  for (const std::string& relative_path : { "config/moveit_controllers.yaml", "config/ros2_controllers.yaml" })
  {
    expectYamlEquivalence(output_dir_ / relative_path, original_config / relative_path);
  }
}

TEST_F(ControllersTest, OutputPanda)
{
  std::string package_name = "moveit_resources_panda_moveit_config";
  std::filesystem::path original_config = getSharePath(package_name);

  config_data_->get<moveit_setup::URDFConfig>("urdf")->loadFromPackage(package_name, "config/panda.urdf.xacro");
  auto srdf_config = config_data_->get<moveit_setup::SRDFConfig>("srdf");
  srdf_config->loadSRDFFile(original_config, "config/panda_arm_hand.srdf.xacro");

  auto control_config = config_data_->get<moveit_setup::controllers::ControlXacroConfig>("control_xacro");
  control_config->loadFromDescription();

  auto ros2_config = config_data_->get<MoveItControllersConfig>("ros2_controllers");
  ros2_config->loadPrevious(original_config, settings_["ros2_controllers"]);

  auto moveit_config = config_data_->get<MoveItControllersConfig>("moveit_controllers");
  moveit_config->setConfigurationFilename("config/gripper_moveit_controllers.yaml");
  moveit_config->loadPrevious(original_config, settings_["moveit_controllers"]);

  generateFiles<ROS2ControllersConfig>("ros2_controllers");
  generateFiles<MoveItControllersConfig>("moveit_controllers");

  for (const std::string& relative_path : { "config/gripper_moveit_controllers.yaml", "config/ros2_controllers.yaml" })
  {
    expectYamlEquivalence(output_dir_ / relative_path, original_config / relative_path);
  }
}

TEST_F(ControllersTest, OutputUBR)
{
  loadURDFAndSRDF("ubr1_moveit");
  std::filesystem::path original_config = getSharePath("ubr1_moveit");

  auto control_config = config_data_->get<moveit_setup::controllers::ControlXacroConfig>("control_xacro");
  control_config->loadFromDescription();
  control_config->setControlInterfaces(control_config->getDefaultControlInterfaces());

  auto ros2_config = config_data_->get<MoveItControllersConfig>("ros2_controllers");
  ros2_config->loadPrevious(original_config, settings_["ros2_controllers"]);

  auto moveit_config = config_data_->get<MoveItControllersConfig>("moveit_controllers");
  moveit_config->loadPrevious(original_config, settings_["moveit_controllers"]);

  generateFiles<ROS2ControllersConfig>("ros2_controllers");
  generateFiles<MoveItControllersConfig>("moveit_controllers");

  for (const std::string& relative_path : { "config/moveit_controllers.yaml", "config/ros2_controllers.yaml" })
  {
    expectYamlEquivalence(output_dir_ / relative_path, original_config / relative_path);
  }
}

TEST_F(ControllersTest, AddDefaultControllers)
{
  loadURDFAndSRDF("moveit_resources_panda_moveit_config", "URDF", "SRDF");

  auto ros2_controllers_config = config_data_->get<ROS2ControllersConfig>("ros2_controllers");

  // Initially no controllers
  EXPECT_EQ(ros2_controllers_config->getControllers().size(), 0u);

  // Run the setup step
  moveit_setup::controllers::ROS2Controllers setup_step;
  initializeStep(setup_step);

  // Adding default controllers, a controller for each planning group
  setup_step.addDefaultControllers();

  // Number of the planning groups defined in the model srdf
  size_t group_count = config_data_->get<moveit_setup::SRDFConfig>("srdf")->getGroups().size();

  // Test that addDefaultControllers() did accually add a controller for the new_group
  EXPECT_EQ(ros2_controllers_config->getControllers().size(), group_count);
}

TEST_F(ControllersTest, AddDefaultControllersToUBR)
{
  loadURDFAndSRDF("ubr1_moveit");

  auto control_config = config_data_->get<moveit_setup::controllers::ControlXacroConfig>("control_xacro");
  control_config->loadFromDescription();
  /*

  // Initially no controllers
  EXPECT_EQ(ros2_controllers_config->getControllers().size(), 0u);

  // Run the setup step
  moveit_setup::controllers::ROS2Controllers setup_step;
  initializeStep(setup_step);

  // Adding default controllers, a controller for each planning group
  setup_step.addDefaultControllers();

  // Number of the planning groups defined in the model srdf
  size_t group_count = srdf_config->getGroups().size();

  // Test that addDefaultControllers() did accually add a controller for the new_group
  EXPECT_EQ(ros2_controllers_config->getControllers().size(), group_count);*/
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
