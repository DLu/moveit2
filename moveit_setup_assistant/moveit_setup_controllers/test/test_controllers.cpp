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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_controllers");

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
  config_data_->get<moveit_setup::controllers::ControlXacroConfig>("control_xacro")->loadFromDescription();
  generateFiles<ROS2ControllersConfig>("ros2_controllers");
  generateFiles<MoveItControllersConfig>("moveit_controllers");

  // TODO: Check files
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
