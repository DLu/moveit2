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
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <tinyxml2.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_srdf");

using moveit_setup::getSharePath;
using moveit_setup::SRDFConfig;

class SRDFTest : public moveit_setup::MoveItSetupTest
{
protected:
  void SetUp() override
  {
    MoveItSetupTest::SetUp();
    srdf_config_ = config_data_->get<SRDFConfig>("srdf");
  }

  std::shared_ptr<SRDFConfig> srdf_config_;
};

TEST_F(SRDFTest, Empty)
{
  auto fanuc_path = getSharePath("moveit_resources_fanuc_description");
  config_data_->preloadWithURDFPath(fanuc_path / "urdf" / "fanuc.urdf");
  srdf_config_->updateRobotModel();
  generateFiles<SRDFConfig>("srdf");

  std::filesystem::path srdf_path = output_dir_ / "config" / "fanuc.srdf";
  ASSERT_TRUE(std::filesystem::is_regular_file(srdf_path));

  tinyxml2::XMLDocument srdf_xml;
  ASSERT_EQ(srdf_xml.LoadFile(srdf_path.c_str()), tinyxml2::XML_SUCCESS);

  auto root = srdf_xml.FirstChildElement("robot");
  EXPECT_EQ(std::string(root->Attribute("name")), std::string("fanuc"));
  EXPECT_TRUE(root->NoChildren());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
