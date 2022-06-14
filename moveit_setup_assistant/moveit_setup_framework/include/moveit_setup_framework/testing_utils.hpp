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

#include <moveit_setup_framework/data_warehouse.hpp>
#include <gtest/gtest.h>
#include <filesystem>

namespace moveit_setup
{
/**
 * @brief Test environment with DataWarehouse setup and help for generating files in a temp dir
 */
class MoveItSetupTest : public ::testing::Test
{
protected:
  /**
   * @brief Initialize the node, DataWarehouse and output dir.
   *
   * You may want to override this method to register data types after calling MoveItSetupTest::SetUp()
   */
  void SetUp() override
  {
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("test_node");
    config_data_ = std::make_shared<moveit_setup::DataWarehouse>(node);
    output_dir_ = std::filesystem::temp_directory_path() / "moveit_setup_test";
  }

  /**
   * @brief Helper function for generating all the files for a particular config to a temporary directory
   *
   * Uses the same template/config_name structure as DataWarehouse::get
   */
  template <typename T>
  void generateFiles(const std::string& config_name)
  {
    std::vector<moveit_setup::GeneratedFilePtr> files;
    config_data_->get<T>(config_name)->collectFiles(output_dir_, placeholder_timestamp_, files);
    for (moveit_setup::GeneratedFilePtr& file : files)
    {
      file->write();
    }
  }

  /**
   * @brief Clean up by removing all files when complete
   *
   * Note that you only want to set delete_when_finished_ to false when testing locally
   */
  void TearDown() override
  {
    if (delete_when_finished_)
      std::filesystem::remove_all(output_dir_);
  }

  moveit_setup::DataWarehousePtr config_data_;
  std::filesystem::path output_dir_;
  moveit_setup::GeneratedTime placeholder_timestamp_;
  bool delete_when_finished_{ true };  // Set to false to keep the files around
};
}  // namespace moveit_setup
