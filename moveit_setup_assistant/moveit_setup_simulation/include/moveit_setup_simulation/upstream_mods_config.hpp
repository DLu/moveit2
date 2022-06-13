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

#include <moveit_setup_framework/config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>

namespace moveit_setup
{
namespace simulation
{
class UpstreamModsConfig : public SetupConfig
{
public:
  void onInit() override;

  void load();

  bool isConfigured() const override
  {
    return true;
  }

  void collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                    std::vector<GeneratedFilePtr>& files) override;

protected:
  struct ModifiedFile
  {
    std::filesystem::path path;
    std::shared_ptr<tinyxml2::XMLDocument> xml_doc;  // shared pointer for insertion into vector
  };

  class GeneratedModifiedFile : public GeneratedFile
  {
  public:
    GeneratedModifiedFile(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                          ModifiedFile& mf)
      : GeneratedFile(package_path, last_gen_time), mf_(mf)
    {
    }

    bool hasChanges() const override
    {
      return true;
    }

    std::filesystem::path getRelativePath() const override
    {
      return mf_.path;
    }

    std::string getDescription() const override
    {
      return "Original URDF file modified to have required simulation properties";
    }

    bool write() override;

  protected:
    ModifiedFile& mf_;
  };

  void loadFromURDF();

  std::shared_ptr<URDFConfig> urdf_config_;
  std::vector<ModifiedFile> modified_files_;
};
}  // namespace simulation
}  // namespace moveit_setup
