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

#include <moveit_setup_controllers/included_xacro_config.hpp>

namespace moveit_setup
{
namespace simulation
{
class SimXacroConfig : public IncludedXacroConfig
{
public:
  /**@name SetupConfig overrides */
  /**@{*/
  void onInit() override;
  void loadPrevious(const std::filesystem::path& /*config_package_path*/, const YAML::Node& node) override;
  YAML::Node saveToYaml() const override;
  bool isConfigured() const override;
  /**@}*/

  /**@name IncludedXacroConfig overrides */
  /**@{*/
  std::string getFilepath() const override;
  bool hasChanges() const override;
  std::vector<std::pair<std::string, std::string>> getArguments() const override;
  std::vector<std::string> getCommands() const override;
  /**@}*/

  class GeneratedSimHeader : public TemplatedGeneratedFile
  {
  public:
    GeneratedSimHeader(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                       SimXacroConfig& parent)
      : TemplatedGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
    }

    std::filesystem::path getRelativePath() const override
    {
      return std::filesystem::path("config") / (parent_.urdf_config_->getRobotName() + ".simulation.xacro");
    }

    std::filesystem::path getTemplatePath() const override
    {
      return getSharePath("moveit_setup_simulation") / "templates" / "config" / "simulation.xacro";
    }

    std::string getDescription() const override
    {
      return "Macro definition for required simulation xacro additions.";
    }

    bool hasChanges() const override
    {
      return parent_.hasChanges();
    }

  protected:
    SimXacroConfig& parent_;
  };

  void collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                    std::vector<GeneratedFilePtr>& files) override
  {
    files.push_back(std::make_shared<GeneratedSimHeader>(package_path, last_gen_time, *this));
  }

  void collectVariables(std::vector<TemplateVariable>& variables) override;

protected:
  bool changed_;
};
}  // namespace simulation
}  // namespace moveit_setup
