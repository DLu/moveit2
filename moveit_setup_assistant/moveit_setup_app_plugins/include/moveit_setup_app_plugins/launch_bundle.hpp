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

#include <moveit_setup_framework/templates.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_setup_framework/utilities.hpp>

namespace moveit_setup_app_plugins
{
class LaunchBundle
{
public:
  LaunchBundle(const std::string& title, const std::string& description, const std::string& launch_name,
               const std::string& function_name)
    : title_(title), description_(description), launch_name_(launch_name), function_name_(function_name)
  {
  }

  const std::string& getTitle() const
  {
    return title_;
  }
  const std::string& getDescription() const
  {
    return description_;
  }
  unsigned int getID() const
  {
    return id_;
  }

  void setID(unsigned int id)
  {
    id_ = id;
  }

  bool operator<(const LaunchBundle& other) const
  {
    return title_ < other.title_;
  }

  class GenericLaunchTemplate : public moveit_setup_framework::TemplatedGeneratedFile
  {
  public:
    GenericLaunchTemplate(const std::string& package_path, const std::time_t& last_gen_time, const LaunchBundle& parent)
      : TemplatedGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
      relative_path_ = moveit_setup_framework::appendPaths("launch", parent_.launch_name_ + ".launch.py");
      template_path_ =
          moveit_setup_framework::appendPaths(ament_index_cpp::get_package_share_directory("moveit_setup_app_plugins"),
                                              "templates/launch/generic.launch.py");
    }

    std::string getRelativePath() const override
    {
      return relative_path_;
    }

    std::string getDescription() const override
    {
      return parent_.description_;
    }

    std::string getTemplatePath() const override
    {
      return template_path_;
    }

    bool hasChanges() const override
    {
      return false;
    }

    bool write() override
    {
      // Add function name as a TemplateVariable, then remove it
      variables_.push_back(moveit_setup_framework::TemplateVariable("FUNCTION_NAME", parent_.function_name_));
      bool ret = TemplatedGeneratedFile::write();
      variables_.pop_back();
      return ret;
    }

  protected:
    const LaunchBundle& parent_;
    std::string relative_path_, template_path_;
  };

  void collectFiles(const std::string& package_path, const std::time_t& last_gen_time,
                    std::vector<moveit_setup_framework::GeneratedFilePtr>& files) const
  {
    files.push_back(std::make_shared<GenericLaunchTemplate>(package_path, last_gen_time, *this));
  }

protected:
  std::string title_, description_, launch_name_, function_name_;
  unsigned int id_;
};
}  // namespace moveit_setup_app_plugins
