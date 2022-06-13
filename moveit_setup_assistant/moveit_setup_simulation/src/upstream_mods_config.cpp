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

#include <moveit_setup_simulation/upstream_mods_config.hpp>

namespace moveit_setup
{
namespace simulation
{
void UpstreamModsConfig::onInit()
{
  urdf_config_ = config_data_->get<URDFConfig>("urdf");
}

void UpstreamModsConfig::load()
{
  loadFromURDF();
}

void UpstreamModsConfig::loadFromURDF()
{
  ModifiedFile urdf;
  urdf.path = urdf_config_->getURDFPath();
  urdf.xml_doc = std::make_shared<tinyxml2::XMLDocument>();
  urdf.xml_doc->Parse(urdf_config_->getURDFContents().c_str());

  // Read the URDF
  using tinyxml2::XMLElement;

  for (XMLElement* link_el = urdf.xml_doc->FirstChildElement("link"); link_el != nullptr;
       link_el = link_el->NextSiblingElement())
  {
    if (!link_el->FirstChildElement("collision"))
    {
      // only modify elements with collision tag
      continue;
    }

    XMLElement* inertial = uniqueInsert(*urdf.xml_doc, *link_el, "inertial");
    uniqueInsert(*urdf.xml_doc, *inertial, "mass", { { "value", "0.1" } });
    uniqueInsert(*urdf.xml_doc, *inertial, "origin", { { "xyz", "0 0 0" }, { "rpy", "0 0 0" } });
    uniqueInsert(*urdf.xml_doc, *inertial, "inertia",
                 { { "ixx", "0.03" },
                   { "iyy", "0.03" },
                   { "izz", "0.03" },
                   { "ixy", "0.0" },
                   { "ixz", "0.0" },
                   { "iyz", "0.0" } });
  }

  modified_files_.emplace_back(urdf);
}

bool UpstreamModsConfig::GeneratedModifiedFile::write()
{
  tinyxml2::XMLError ret = mf_.xml_doc->SaveFile(mf_.path.c_str());
  return ret == tinyxml2::XML_SUCCESS;
}

void UpstreamModsConfig::collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                                      std::vector<GeneratedFilePtr>& files)
{
  for (ModifiedFile& mf : modified_files_)
  {
    files.push_back(std::make_shared<GeneratedModifiedFile>(package_path, last_gen_time, mf));
  }
}

}  // namespace simulation
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::simulation::UpstreamModsConfig, moveit_setup::SetupConfig)
