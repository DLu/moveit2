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

#include <moveit_setup_simulation/simulation.hpp>
#include <tinyxml.h>

namespace moveit_setup_simulation
{
void Simulation::onInit()
{
  urdf_config_ = config_data_->get<moveit_setup_framework::URDFConfig>("urdf");
}

// ******************************************************************************************
// Helper function to get the controller that is controlling the joint
// ******************************************************************************************
std::string Simulation::getJointHardwareInterface(const std::string& joint_name)
{
  /** TODO: Need to port this - may require depending on moveit_setup_controllers ControllerConfig

  for (ControllerConfig& ros_control_config : controller_configs_)
  {
    std::vector<std::string>::iterator joint_it =
        std::find(ros_control_config.joints_.begin(), ros_control_config.joints_.end(), joint_name);
    if (joint_it != ros_control_config.joints_.end())
    {
      if (ros_control_config.type_.substr(0, 8) == "position")
        return "hardware_interface/PositionJointInterface";
      else if (ros_control_config.type_.substr(0, 8) == "velocity")
        return "hardware_interface/VelocityJointInterface";
      // As of writing this, available joint command interfaces are position, velocity and effort.
      else
        return "hardware_interface/EffortJointInterface";
    }
  }
  */
  // If the joint was not found in any controller return EffortJointInterface
  return "hardware_interface/EffortJointInterface";
}

// ******************************************************************************************
// Writes a Gazebo compatible robot URDF to gazebo_compatible_urdf_string_
// ******************************************************************************************
std::string Simulation::getGazeboCompatibleURDF()
{
  bool new_urdf_needed = false;
  TiXmlDocument urdf_document;

  // Used to convert XmlDocument to std string
  TiXmlPrinter printer;
  std::string urdf_string = urdf_config_->getURDFContents();
  urdf_document.Parse((const char*)urdf_string.c_str(), nullptr, TIXML_ENCODING_UTF8);
  try
  {
    for (TiXmlElement* doc_element = urdf_document.RootElement()->FirstChildElement(); doc_element != nullptr;
         doc_element = doc_element->NextSiblingElement())
    {
      if (static_cast<std::string>(doc_element->Value()).find("link") != std::string::npos)
      {
        // Before adding inertial elements, make sure there is none and the link has collision element
        if (doc_element->FirstChildElement("inertial") == nullptr &&
            doc_element->FirstChildElement("collision") != nullptr)
        {
          new_urdf_needed = true;
          TiXmlElement inertia_link("inertial");
          TiXmlElement mass("mass");
          TiXmlElement inertia_joint("inertia");

          mass.SetAttribute("value", "0.1");

          inertia_joint.SetAttribute("ixx", "0.03");
          inertia_joint.SetAttribute("iyy", "0.03");
          inertia_joint.SetAttribute("izz", "0.03");
          inertia_joint.SetAttribute("ixy", "0.0");
          inertia_joint.SetAttribute("ixz", "0.0");
          inertia_joint.SetAttribute("iyz", "0.0");

          inertia_link.InsertEndChild(mass);
          inertia_link.InsertEndChild(inertia_joint);

          doc_element->InsertEndChild(inertia_link);
        }
      }
      else if (static_cast<std::string>(doc_element->Value()).find("joint") != std::string::npos)
      {
        // Before adding a transmission element, make sure there the joint is not fixed
        if (static_cast<std::string>(doc_element->Attribute("type")) != "fixed")
        {
          new_urdf_needed = true;
          std::string joint_name = static_cast<std::string>(doc_element->Attribute("name"));
          TiXmlElement transmission("transmission");
          TiXmlElement type("type");
          TiXmlElement joint("joint");
          TiXmlElement hardware_interface("hardwareInterface");
          TiXmlElement actuator("actuator");
          TiXmlElement mechanical_reduction("mechanicalReduction");

          transmission.SetAttribute("name", std::string("trans_") + joint_name);
          joint.SetAttribute("name", joint_name);
          actuator.SetAttribute("name", joint_name + std::string("_motor"));

          type.InsertEndChild(TiXmlText("transmission_interface/SimpleTransmission"));
          transmission.InsertEndChild(type);

          hardware_interface.InsertEndChild(TiXmlText(getJointHardwareInterface(joint_name).c_str()));
          joint.InsertEndChild(hardware_interface);
          transmission.InsertEndChild(joint);

          mechanical_reduction.InsertEndChild(TiXmlText("1"));
          actuator.InsertEndChild(hardware_interface);
          actuator.InsertEndChild(mechanical_reduction);
          transmission.InsertEndChild(actuator);

          urdf_document.RootElement()->InsertEndChild(transmission);
        }
      }
    }

    // Add gazebo_ros_control plugin which reads the transmission tags
    TiXmlElement gazebo("gazebo");
    TiXmlElement plugin("plugin");
    TiXmlElement robot_namespace("robotNamespace");

    plugin.SetAttribute("name", "gazebo_ros_control");
    plugin.SetAttribute("filename", "libgazebo_ros_control.so");
    robot_namespace.InsertEndChild(TiXmlText(std::string("/")));

    plugin.InsertEndChild(robot_namespace);
    gazebo.InsertEndChild(plugin);

    urdf_document.RootElement()->InsertEndChild(gazebo);
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    RCLCPP_ERROR_STREAM(*logger_, e.what());
    return std::string("");
  }

  if (new_urdf_needed)
  {
    urdf_document.Accept(&printer);
    return std::string(printer.CStr());
  }

  return std::string("");
}

}  // namespace moveit_setup_simulation
