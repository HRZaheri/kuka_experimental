/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/*
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */

#ifndef KUKA_RSI_HW_INTERFACE_RSI_COMMAND_
#define KUKA_RSI_HW_INTERFACE_RSI_COMMAND_

#include <tinyxml.h>
#include <vector>

namespace kuka_rsi_hw_interface
{

class RSICommand
{
public:
  RSICommand();
  RSICommand(std::vector<double> cartesian_correction, std::vector<double> position_corrections, unsigned long long ipoc);
  std::string xml_doc;
};

RSICommand::RSICommand()
{
  // Intentionally empty
}

RSICommand::RSICommand(std::vector<double> cartesian_coordinate_correction,
                       std::vector<double> joint_position_correction,
                       unsigned long long ipoc)
{
  TiXmlDocument doc;
  TiXmlElement* root = new TiXmlElement("Sen");
  root->SetAttribute("Type", "ImFree");

  TiXmlElement* el = new TiXmlElement("DEF_EStr");
  el->LinkEndChild(new TiXmlText("Free config!"));
  root->LinkEndChild(el);

  // Add string attribute
  el = new TiXmlElement("RKorr");
  el->SetAttribute("X", std::to_string(cartesian_coordinate_correction[0]));
  el->SetAttribute("Y", std::to_string(cartesian_coordinate_correction[1]));
  el->SetAttribute("Z", std::to_string(cartesian_coordinate_correction[2]));
  el->SetAttribute("A", std::to_string(cartesian_coordinate_correction[3]));
  el->SetAttribute("B", std::to_string(cartesian_coordinate_correction[4]));
  el->SetAttribute("C", std::to_string(cartesian_coordinate_correction[5]));
  root->LinkEndChild(el);

  // Add string attribute
  el = new TiXmlElement("AKorr");
  el->SetAttribute("A1", std::to_string(joint_position_correction[0]));
  el->SetAttribute("A2", std::to_string(joint_position_correction[1]));
  el->SetAttribute("A3", std::to_string(joint_position_correction[2]));
  el->SetAttribute("A4", std::to_string(joint_position_correction[3]));
  el->SetAttribute("A5", std::to_string(joint_position_correction[4]));
  el->SetAttribute("A6", std::to_string(joint_position_correction[5]));
  root->LinkEndChild(el);

  // Add string attribute
  el = new TiXmlElement("EKorr");
  el->SetAttribute("E1", std::to_string(0.0));
  el->SetAttribute("E2", std::to_string(0.0));
  el->SetAttribute("E3", std::to_string(0.0));
  el->SetAttribute("E4", std::to_string(0.0));
  el->SetAttribute("E5", std::to_string(0.0));
  el->SetAttribute("E6", std::to_string(0.0));
  root->LinkEndChild(el);

  el = new TiXmlElement("Tech");
  el->SetAttribute("T21",  std::to_string(0.0));
  el->SetAttribute("T22",  std::to_string(0.0));
  el->SetAttribute("T23",  std::to_string(0.0));
  el->SetAttribute("T24",  std::to_string(0.0));
  el->SetAttribute("T25",  std::to_string(0.0));
  el->SetAttribute("T26",  std::to_string(0.0));
  el->SetAttribute("T27",  std::to_string(0.0));
  el->SetAttribute("T28",  std::to_string(0.0));
  el->SetAttribute("T29",  std::to_string(0.0));
  el->SetAttribute("T210", std::to_string(0.0));
  root->LinkEndChild(el);

  el = new TiXmlElement("DiO");
  el->LinkEndChild(new TiXmlText(std::to_string(125)));
  root->LinkEndChild(el);

  el = new TiXmlElement("IPOC");
  el->LinkEndChild(new TiXmlText(std::to_string(ipoc)));
  root->LinkEndChild(el);

  doc.LinkEndChild(root);
  TiXmlPrinter printer;
  printer.SetStreamPrinting();
  doc.Accept(&printer);

  xml_doc = printer.Str();
}

} // namespace kuka_rsi_hw_interface

#endif
