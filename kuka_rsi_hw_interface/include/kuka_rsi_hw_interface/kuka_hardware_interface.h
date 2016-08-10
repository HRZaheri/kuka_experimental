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
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
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
 * Author: Lars Tingelstad
 */

#ifndef KUKA_RSI_HARDWARE_INTERFACE_KUKA_HARDWARE_INTERFACE_
#define KUKA_RSI_HARDWARE_INTERFACE_KUKA_HARDWARE_INTERFACE_

// STL
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>


// ros_control
#include <realtime_tools/realtime_publisher.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// Timers
#include <chrono>

//SERVER
//#include <kuka_rsi_hw_interface/udp_server.h>
#include <kuka_rsi_hw_interface/tcp_server.h>

// RSI
#include <kuka_rsi_hw_interface/rsi_state.h>
#include <kuka_rsi_hw_interface/rsi_command.h>

//redis subscriber
#include "kuka_rsi_hw_interface/teleopMsg.h"

//transformations
#include "kuka_rsi_hw_interface/transformation.h"

namespace kuka_rsi_hw_interface
{

class KukaHardwareInterface : public hardware_interface::RobotHW
{

private:

  // ROS node handle
  ros::NodeHandle nh_;

  //remote control
  Eigen::Matrix4f tool2world;
  ros::Subscriber teleop_subscriber;
  std::vector<float> pose_current;
  std::vector<float> pose_old;
  void get_pose(const kuka_rsi_hw_interface::teleopMsg::ConstPtr &msg);
  bool robot_init = false;

  float sensitivity = 200.0f;
  float scaleFactor = 200.0f;
  float stepSize = 0.8f; //mm/step
  float stepSize_rot = 0.2f; //deg/step

  /******************/
  //PID
  float err_I = 0.0f;
  float KP = 3.0f;
  float KI = 1.2f;
  /******************/

  unsigned int n_dof_;

  std::vector<std::string> joint_names_;

  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  std::vector<double> cartesian_coordinate_;

  // RSI
  RSIState rsi_state_;
  RSICommand rsi_command_;

  std::vector<double> rsi_initial_joint_positions_;
  std::vector<double> rsi_joint_position_corrections_;
  std::vector<double> rsi_initial_cartesian_coordinate_;
  std::vector<double> rsi_target_cartesian_coordinate_;
  std::vector<double> rsi_cartesian_coordinate_corrections_;

  unsigned long long ipoc_;

  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::String> > rt_rsi_pub_;

  std::unique_ptr<TCPServer> server_;
  std::string local_host_;
  int local_port_;
  std::string remote_host_;
  std::string remote_port_;
  std::string in_buffer_;
  std::string out_buffer_;

  // Timing
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;

  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

public:

  KukaHardwareInterface();
  ~KukaHardwareInterface();

  void start();
  void configure();
  bool read(const ros::Time time, const ros::Duration period);
  bool write(const ros::Time time, const ros::Duration period);
};

} // namespace kuka_rsi_hw_interface

#endif
