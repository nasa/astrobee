/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef TRAJ_OPT_ROS_ROS_BRIDGE_H_
#define TRAJ_OPT_ROS_ROS_BRIDGE_H_

// Standard includes
#include <ff_common/ff_ros.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>

#include <traj_opt_basic/traj_data.h>
#include <traj_opt_msgs/msg/polynomial.hpp>
#include <traj_opt_msgs/msg/spline.hpp>
#include <traj_opt_msgs/msg/trajectory.hpp>
#include <traj_opt_msgs/msg/solver_info.hpp>
#include <string>
#include <map>

namespace traj_opt_msgs {
  typedef msg::Polynomial Polynomial;
  typedef msg::Spline Spline;
  typedef msg::Trajectory Trajectory;
  typedef msg::SolverInfo SolverInfo;
} // namespace traj_opt_msgs


class TrajRosBridge {
 public:
  // No need to instantiate pesky variables!
  static traj_opt_msgs::Trajectory convert(const traj_opt::TrajData &data);
  static traj_opt::TrajData convert(const traj_opt_msgs::Trajectory &msg);
  static traj_opt_msgs::SolverInfo convert(const traj_opt::SolverInfo &data);

  // make sure to run ros::init() before calling these functions or they won't work
  static void publish_msg(const traj_opt_msgs::Trajectory &msg,
                          std::string frame_id = "map",
                          std::string topic = "trajectory");
  static void publish_msg(const traj_opt::TrajData &data,
                          std::string frame_id = "map",
                          std::string topic = "trajectory");
  static void publish_msg(const traj_opt::SolverInfo &data,
                          std::string topic = "solver_info");
  static void publish_msg(const traj_opt_msgs::SolverInfo &msg,
                          std::string topic = "solver_info");

  static bool are_subscribers(std::string topic);

  // Use global singleton paradigm.  All these things are private!
  // Keep your government out of my constructors!
 private:
  TrajRosBridge();
  static TrajRosBridge &instance();

  static rclcpp::Publisher<traj_opt_msgs::Trajectory>::SharedPtr getPub(std::string topic);
  static rclcpp::Publisher<traj_opt_msgs::SolverInfo>::SharedPtr getInfoPub(std::string topic);

  NodeHandle nh_traj_;
  NodeHandle nh_info_;
  std::map<std::string, rclcpp::Publisher<traj_opt_msgs::Trajectory>::SharedPtr> pubs_traj_;
  std::map<std::string, rclcpp::Publisher<traj_opt_msgs::SolverInfo>::SharedPtr> pubs_info_;

  // static rclcpp::PublisherBase::SharedPtr getPub(std::string topic);
  // static rclcpp::PublisherBase::SharedPtr getInfoPub(std::string topic);

  // static rclcpp::PublisherBase::SharedPtr getPub(std::string topic);
  // static rclcpp::PublisherBase::SharedPtr getInfoPub(std::string topic);

  // // rclcpp::node::Node::make_shared nh_("~");
  // NodeHandle nh_;
  // std::map<std::string, rclcpp::PublisherBase::SharedPtr> pubs_;
};

#endif  // TRAJ_OPT_ROS_ROS_BRIDGE_H_
