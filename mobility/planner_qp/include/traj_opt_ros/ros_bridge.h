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

#include <ros/ros.h>
#include <traj_opt_basic/traj_data.h>
#include <planner_qp/Polynomial.h>
#include <planner_qp/Spline.h>
#include <planner_qp/Trajectory.h>
#include <planner_qp/SolverInfo.h>
#include <string>
#include <map>

class TrajRosBridge {
 public:
  // No need to instantiate pesky variables!
  static planner_qp::Trajectory convert(const traj_opt::TrajData &data);
  static traj_opt::TrajData convert(const planner_qp::Trajectory &msg);
  static planner_qp::SolverInfo convert(const traj_opt::SolverInfo &data);

  // make sure to run ros::init() before calling these functions or they won't work
  static void publish_msg(const planner_qp::Trajectory &msg,
                          std::string frame_id = "map",
                          std::string topic = "trajectory");
  static void publish_msg(const traj_opt::TrajData &data,
                          std::string frame_id = "map",
                          std::string topic = "trajectory");
  static void publish_msg(const planner_qp::SolverInfo &data,
                          std::string topic = "solver_info");
  static void publish_msg(const traj_opt::SolverInfo &msg,
                          std::string topic = "solver_info");

  static bool are_subscribers(std::string topic);

  // Use global singleton paradignm.  All these things are private!
  // Keep your government out of my contructors!
 private:
  TrajRosBridge();
  static TrajRosBridge &instance();
  static ros::Publisher getPub(std::string topic);
  static ros::Publisher getInfoPub(std::string topic);
  ros::NodeHandle nh_;
  std::map<std::string, ros::Publisher> pubs_;
};

#endif  // TRAJ_OPT_ROS_ROS_BRIDGE_H_
