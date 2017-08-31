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
#include <traj_opt_msgs/Polynomial.h>
#include <traj_opt_msgs/Spline.h>
#include <traj_opt_msgs/Trajectory.h>
#include <string>
#include <map>

class TrajRosBridge {
 public:
  // No need to instantiate pesky variables!
  static traj_opt_msgs::Trajectory convert(const traj_opt::TrajData &data);
  static traj_opt::TrajData convert(const traj_opt_msgs::Trajectory &msg);

  // make sure to run ros::init() before calling this function or it won't work
  static void publish_msg(const traj_opt_msgs::Trajectory &msg,
                          std::string frame_id = "map",
                          std::string topic = "trajectory");
  static void publish_msg(const traj_opt::TrajData &data,
                          std::string frame_id = "map",
                          std::string topic = "trajectory");

  // Use global singleton paradignm.  All these things are private!
  // Keep your government out of my contructors!
 private:
  TrajRosBridge();
  static TrajRosBridge &instance();
  static ros::Publisher getPub(std::string topic);
  ros::NodeHandle nh_;
  std::map<std::string, ros::Publisher> pubs_;
};

#endif  // TRAJ_OPT_ROS_ROS_BRIDGE_H_
