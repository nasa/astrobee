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

#ifndef DOCKING_DOCKER_H_
#define DOCKING_DOCKER_H_

#include <stdint.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <config_reader/config_reader.h>

#include <ff_msgs/DockAction.h>
#include <ff_msgs/UndockAction.h>
#include <ff_msgs/EkfState.h>
#include <ff_hw_msgs/DockStateStamped.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <condition_variable>  // NOLINT
#include <vector>
#include <mutex>

namespace docking {

struct Bay {
  ::geometry_msgs::Pose start;
  ::geometry_msgs::Pose end;
};

struct Limits {
  ::geometry_msgs::Twist velocity;
  ::geometry_msgs::Twist acceleration;
  float speed;
};

class Docker {
 public:
  Docker();

  bool Valid() const noexcept { return valid_; }

  void OnEkfState(const ff_msgs::EkfState &ekf);
  void OnDockState(const ff_hw_msgs::DockStateStamped &state);

  void ExecDock(ff_msgs::DockGoal::ConstPtr const& goal);
  void ExecUndock(ff_msgs::UndockGoal::ConstPtr const& goal);

  static constexpr size_t kFeatureCountBufferSize = 10;

 protected:
  bool EnoughFeatures(uint32_t minimum);

  bool ReadConfig();

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<ff_msgs::DockAction> sas_dock_;
  actionlib::SimpleActionServer<ff_msgs::UndockAction> sas_undock_;

  ros::Subscriber sub_dock_;
  ros::Subscriber sub_ekf_;

  bool valid_;
  ::config_reader::ConfigReader config_;

  uint8_t nb_features_;
  uint8_t ekf_confidence_;
  std::vector<uint8_t> features_count_;
  uint32_t localization_updates_;

  std::mutex pose_mutex_;
  bool pose_valid_ = false;
  geometry_msgs::Pose pose_;

  // dealing with determining docked status
  std::mutex docked_mutex_;
  std::condition_variable docked_cv_;
  bool docked_valid_;
  bool docked_;

  bool have_goal_;

  // config params
  float p_cam_timeout_;
  float p_ekf_timeout_;
  float p_pos_timeout_;
  float p_pmc_timeout_;
  float p_mob_timeout_;

  Limits ingress_, egress_;

  // position of the docking bays
  Bay p_bays_[2];
};

typedef std::unique_ptr<Docker> DockerPtr;

}  // namespace docking

#endif  // DOCKING_DOCKER_H_
