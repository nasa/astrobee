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
#ifndef SIMULATION_LOCALIZER_SIMULATION_LOCALIZER_NODELET_H_
#define SIMULATION_LOCALIZER_SIMULATION_LOCALIZER_NODELET_H_

#include <ff_msgs/SetEkfInput.h>
#include <ff_util/ff_nodelet.h>
#include <localization_common/time.h>
#include <simulation_localizer/twist.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <std_srvs/Empty.h>

#include <boost/optional.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace simulation_localizer {
class SimulationLocalizerNodelet : public ff_util::FreeFlyerNodelet {
 public:
  SimulationLocalizerNodelet();

 private:
  void Initialize(ros::NodeHandle* nh);

  void SubscribeAndAdvertise(ros::NodeHandle* nh);

  bool SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res);

  void PoseCallback(geometry_msgs::PoseStamped::ConstPtr const& pose);

  void TwistCallback(geometry_msgs::TwistStamped::ConstPtr const& twist);

  void PublishLocState(const localization_common::Time& timestamp);

  boost::optional<Eigen::Isometry3d> pose_;
  boost::optional<Twist> twist_;
  int input_mode_ = ff_msgs::SetEkfInputRequest::MODE_TRUTH;
  ros::Subscriber pose_sub_, twist_sub_;
  ros::Publisher state_pub_, pose_pub_, twist_pub_;
  ros::ServiceServer input_mode_srv_;
};
}  // namespace simulation_localizer

#endif  // SIMULATION_LOCALIZER_SIMULATION_LOCALIZER_NODELET_H_
