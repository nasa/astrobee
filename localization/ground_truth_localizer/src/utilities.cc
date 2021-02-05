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

#include <ground_truth_localizer/utilities.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

namespace ground_truth_localizer {
namespace lc = localization_common;
namespace mc = msg_conversions;

Eigen::Isometry3d PoseFromMsg(const geometry_msgs::PoseStamped& pose_msg) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = mc::VectorFromMsg<Eigen::Vector3d, geometry_msgs::Point>(pose_msg.pose.position);
  pose.linear() =
    mc::RotationFromMsg<Eigen::Quaterniond, geometry_msgs::Quaternion>(pose_msg.pose.orientation).toRotationMatrix();
  return pose;
}

Twist TwistFromMsg(const geometry_msgs::TwistStamped& twist_msg) {
  Twist twist;
  twist.linear_velocity = mc::VectorFromMsg<Eigen::Vector3d, geometry_msgs::Vector3>(twist_msg.twist.linear);
  twist.angular_velocity = mc::VectorFromMsg<Eigen::Vector3d, geometry_msgs::Vector3>(twist_msg.twist.angular);
  return twist;
}

ff_msgs::EkfState LocStateMsg(const Eigen::Isometry3d& pose, const Twist& twist,
                              const localization_common::Time& timestamp) {
  ff_msgs::EkfState loc_msg;
  mc::EigenPoseToMsg(pose, loc_msg.pose);
  mc::VectorToMsg(twist.linear_velocity, loc_msg.velocity);
  mc::VectorToMsg(twist.angular_velocity, loc_msg.omega);
  lc::TimeToHeader(timestamp, loc_msg.header);
  loc_msg.confidence = lc::Confidence::kGood;
  return loc_msg;
}
}  // namespace ground_truth_localizer
