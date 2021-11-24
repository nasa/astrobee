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
#ifndef DEPTH_ODOMETRY_UTILITIES_H_
#define DEPTH_ODOMETRY_UTILITIES_H_

#include <depth_odometry/depth_correspondences.h>
#include <depth_odometry/pose_with_covariance_and_correspondences.h>
#include <ff_msgs/DepthCorrespondence.h>
#include <ff_msgs/DepthOdometry.h>
#include <ff_msgs/Odometry.h>
#include <localization_common/pose_with_covariance.h>
#include <localization_common/time.h>

#include <vector>

namespace depth_odometry {
ff_msgs::DepthOdometry DepthOdometryMsg(
  const PoseWithCovarianceAndCorrespondences& pose_with_covariance_and_correspondences,
  const localization_common::PoseWithCovariance& body_frame_pose_with_covariance);
ff_msgs::Odometry OdometryMsg(const localization_common::PoseWithCovariance& sensor_frame_pose_with_covariance,
                              const localization_common::PoseWithCovariance& body_frame_pose_with_covariance);
std::vector<ff_msgs::DepthCorrespondence> CorrespondencesMsg(const DepthCorrespondences& depth_correspondences);
}  // namespace depth_odometry
#endif  // DEPTH_ODOMETRY_UTILITIES_H_
