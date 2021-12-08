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

#include <depth_odometry/pose_with_covariance_and_correspondences.h>
#include <depth_odometry/utilities.h>
#include <localization_common/utilities.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

ff_msgs::DepthOdometry DepthOdometryMsg(const PoseWithCovarianceAndCorrespondences& sensor_F_source_T_target,
                                        const lc::PoseWithCovariance& body_F_source_T_target, const double runtime) {
  ff_msgs::DepthOdometry depth_odometry_msg;
  depth_odometry_msg.odometry = OdometryMsg(sensor_F_source_T_target.pose_with_covariance, body_F_source_T_target);
  depth_odometry_msg.correspondences = CorrespondencesMsg(sensor_F_source_T_target.depth_correspondences);
  depth_odometry_msg.valid_image_points = sensor_F_source_T_target.depth_correspondences.valid_image_points;
  depth_odometry_msg.valid_points_3d = sensor_F_source_T_target.depth_correspondences.valid_3d_points;
  lc::TimeToHeader(sensor_F_source_T_target.target_time, depth_odometry_msg.header);
  lc::TimeToMsg(sensor_F_source_T_target.source_time, depth_odometry_msg.odometry.source_time);
  lc::TimeToMsg(sensor_F_source_T_target.target_time, depth_odometry_msg.odometry.target_time);
  depth_odometry_msg.runtime = runtime;
  return depth_odometry_msg;
}

ff_msgs::Odometry OdometryMsg(const lc::PoseWithCovariance& sensor_F_source_T_target,
                              const lc::PoseWithCovariance& body_F_source_T_target) {
  ff_msgs::Odometry odometry_msg;
  mc::EigenPoseCovarianceToMsg(sensor_F_source_T_target.pose, sensor_F_source_T_target.covariance,
                               odometry_msg.sensor_F_source_T_target);
  mc::EigenPoseCovarianceToMsg(body_F_source_T_target.pose, body_F_source_T_target.covariance,
                               odometry_msg.body_F_source_T_target);
  return odometry_msg;
}

std::vector<ff_msgs::DepthCorrespondence> CorrespondencesMsg(const lm::DepthCorrespondences& depth_correspondences) {
  std::vector<ff_msgs::DepthCorrespondence> correspondences_msg;
  int num_points = 0;
  if (depth_correspondences.valid_3d_points)
    num_points = depth_correspondences.source_3d_points.size();
  else if (depth_correspondences.valid_image_points)
    num_points = depth_correspondences.source_image_points.size();
  for (int i = 0; i < num_points; ++i) {
    ff_msgs::DepthCorrespondence correspondence;
    if (depth_correspondences.valid_3d_points) {
      mc::VectorToMsg(depth_correspondences.source_3d_points[i], correspondence.source_3d_point);
      mc::VectorToMsg(depth_correspondences.target_3d_points[i], correspondence.target_3d_point);
    }
    if (depth_correspondences.valid_image_points) {
      mc::Vector2dToMsg(depth_correspondences.source_image_points[i], correspondence.source_image_point);
      mc::Vector2dToMsg(depth_correspondences.target_image_points[i], correspondence.target_image_point);
    }
    correspondences_msg.emplace_back(correspondence);
  }
  return correspondences_msg;
}
}  // namespace depth_odometry
