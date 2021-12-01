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

#include <depth_odometry/utilities.h>

namespace depth_odometry {
ff_msgs::DepthOdometry DepthOdometryMsg(
  const PoseWithCovarianceAndCorrespondences& pose_with_covariance_and_correspondences,
  const PoseWithCovariance& body_frame_pose_with_covariance) {
  ff_msgs::DepthOdometry depth_odometry_msg;
  depth_odometry_msg.odometry = OdometryMsg(pose_with_covariance_and_correspondences.pose_with_covariance);
  depth_odometry_msg.correspondences =
    CorrespondencesMsg(pose_with_covariance_and_correspondences.depth_correspondences);
  depth_odometry_msg.valid_image_points =
    pose_with_covariance_and_correspondences.depth_correspondences.valid_image_points;
  depth_odometry_msg.valid_3d_points = pose_with_covariance_and_correspondences.depth_correspondences.valid_3d_points;
  lc::TimeToHeader(pose_with_covariance_and_correspondences.target_time, depth_odometry_msg.header);
  lc::TimeToMsg(pose_with_covariance_and_correspondences.source_time, depth_odometry_msg.odometry.time_a);
  lc::TimeToMsg(pose_with_covariance_and_correspondences.target_time, depth_odometry_msg.odometry.time_b);
  return depth_odometry_msg;
}

ff_msgs::Odometry OdometryMsg(const PoseWithCovariance& sensor_frame_pose_with_covariance,
                              const PoseWithCovariance& body_frame_pose_with_covariance) {
  ff_msgs::Odometry odometry_msg;
  mc::EigenPoseCovarianceToMsg(sensor_frame_pose_with_covariance.pose, sensor_frame_pose_with_covariance.covariance,
                               odometry_msg.sensor_F_a_T_b);
  mc::EigenPoseCovarianceToMsg(body_frame_pose_with_covariance.pose, body_frame_pose_with_covariance.covariance,
                               odometry_msg.body_F_a_T_b);
  return odometry_msg;
}

std::vector<ff_msgs::DepthCorrespondence> CorrespondencesMsg(const DepthCorrespondences& depth_correspondences) {
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