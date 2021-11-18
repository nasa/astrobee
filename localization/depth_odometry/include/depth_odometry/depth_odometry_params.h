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
#ifndef DEPTH_ODOMETRY_DEPTH_ODOMETRY_PARAMS_H_
#define DEPTH_ODOMETRY_DEPTH_ODOMETRY_PARAMS_H_

#include <depth_odometry/depth_image_aligner_params.h>
#include <point_cloud_common/icp_params.h>

#include <Eigen/Geometry>

namespace depth_odometry {
struct DepthOdometryParams {
  bool depth_point_cloud_registration_enabled;
  bool depth_image_registration_enabled;
  DepthImageAlignerParams depth_image_aligner;
  point_cloud_common::ICPParams icp;
  double max_time_diff;
  double max_image_and_point_cloud_time_diff;
  double position_covariance_threshold;
  double orientation_covariance_threshold;
  bool inital_estimate_with_ransac_ia;
  Eigen::Isometry3d body_T_haz_cam;
  Eigen::Affine3d haz_cam_A_haz_depth;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_ODOMETRY_PARAMS_H_
