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
#ifndef DEPTH_ODOMETRY_POINT_TO_PLANE_ICP_DEPTH_ODOMETRY_PARAMS_H_
#define DEPTH_ODOMETRY_POINT_TO_PLANE_ICP_DEPTH_ODOMETRY_PARAMS_H_

#include <depth_odometry/depth_odometry_params.h>
#include <point_cloud_common/point_to_plane_icp_params.h>

namespace depth_odometry {
struct PointToPlaneICPDepthOdometryParams : public DepthOdometryParams {
  point_cloud_common::PointToPlaneICPParams icp;
  bool downsample;
  double downsample_leaf_size;
  // Organized options
  bool use_organized_normal_estimation;
  // Organized normal estimation
  double max_depth_change_factor;
  double normal_smoothing_size;
  Eigen::Matrix3d intrinsics_matrix;
  // Normal space sampling
  bool use_normal_space_sampling;
  int bins_per_axis;
  int num_samples;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_POINT_TO_PLANE_ICP_DEPTH_ODOMETRY_PARAMS_H_
