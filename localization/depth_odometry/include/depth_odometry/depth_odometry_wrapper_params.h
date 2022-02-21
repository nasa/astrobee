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
#ifndef DEPTH_ODOMETRY_DEPTH_ODOMETRY_WRAPPER_PARAMS_H_
#define DEPTH_ODOMETRY_DEPTH_ODOMETRY_WRAPPER_PARAMS_H_

#include <depth_odometry/point_to_plane_icp_depth_odometry_params.h>
#include <depth_odometry/image_features_with_known_correspondences_aligner_depth_odometry_params.h>

#include <Eigen/Geometry>

#include <string>

namespace depth_odometry {
struct DepthOdometryWrapperParams {
  double max_image_and_point_cloud_time_diff;
  // icp or image_feature
  std::string method;
  Eigen::Isometry3d body_T_haz_cam;
  Eigen::Affine3d haz_cam_A_haz_depth;
  PointToPlaneICPDepthOdometryParams icp;
  ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams image_features;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_ODOMETRY_WRAPPER_PARAMS_H_
