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
#ifndef DEPTH_ODOMETRY_IMAGE_FEATURES_WITH_KNOWN_CORRESPONDENCES_ALIGNER_DEPTH_ODOMETRY_PARAMS_H_
#define DEPTH_ODOMETRY_IMAGE_FEATURES_WITH_KNOWN_CORRESPONDENCES_ALIGNER_DEPTH_ODOMETRY_PARAMS_H_

#include <depth_odometry/depth_odometry_params.h>
#include <depth_odometry/point_to_plane_icp_depth_odometry_params.h>
#include <point_cloud_common/point_cloud_with_known_correspondences_aligner_params.h>
#include <vision_common/brisk_feature_detector_and_matcher_params.h>
#include <vision_common/lk_optical_flow_feature_detector_and_matcher_params.h>
#include <vision_common/surf_feature_detector_and_matcher_params.h>

#include <string>

namespace depth_odometry {
struct ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams : public DepthOdometryParams {
  point_cloud_common::PointCloudWithKnownCorrespondencesAlignerParams aligner;
  vision_common::BriskFeatureDetectorAndMatcherParams brisk_feature_detector_and_matcher;
  vision_common::LKOpticalFlowFeatureDetectorAndMatcherParams lk_optical_flow_feature_detector_and_matcher;
  vision_common::SurfFeatureDetectorAndMatcherParams surf_feature_detector_and_matcher;
  std::string detector;
  // CLAHE params
  bool use_clahe;
  int clahe_grid_length;
  double clahe_clip_limit;
  // Other
  double min_x_distance_to_border;
  double min_y_distance_to_border;
  int min_num_inliers;
  bool refine_estimate;
  PointToPlaneICPDepthOdometryParams point_to_plane_icp;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_IMAGE_FEATURES_WITH_KNOWN_CORRESPONDENCES_ALIGNER_DEPTH_ODOMETRY_PARAMS_H_
