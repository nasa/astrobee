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
#ifndef VISION_COMMON_TEST_UTILITIES_H_
#define VISION_COMMON_TEST_UTILITIES_H_

#include <ff_common/eigen_vectors.h>
#include <localization_common/image_correspondences.h>
#include <optimization_common/optimization_params.h>
#include <vision_common/lk_optical_flow_feature_detector_and_matcher_params.h>
#include <vision_common/pose_estimation.h>
#include <vision_common/ransac_pnp_params.h>
#include <vision_common/reprojection_pose_estimate_params.h>

#include <opencv2/core.hpp>

#include <vector>

namespace vision_common {
void SetFocalLengths(const Eigen::Vector2d& focal_lengths, Eigen::Matrix3d& intrinsics);

void SetPrincipalPoints(const Eigen::Vector2d& principal_points, Eigen::Matrix3d& intrinsics);

LKOpticalFlowFeatureDetectorAndMatcherParams DefaultLKOpticalFlowFeatureDetectorAndMatcherParams();

cv::Mat MarkerImage(const int row_spacing, const int col_spacing, int& num_markers_added,
                    const cv::Point2i& offset = cv::Point2i(0, 0));

int AddMarkers(const int row_spacing, const int col_spacing, cv::Mat& image,
               const cv::Point2i& offset = cv::Point2i(0, 0));

optimization_common::OptimizationParams DefaultOptimizationParams();

RansacPnPParams DefaultRansacPnPParams();

ReprojectionPoseEstimateParams DefaultReprojectionPoseEstimateParams();

Eigen::VectorXd RandomFovDistortion();

Eigen::VectorXd RandomRadDistortion();

Eigen::VectorXd RandomRadTanDistortion();

// Spaced out poses for targets which when projected into image space cover
// the image well with target points.  Poses are sampled for each row/col combination
// and evenly spaced in cylindrical coordinates
std::vector<Eigen::Isometry3d> EvenlySpacedTargetPoses(const int num_rows = 3, const int num_cols = 5,
                                                       const int num_y_levels = 2);

std::vector<Eigen::Vector3d> TargetPoints(const int points_per_row, const int points_per_col,
                                          const double row_spacing = 0.1, const double col_spacing = 0.1);

template <typename DISTORTER>
class RegistrationCorrespondences {
 public:
  RegistrationCorrespondences(const Eigen::Isometry3d& camera_T_target, const Eigen::Matrix3d& intrinsics,
                              const std::vector<Eigen::Vector3d>& target_t_target_point,
                              const Eigen::VectorXd& distortion = Eigen::VectorXd());

  const localization_common::ImageCorrespondences& correspondences() const { return correspondences_; }

  const Eigen::Isometry3d& camera_T_target() const { return camera_T_target_; }

  const Eigen::Matrix3d& intrinsics() const { return intrinsics_; }

 private:
  localization_common::ImageCorrespondences correspondences_;
  Eigen::Isometry3d camera_T_target_;
  Eigen::Matrix3d intrinsics_;
};

template <typename DISTORTER>
RegistrationCorrespondences<DISTORTER>::RegistrationCorrespondences(
  const Eigen::Isometry3d& camera_T_target, const Eigen::Matrix3d& intrinsics,
  const std::vector<Eigen::Vector3d>& target_t_target_points, const Eigen::VectorXd& distortion)
    : camera_T_target_(camera_T_target), intrinsics_(intrinsics) {
  for (const auto& target_t_target_point : target_t_target_points) {
    const Eigen::Vector3d camera_t_target_point = camera_T_target_ * target_t_target_point;
    if (camera_t_target_point.z() <= 0) continue;
    const Eigen::Vector2d image_point =
      ProjectWithDistortion<DISTORTER>(camera_t_target_point, intrinsics_, distortion);
    correspondences_.AddCorrespondence(image_point, target_t_target_point);
  }
}
}  // namespace vision_common
#endif  // VISION_COMMON_TEST_UTILITIES_H_
