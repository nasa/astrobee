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

#include <camera/camera_params.h>
#include <camera/camera_model.h>
#include <depth_odometry/brisk_feature_detector_and_matcher.h>
#include <depth_odometry/depth_image_aligner.h>
#include <depth_odometry/lk_optical_flow_feature_detector_and_matcher.h>
#include <depth_odometry/point_cloud_utilities.h>
#include <depth_odometry/surf_feature_detector_and_matcher.h>
#include <localization_common/logger.h>
#include <localization_common/timer.h>
#include <sparse_mapping/reprojection.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;

DepthImageAligner::DepthImageAligner(const DepthImageAlignerParams& params)
    : params_(params), cam_(*(params_.camera_params)) {
  if (params_.detector == "brisk") {
    feature_detector_and_matcher_.reset(new BriskFeatureDetectorAndMatcher(params_.brisk_feature_detector_and_matcher));
  } else if (params_.detector == "lk_optical_flow") {
    feature_detector_and_matcher_.reset(
      new LKOpticalFlowFeatureDetectorAndMatcher(params_.lk_optical_flow_feature_detector_and_matcher));
  } else if (params_.detector == "surf") {
    feature_detector_and_matcher_.reset(new SurfFeatureDetectorAndMatcher(params_.surf_feature_detector_and_matcher));
  } else {
    LogFatal("DepthImageAligner: Invalid feature detector and matcher.");
    std::exit(1);
  }
  clahe_ = cv::createCLAHE(params_.clahe_clip_limit, cv::Size(params_.clahe_grid_length, params_.clahe_grid_length));
}

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>>
DepthImageAligner::ComputeRelativeTransform() {
  if (!previous_feature_depth_image_ || !latest_feature_depth_image_) return boost::none;
  auto matches = feature_detector_and_matcher_->Match(*previous_feature_depth_image_, *latest_feature_depth_image_);

  std::vector<Eigen::Vector3d> landmarks;
  std::vector<Eigen::Vector2d> observations;
  // Get 3D points for image features
  for (auto match_it = matches.begin(); match_it != matches.end();) {
    const auto& previous_image_point = match_it->point_a;
    const auto& latest_image_point = match_it->point_b;
    const auto& latest_point_3d =
      latest_feature_depth_image_->InterpolatePoint3D(latest_image_point.x(), latest_image_point.y());
    if (!latest_point_3d || !ValidPoint(*latest_point_3d)) {
      match_it = matches.erase(match_it);
      continue;
    }
    const Eigen::Vector3d landmark(latest_point_3d->x, latest_point_3d->y, latest_point_3d->z);
    landmarks.emplace_back(landmark);
    // RansacEstimateCamera expects image points in undistorted centered frame
    Eigen::Vector2d undistorted_previous_image_point;
    params_.camera_params->Convert<camera::DISTORTED, camera::UNDISTORTED_C>(previous_image_point,
                                                                             &undistorted_previous_image_point);
    observations.emplace_back(undistorted_previous_image_point);
    ++match_it;
    // LogError("3d point: " << latest_point_3d->x << ", " << latest_point_3d->y << ", " << latest_point_3d->z);
    // LogError("image point: " << previous_image_point.x << ", " << previous_image_point.y);
  }
  LogError("pnp points: " << landmarks.size());
  if (landmarks.size() < 4) {
    LogError("ComputeRelativeTransform: Too few points for Ransac PnP, need 4 but given " << landmarks.size() << ".");
    return boost::none;
  }

  LogError("filtered matches: " << matches.size());
  LogError("landmarks: " << landmarks.size() << ", observations: " << observations.size());
  std::vector<Eigen::Vector3d> inlier_landmarks;
  std::vector<Eigen::Vector2d> inlier_observations;
  sparse_mapping::RansacEstimateCamera(landmarks, observations, params_.num_ransac_iterations,
                                       params_.max_inlier_tolerance, &cam_, &inlier_landmarks, &inlier_observations);
  LogError("num inliear obs: " << inlier_observations.size());
  if (static_cast<int>(inlier_observations.size()) < params_.min_num_inliers) {
    LogError("ComputeRelativeTransform: Too few inlier matches, num matches: "
             << inlier_observations.size() << ", min num matches: " << params_.min_num_inliers << ".");
    return boost::none;
  }
  const Eigen::Isometry3d relative_transform(cam_.GetTransform().matrix());

  matches_ = StampedFeatureMatches(std::move(matches), previous_feature_depth_image_->timestamp,
                                   latest_feature_depth_image_->timestamp);
  LogError("rel trafo trans: " << relative_transform.translation().matrix());
  LogError("rel trafo trans norm: " << relative_transform.translation().norm());
  if (relative_transform.translation().norm() > 10) {
    LogError("large norm!!!");
    return boost::none;
  }

  return std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>{relative_transform,
                                                                   Eigen::Matrix<double, 6, 6>::Zero()};
}

void DepthImageAligner::AddLatestDepthImage(const lm::DepthImageMeasurement& latest_depth_image) {
  previous_feature_depth_image_ = std::move(latest_feature_depth_image_);
  if (params_.use_clahe) {
    lm::DepthImageMeasurement clahe_depth_image = latest_depth_image;
    clahe_->apply(latest_depth_image.image, clahe_depth_image.image);
    latest_feature_depth_image_.reset(
      new FeatureDepthImageMeasurement(clahe_depth_image, feature_detector_and_matcher_->detector()));
  } else
    latest_feature_depth_image_.reset(
      new FeatureDepthImageMeasurement(latest_depth_image, feature_detector_and_matcher_->detector()));
}
}  // namespace depth_odometry
