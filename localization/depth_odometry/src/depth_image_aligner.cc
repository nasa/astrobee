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

bool DepthImageAligner::ValidImagePoint(const Eigen::Vector2d& image_point) const {
  // TODO(rsoussan): Get these from somewhere else
  const int cols = latest_feature_depth_image_->cols();
  const int rows = latest_feature_depth_image_->rows();
  const double x_distance_to_border = std::min(image_point.x(), cols - image_point.x());
  const double y_distance_to_border = std::min(image_point.y(), rows - image_point.y());
  return (x_distance_to_border <= params_.max_x_distance_to_border &&
          y_distance_to_border <= params_.max_y_distance_to_border);
}

boost::optional<lc::PoseWithCovariance> DepthImageAligner::ComputeRelativeTransform() {
  if (!previous_feature_depth_image_ || !latest_feature_depth_image_) return boost::none;
  auto matches = feature_detector_and_matcher_->Match(*previous_feature_depth_image_, *latest_feature_depth_image_);

  std::vector<Eigen::Vector2d> source_observations;
  std::vector<Eigen::Vector3d> source_landmarks;
  std::vector<Eigen::Vector2d> target_observations;
  std::vector<Eigen::Vector3d> target_landmarks;
  // Get 3D points for image features
  for (auto match_it = matches.begin(); match_it != matches.end();) {
    const auto& source_image_point = match_it->source_point;
    const auto& target_image_point = match_it->target_point;
    const auto& target_point_3d =
      latest_feature_depth_image_->InterpolatePoint3D(target_image_point.x(), target_image_point.y());
    if (!target_point_3d || !ValidPoint(*target_point_3d) || target_point_3d->z < 0 ||
        !ValidImagePoint(source_image_point)) {
      match_it = matches.erase(match_it);
      continue;
    }
    const Eigen::Vector3d target_landmark(target_point_3d->x, target_point_3d->y, target_point_3d->z);
    target_landmarks.emplace_back(target_landmark);

    // It's ok if source landmark interpolation fails since this is not used for camera estimation
    const auto& source_point_3d =
      previous_feature_depth_image_->InterpolatePoint3D(source_image_point.x(), source_image_point.y());
    Eigen::Vector3d source_landmark(Eigen::Vector3d::Zero());
    if (source_point_3d && ValidPoint(*source_point_3d)) {
      source_landmark = Eigen::Vector3d(source_point_3d->x, source_point_3d->y, source_point_3d->z);
    }
    source_landmarks.emplace_back(source_landmark);

    // RansacEstimateCamera expects image points in undistorted centered frame
    Eigen::Vector2d undistorted_source_image_point;
    params_.camera_params->Convert<camera::DISTORTED, camera::UNDISTORTED_C>(source_image_point,
                                                                             &undistorted_source_image_point);
    source_observations.emplace_back(undistorted_source_image_point);

    Eigen::Vector2d undistorted_target_image_point;
    params_.camera_params->Convert<camera::DISTORTED, camera::UNDISTORTED_C>(target_image_point,
                                                                             &undistorted_target_image_point);
    target_observations.emplace_back(undistorted_target_image_point);

    ++match_it;
  }
  if (target_landmarks.size() < 4) {
    LogError("ComputeRelativeTransform: Too few points for Ransac PnP, need 4 but given " << target_landmarks.size()
                                                                                          << ".");
    return boost::none;
  }

  LogError("filtered_matches: " << matches.size());
  LogError("landmarks: " << target_landmarks.size() << ", observations: " << source_observations.size());
  std::vector<Eigen::Vector3d> inlier_target_landmarks;
  std::vector<Eigen::Vector2d> inlier_source_observations;
  sparse_mapping::RansacEstimateCamera(target_landmarks, source_observations, params_.num_ransac_iterations,
                                       params_.max_inlier_tolerance, &cam_, &inlier_target_landmarks,
                                       &inlier_source_observations);
  LogError("num inliear obs: " << inlier_source_observations.size());
  if (static_cast<int>(inlier_source_observations.size()) < params_.min_num_inliers) {
    LogError("ComputeRelativeTransform: Too few inlier matches, num matches: "
             << inlier_source_observations.size() << ", min num matches: " << params_.min_num_inliers << ".");
    return boost::none;
  }
  const Eigen::Isometry3d relative_transform(cam_.GetTransform().matrix());

  // Remove outlier target observations and source landmarks (since these aren't filled by RansacEstimateCamera)
  auto source_landmarks_it = source_landmarks.begin();
  auto target_observations_it = target_observations.begin();
  for (int i = 0, inlier_index = 0;
       i < static_cast<int>(source_observations.size()) &&
       inlier_index < static_cast<int>(inlier_source_observations.size()) &&
       source_landmarks_it != source_landmarks.end() && target_observations_it != target_observations.end();
       ++i) {
    if (source_observations[i].isApprox(inlier_source_observations[inlier_index]) &&
        target_landmarks[i].isApprox(inlier_target_landmarks[inlier_index])) {
      LogError("inlier index: " << inlier_index << ", total size: " << inlier_source_observations.size());
      ++inlier_index;
      ++source_landmarks_it;
      ++target_observations_it;
      continue;
    }
    source_landmarks_it = source_landmarks.erase(source_landmarks_it);
    target_observations_it = target_observations.erase(target_observations_it);
  }

  matches_ = DepthMatches(inlier_source_observations, target_observations, source_landmarks, inlier_target_landmarks,
                          previous_feature_depth_image_->timestamp, latest_feature_depth_image_->timestamp);
  LogError("rel trafo trans: " << relative_transform.translation().matrix());
  LogError("rel trafo trans norm: " << relative_transform.translation().norm());
  if (relative_transform.translation().norm() > 10) {
    LogError("large norm!!!");
    return boost::none;
  }

  return lc::PoseWithCovariance(relative_transform, Eigen::Matrix<double, 6, 6>::Zero());
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
