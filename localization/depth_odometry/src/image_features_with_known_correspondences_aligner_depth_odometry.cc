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

#include <depth_odometry/image_features_with_known_correspondences_aligner_depth_odometry.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <point_cloud_common/utilities.h>
#include <vision_common/brisk_feature_detector_and_matcher.h>
#include <vision_common/lk_optical_flow_feature_detector_and_matcher.h>
#include <vision_common/surf_feature_detector_and_matcher.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace pc = point_cloud_common;
namespace vc = vision_common;

ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry::ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry(
  const ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams& params)
    : params_(params), aligner_(params.aligner) {
  if (params_.detector == "brisk") {
    feature_detector_and_matcher_.reset(
      new vc::BriskFeatureDetectorAndMatcher(params_.brisk_feature_detector_and_matcher));
  } else if (params_.detector == "lk_optical_flow") {
    feature_detector_and_matcher_.reset(
      new vc::LKOpticalFlowFeatureDetectorAndMatcher(params_.lk_optical_flow_feature_detector_and_matcher));
  } else if (params_.detector == "surf") {
    feature_detector_and_matcher_.reset(
      new vc::SurfFeatureDetectorAndMatcher(params_.surf_feature_detector_and_matcher));
  } else {
    LogFatal("ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry: Invalid feature detector and matcher.");
  }

  if (params_.use_clahe)
    clahe_ = cv::createCLAHE(params_.clahe_clip_limit, cv::Size(params_.clahe_grid_length, params_.clahe_grid_length));

  normals_required_ = params_.aligner.use_point_to_plane_cost || params_.aligner.use_symmetric_point_to_plane_cost;
}

boost::optional<PoseWithCovarianceAndCorrespondences>
ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry::DepthImageCallback(
  const lm::DepthImageMeasurement& depth_image_measurement) {
  if (!previous_depth_image_features_and_points_ && !latest_depth_image_features_and_points_) {
    latest_depth_image_features_and_points_.reset(new DepthImageFeaturesAndPoints(
      depth_image_measurement.depth_image, *(feature_detector_and_matcher_->detector()), clahe_, normals_required_));
    latest_timestamp_ = depth_image_measurement.timestamp;
    return boost::none;
  }
  const lc::Time timestamp = depth_image_measurement.timestamp;
  if (timestamp < latest_timestamp_) {
    LogWarning("DepthImageCallback: Out of order measurement received.");
    return boost::none;
  }

  previous_depth_image_features_and_points_ = latest_depth_image_features_and_points_;
  previous_timestamp_ = latest_timestamp_;
  latest_depth_image_features_and_points_.reset(new DepthImageFeaturesAndPoints(
    depth_image_measurement.depth_image, *(feature_detector_and_matcher_->detector()), clahe_, normals_required_));
  latest_timestamp_ = timestamp;

  const double time_diff = latest_timestamp_ - previous_timestamp_;
  if (time_diff > params_.max_time_diff) {
    LogWarning("DepthImageCallback: Time difference too large, time diff: " << time_diff);
    return boost::none;
  }

  const auto& matches = feature_detector_and_matcher_->Match(previous_depth_image_features_and_points_->feature_image(),
                                                             latest_depth_image_features_and_points_->feature_image());
  // Get 3d points and required normals for matches
  // Continue if any of these, including image points, are invalid
  std::vector<Eigen::Vector2d> source_image_points;
  std::vector<Eigen::Vector2d> target_image_points;
  std::vector<Eigen::Vector3d> source_landmarks;
  std::vector<Eigen::Vector3d> target_landmarks;
  std::vector<Eigen::Vector3d> source_normals;
  std::vector<Eigen::Vector3d> target_normals;
  for (int i = 0; i < static_cast<int>(matches.size()); ++i) {
    const auto& match = matches[i];
    const auto& source_image_point = match.source_point;
    const auto& target_image_point = match.target_point;
    if (!ValidImagePoint(source_image_point) || !ValidImagePoint(target_image_point)) continue;
    const auto source_point_3d = previous_depth_image_features_and_points_->depth_image().InterpolatePoint3D(
      source_image_point.x(), source_image_point.y());
    const auto target_point_3d = latest_depth_image_features_and_points_->depth_image().InterpolatePoint3D(
      target_image_point.x(), target_image_point.y());
    if (!Valid3dPoint(source_point_3d) || !Valid3dPoint(target_point_3d)) continue;
    const Eigen::Vector3d source_landmark = pc::Vector3d(*source_point_3d);
    const Eigen::Vector3d target_landmark = pc::Vector3d(*target_point_3d);
    if (normals_required_) {
      const auto target_normal =
        latest_depth_image_features_and_points_->Normal(target_landmark, params_.aligner.normal_search_radius);
      if (!target_normal) continue;
      if (params_.aligner.use_symmetric_point_to_plane_cost) {
        const auto source_normal =
          previous_depth_image_features_and_points_->Normal(source_landmark, params_.aligner.normal_search_radius);
        if (!source_normal) continue;
        source_normals.emplace_back(*source_normal);
      }
      target_normals.emplace_back(*target_normal);
    }

    source_image_points.emplace_back(source_image_point);
    target_image_points.emplace_back(target_image_point);
    source_landmarks.emplace_back(source_landmark);
    target_landmarks.emplace_back(target_landmark);
  }

  if (target_landmarks.size() < 4) {
    LogError("DepthImageCallback: Too few points provided, need 4 but given " << target_landmarks.size() << ".");
    return boost::none;
  }

  // TODO(rsoussan): This isn't required with std:optional, remove when upgrade to c++17 and change normals
  // containers to be boost::optional types
  boost::optional<const std::vector<Eigen::Vector3d>&> source_normals_ref =
    normals_required_ && params_.aligner.use_symmetric_point_to_plane_cost
      ? boost::optional<const std::vector<Eigen::Vector3d>&>(source_normals)
      : boost::none;
  boost::optional<const std::vector<Eigen::Vector3d>&> target_normals_ref =
    normals_required_ ? boost::optional<const std::vector<Eigen::Vector3d>&>(target_normals) : boost::none;

  const auto target_T_source =
    aligner_.ComputeRelativeTransform(source_landmarks, target_landmarks, source_normals_ref, target_normals_ref);
  if (!target_T_source) {
    LogWarning("DepthImageCallback: Failed to get relative transform.");
    return boost::none;
  }

  const auto source_T_target = lc::InvertPoseWithCovariance(*target_T_source);

  if (!lc::PoseCovarianceSane(source_T_target.covariance, params_.position_covariance_threshold,
                              params_.orientation_covariance_threshold)) {
    LogWarning("DepthImageCallback: Sanity check failed - invalid covariance.");
    return boost::none;
  }

  return PoseWithCovarianceAndCorrespondences(
    source_T_target, DepthCorrespondences(source_image_points, target_image_points, source_landmarks, target_landmarks),
    previous_timestamp_, latest_timestamp_);
}

bool ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry::ValidImagePoint(
  const Eigen::Vector2d& image_point) const {
  const int cols = latest_depth_image_features_and_points_->feature_image().cols();
  const int rows = latest_depth_image_features_and_points_->feature_image().rows();
  const double x_distance_to_border = std::min(image_point.x(), cols - image_point.x());
  const double y_distance_to_border = std::min(image_point.y(), rows - image_point.y());
  return (x_distance_to_border >= params_.min_x_distance_to_border &&
          y_distance_to_border >= params_.min_y_distance_to_border);
}

bool ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry::Valid3dPoint(
  const boost::optional<pcl::PointXYZI>& point) const {
  return point && pc::ValidPoint(*point) && point->z >= 0;
}
}  // namespace depth_odometry
