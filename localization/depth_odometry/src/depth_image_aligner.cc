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
    clahe_ = cv::createCLAHE(params_.clahe_clip_limit, cv::Size(params_.clahe_grid_length, params_.clahe_grid_length));
  } else {
    LogFatal("DepthImageAligner: Invalid feature detector and matcher.");
    std::exit(1);
  }
}

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>>
DepthImageAligner::ComputeRelativeTransform() {
  if (!previous_feature_depth_image_ || !latest_feature_depth_image_) return boost::none;
  const auto matches =
    feature_detector_and_matcher_->Match(*previous_feature_depth_image_, *latest_feature_depth_image_);
  std::vector<cv::Point3d> match_points_3d;
  std::vector<cv::Point2d> match_image_points;
  std::vector<cv::DMatch> filtered_matches;
  // Get 3D points for image features
  for (const auto& match : matches) {
    const auto& latest_image_point = latest_feature_depth_image_->keypoints()[match.trainIdx].pt;
    const auto& latest_point_3d =
      latest_feature_depth_image_->InterpolatePoint3D(latest_image_point.x, latest_image_point.y);
    if (!latest_point_3d || !ValidPoint(*latest_point_3d)) continue;
    match_points_3d.emplace_back(cv::Point3d(latest_point_3d->x, latest_point_3d->y, latest_point_3d->z));
    const auto image_point = static_cast<cv::Point2d>(previous_feature_depth_image_->keypoints()[match.queryIdx].pt);
    match_image_points.emplace_back(image_point);
    // LogError("3d point: " << latest_point_3d->x << ", " << latest_point_3d->y << ", " << latest_point_3d->z);
    // LogError("image point: " << image_point.x << ", " << image_point.y);
    filtered_matches.emplace_back(match);
  }
  LogError("pnp points: " << match_points_3d.size());
  if (match_points_3d.size() < 4) {
    LogError("ComputeRelativeTransform: Too few points for Ransac PnP, need 4 but given " << match_points_3d.size()
                                                                                          << ".");
    return boost::none;
  }
  std::vector<Eigen::Vector3d> inlier_landmarks;
  std::vector<Eigen::Vector2d> inlier_observations;

  std::vector<Eigen::Vector3d> landmarks;
  std::vector<Eigen::Vector2d> observations;
  for (size_t i = 0; i < match_points_3d.size(); ++i) {
    const auto& landmark = match_points_3d[i];
    landmarks.emplace_back(Eigen::Vector3d(landmark.x, landmark.y, landmark.z));
    const auto& image_point = match_image_points[i];
    // RansacEstimateCamera expects image points in undistorted centered frame
    Eigen::Vector2d undistorted_c_observation;
    params_.camera_params->Convert<camera::DISTORTED, camera::UNDISTORTED_C>(
      Eigen::Vector2d(image_point.x, image_point.y), &undistorted_c_observation);
    observations.emplace_back(undistorted_c_observation);
  }

  LogError("filtered matches: " << filtered_matches.size());
  LogError("landmarks: " << landmarks.size() << ", observations: " << observations.size());
  sparse_mapping::RansacEstimateCamera(landmarks, observations, params_.num_ransac_iterations,
                                       params_.max_inlier_tolerance, &cam_, &inlier_landmarks, &inlier_observations);
  LogError("num inliear obs: " << inlier_observations.size());
  if (static_cast<int>(inlier_observations.size()) < params_.min_num_inliers) {
    LogError("ComputeRelativeTransform: Too few inlier matches, num matches: "
             << inlier_observations.size() << ", min num matches: " << params_.min_num_inliers << ".");
    return boost::none;
  }
  const Eigen::Isometry3d relative_transform(cam_.GetTransform().matrix());
  correspondences_ = ImageCorrespondences(
    filtered_matches, previous_feature_depth_image_->keypoints(), latest_feature_depth_image_->keypoints(),
    previous_feature_depth_image_->timestamp, latest_feature_depth_image_->timestamp);

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
