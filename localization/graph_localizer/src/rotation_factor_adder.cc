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

#include <graph_localizer/pose_rotation_factor.h>
#include <graph_localizer/rotation_factor_adder.h>
#include <graph_localizer/utilities.h>

#include <gtsam/inference/Symbol.h>

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/types.hpp>

namespace graph_localizer {
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
RotationFactorAdder::RotationFactorAdder(const RotationFactorAdderParams& params,
                                         std::shared_ptr<const FeatureTracker> feature_tracker)
    : RotationFactorAdder::Base(params), feature_tracker_(feature_tracker) {}

std::vector<FactorsToAdd> RotationFactorAdder::AddFactors(const lm::FeaturePointsMeasurement& measurement) const {
  std::vector<cv::Point2d> points_1;
  std::vector<cv::Point2d> points_2;
  double total_disparity = 0;
  for (const auto& feature_track_pair : feature_tracker_->feature_tracks()) {
    const auto& feature_track = feature_track_pair.second;
    if (feature_track.points.size() < 2) continue;
    // Get points for most recent and second to most recent images
    const auto& point_1 = feature_track.points[feature_track.points.size() - 2].image_point;
    const auto& point_2 = feature_track.points.back().image_point;
    points_1.emplace_back(cv::Point2d(point_1.x(), point_1.y()));
    points_2.emplace_back(cv::Point2d(point_2.x(), point_2.y()));
    total_disparity += (point_1 - point_2).norm();
  }

  if (points_1.size() < 5) {
    LOG(WARNING) << "RotationFactorAdder::AddFactors: Not enough corresponding points found.";
    return {};
  }
  const double average_disparity = total_disparity / points_1.size();
  if (average_disparity < params().min_avg_disparity) {
    VLOG(2) << "RotationFactorAdder::AddFactors: Disparity too low.";
    return {};
  }

  cv::Mat intrinsics;
  cv::eigen2cv(params().nav_cam_intrinsics.K(), intrinsics);
  // TODO(rsoussan): is this the correct point 1 and point 2 order????
  const auto essential_matrix = cv::findEssentialMat(points_1, points_2, intrinsics);
  cv::Mat cv_cam_1_R_cam_2;
  cv::Mat cv_translation;
  cv::recoverPose(essential_matrix, points_1, points_2, intrinsics, cv_cam_1_R_cam_2, cv_translation);
  Eigen::Matrix3d eigen_cam_1_R_cam_2;
  cv::cv2eigen(cv_cam_1_R_cam_2, eigen_cam_1_R_cam_2);
  const gtsam::Rot3& body_R_cam = params().body_T_nav_cam.rotation();
  // Put measurement in body frame since factor expects this
  const gtsam::Rot3 cam_1_R_cam_2(eigen_cam_1_R_cam_2);
  const gtsam::Rot3 body_1_R_body_2 = body_R_cam * cam_1_R_cam_2 * body_R_cam.inverse();
  // Create Rotation Factor
  const auto& points = feature_tracker_->feature_tracks().begin()->second.points;
  const KeyInfo pose_1_key_info(&sym::P, points[points.size() - 2].timestamp);
  const KeyInfo pose_2_key_info(&sym::P, points.back().timestamp);
  const gtsam::Vector3 rotation_noise_sigmas(
    (gtsam::Vector(3) << params().rotation_stddev, params().rotation_stddev, params().rotation_stddev).finished());
  const auto rotation_noise =
    Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(rotation_noise_sigmas)));
  const auto rotation_factor = boost::make_shared<gtsam::PoseRotationFactor>(
    body_1_R_body_2, rotation_noise, pose_1_key_info.UninitializedKey(), pose_2_key_info.UninitializedKey());
  FactorsToAdd rotation_factors_to_add;
  rotation_factors_to_add.push_back({{pose_1_key_info, pose_2_key_info}, rotation_factor});
  rotation_factors_to_add.SetTimestamp(pose_2_key_info.timestamp());
  VLOG(2) << "RotationFactorAdder::AddFactors: Added a rotation factor.";
  return {rotation_factors_to_add};
}
}  // namespace graph_localizer
