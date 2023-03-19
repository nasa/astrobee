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

#ifndef FACTOR_ADDERS_LOC_FACTOR_ADDER_H_
#define FACTOR_ADDERS_LOC_FACTOR_ADDER_H_

#include <factor_adders/single_measurement_based_factor_adder.h>
#include <factor_adders/loc_factor_adder_params.h>
#include <graph_factors/loc_pose_factor.h>
#include <graph_factors/loc_projection_factor.h>
#include <localization_common/utilities.h>
#include <localization_measurements/matched_projections_measurement.h>

#include <vector>

namespace factor_adders {
template <typename PoseNodeAdderType>
class LocFactorAdder
    : public SingleMeasurementBasedFactorAdder<localization_measurements::MatchedProjectionsMeasurement> {
  using Base = SingleMeasurementBasedFactorAdder<localization_measurements::MatchedProjectionsMeasurement>;

 public:
  LocFactorAdder(const LocFactorAdderParams& params, const std::shared_ptr<PoseNodeAdderType> node_adder);

 private:
  // Adds loc projection factor or loc pose factor depending on params.
  // If the loc projection factor is selected but fails due to a reprojection error, adds a loc pose factor
  // as a fallback.
  int AddFactorsForSingleMeasurement(
    const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
    gtsam::NonlinearFactorGraph& factors) final;

  // Adds a loc pose factor (pose prior)
  int AddLocPoseFactor(const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
                       gtsam::NonlinearFactorGraph& factors) final;

  // Adds a loc projection factor. If a reprojection error occurs, adds a loc pose factor.
  int AddLocProjectionFactor(
    const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
    gtsam::NonlinearFactorGraph& factors) final;

  // Helper function to add a pose node if needed and return the node's key.
  boost::optional<gtsam::Key> AddPoseNode(const localization_common::Time timestamp,
                                          gtsam::NonlinearFactorGraph& factors);

  std::shared_ptr<PoseNodeAdderType> node_adder_;
  LocFactorAdderParams params_;
  localization_common::Averager num_landmarks_averager_ = localization_common::Averager("Num Landmarks");
  gtsam::Vector6 pose_prior_noise_sigmas_;
};

// Implementation
template <typename PoseNodeAdderType>
LocFactorAdder<PoseNodeAdderType>::LocFactorAdder(const LocFactorAdderParams& params,
                                                  std::shared_ptr<PoseNodeAdderType> node_adder)
    : Base(params), params_(params), node_adder_(node_adder) {
  const gtsam::Vector6 pose_prior_noise_sigmas((gtsam::Vector(6) << params_.prior_translation_stddev,
                                                params_.prior_translation_stddev, params_.prior_translation_stddev,
                                                params_.prior_quaternion_stddev, params_.prior_quaternion_stddev,
                                                params_.prior_quaternion_stddev)
                                                 .finished());
}

template <typename PoseNodeAdderType>
int LocFactorAdder<PoseNodeAdderType>::AddFactorsForSingleMeasurement(
  const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
  gtsam::NonlinearFactorGraph& factors) {
  if (matched_projections_measurement.matched_projections.empty()) {
    LogDebug("AddFactorsForSingleMeasurement: Empty measurement.");
    return 0;
  }

  if (static_cast<int>(matched_projections_measurement.matched_projections.size()) < params().min_num_matches) {
    LogDebug("AddFactorsForSingleMeasurement: Not enough matches in projection measurement.");
    return 0;
  }

  const int num_landmarks = matched_projections_measurement.matched_projections.size();
  num_landmarks_averager_.Update(num_landmarks);
  num_loc_projection_factors = 0;
  num_loc_pose_factors = 0;
  if (params_.add_projections) {
    num_loc_projection_factors = AddLocProjectionFactor(matched_projections_measurement, factors);
  }
  if (params_.add_pose_priors) {
    num_loc_pose_factors = AddLocPoseFactor(matched_projections_measurement, factors);
  }
  return num_loc_pose_factors + num_loc_projection_factors;
}

template <typename PoseNodeAdderType>
int LocFactorAdder<PoseNodeAdderType>::AddLocProjectionFactor(
  const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
  gtsam::NonlinearFactorGraph& factors) {
  double noise_scale = params_.projection_noise_scale;
  if (params_.scale_projection_noise_with_num_landmarks) {
    noise_scale *= std::pow((num_landmarks_averager_.average() / static_cast<double>(num_landmarks)), 2);
  }
  const auto pose_key = AddPoseNode(matched_projections_measurement.timestamp, factors);
  if (!pose_key) {
    LogError("AddLocProjectionFactors: Failed to get pose key".);
    return 0;
  }

  int num_loc_projection_factors = 0;
  for (const auto& matched_projection : matched_projections_measurement.matched_projections) {
    // TODO(rsoussan): Pass sigma insted of already constructed isotropic noise
    const gtsam::SharedIsotropic scaled_noise(
      gtsam::noiseModel::Isotropic::Sigma(2, noise_scale * params_.cam_noise->sigma()));
    gtsam::LocProjectionFactor<>::shared_ptr loc_projection_factor(new gtsam::LocProjectionFactor<>(
      matched_projection.image_point, matched_projection.map_point, go::Robust(scaled_noise, params_.huber_k), pose_key,
      params_.cam_intrinsics, params_.body_T_cam));
    // Set world_T_cam estimate in case need to use it as a fallback
    loc_projection_factor->setWorldTCam(matched_projections_measurement.global_T_cam);
    factors.push_back(loc_projection_factor);
    ++num_loc_projection_factors;
    if (num_loc_projection_factors >= params_.max_num_factors) break;
  }

  LogDebug("AddFactorsForSingleMeasurement: Added " << num_loc_projection_factors
                                                    << " loc projection factors at timestamp " << timestamp << ".");
  return num_loc_projection_factors;
}

template <typename PoseNodeAdderType>
int LocFactorAdder<PoseNodeAdderType>::AddLocPoseFactor(
  const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
  gtsam::NonlinearFactorGraph& factors) {
  double noise_scale = params_.pose_noise_scale;
  if (params_.scale_pose_noise_with_num_landmarks) {
    noise_scale *= std::pow((num_landmarks_averager_.average() / static_cast<double>(num_landmarks)), 2);
  }
  const auto pose_key = AddPoseNode(matched_projections_measurement.timestamp, factors);
  if (!pose_key) {
    LogError("AddLocProjectionFactors: Failed to get pose key".);
    return 0;
  }

  const auto pose_noise = go::Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(noise_scale * pose_prior_noise_sigmas_)),
    params_.huber_k);

  gtsam::LocPoseFactor::shared_ptr pose_prior_factor(new gtsam::LocPoseFactor(
    pose_key, matched_projections_measurement.global_T_cam * params_.body_T_cam.inverse(), pose_noise));
  factors.push_back(pose_prior_factor);
  LogDebug("AddLocPoseFactor: Added loc pose prior factor at timestamp " << timestamp << ".");
  return 1;
}

template <typename PoseNodeAdderType>
boost::optional<gtsam::Key> LocFactorAdder<PoseNodeAdderType>::AddPoseNode(const localization_common::Time timestamp,
                                                                           gtsam::NonlinearFactorGraph& factors) {
  if (!node_adder_->AddNode(timestamp, factors)) {
    LogError("AddLocPoseFactor: Failed to add node for timestamp " << timestamp << ".");
    return boost::none;
  }
  const auto keys = node_adder_->Keys(timestamp);
  if (keys.empty()) {
    LogError("AddLocPoseFactor: Failed to get keys for timestamp " << timestamp << ".");
    return boost::none;
  }

  // Assumes first key is pose
  const auto& pose_key = keys[0];
  return pose_key;
}
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_LOC_FACTOR_ADDER_H_
