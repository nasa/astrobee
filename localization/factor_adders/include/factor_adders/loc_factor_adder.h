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
#include <localization_common/averager.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/matched_projections_measurement.h>

#include <vector>

namespace factor_adders {
// SingleMeasurementBasedFactorAdder that adds either loc projection factors and loc pose priors for given matched
// projection measurements.
template <typename PoseNodeAdderType>
class LocFactorAdder
    : public SingleMeasurementBasedFactorAdder<localization_measurements::MatchedProjectionsMeasurement> {
  using Base = SingleMeasurementBasedFactorAdder<localization_measurements::MatchedProjectionsMeasurement>;

 public:
  LocFactorAdder(const LocFactorAdderParams& params, const std::shared_ptr<PoseNodeAdderType> node_adder);

  // Default constructor for serialization only.
  LocFactorAdder() = default;

 private:
  // Adds loc projection factor or loc pose factor depending on params.
  // If the loc projection factor is selected but fails due to a reprojection error, adds a loc pose factor
  // as a fallback.
  int AddFactorsForSingleMeasurement(
    const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
    gtsam::NonlinearFactorGraph& factors) final;

  // Returns whether the node adder can add a node at the provided time.
  bool CanAddFactor(const localization_measurements::MatchedProjectionsMeasurement& measurement) const final;

  // Adds a loc pose factor (pose prior)
  int AddLocPoseFactor(const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
                       gtsam::NonlinearFactorGraph& factors);

  // Adds a loc projection factor. If a reprojection error occurs, adds a loc pose factor.
  int AddLocProjectionFactor(
    const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
    gtsam::NonlinearFactorGraph& factors);

  // Helper function to add a pose node if needed and return the node's key.
  boost::optional<gtsam::Key> AddPoseNode(const localization_common::Time timestamp,
                                          gtsam::NonlinearFactorGraph& factors);

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(node_adder_);
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(num_landmarks_averager_);
    ar& BOOST_SERIALIZATION_NVP(pose_prior_noise_sigmas_);
  }

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
  pose_prior_noise_sigmas_ = (gtsam::Vector(6) << params_.prior_translation_stddev, params_.prior_translation_stddev,
                              params_.prior_translation_stddev, params_.prior_quaternion_stddev,
                              params_.prior_quaternion_stddev, params_.prior_quaternion_stddev)
                               .finished();
}

template <typename PoseNodeAdderType>
int LocFactorAdder<PoseNodeAdderType>::AddFactorsForSingleMeasurement(
  const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
  gtsam::NonlinearFactorGraph& factors) {
  if (matched_projections_measurement.matched_projections.empty()) {
    LogDebug("AddFactorsForSingleMeasurement: Empty measurement.");
    return 0;
  }

  if (static_cast<int>(matched_projections_measurement.matched_projections.size()) <
      params_.min_num_matches_per_measurement) {
    LogDebug("AddFactorsForSingleMeasurement: Not enough matches in projection measurement.");
    return 0;
  }

  const int num_landmarks = matched_projections_measurement.matched_projections.size();
  num_landmarks_averager_.Update(num_landmarks);
  int num_loc_projection_factors = 0;
  int num_loc_pose_factors = 0;
  if (params_.add_projection_factors) {
    num_loc_projection_factors = AddLocProjectionFactor(matched_projections_measurement, factors);
    // Add loc pose factors as a fallback if all projection factors failed and fallback
    // enabled
    if (num_loc_projection_factors == 0 && params_.add_prior_if_projection_factors_fail) {
      num_loc_pose_factors = AddLocPoseFactor(matched_projections_measurement, factors);
    }
  }
  // Add loc pose factors if enabled and fallback hasn't already been triggered
  if (params_.add_pose_priors && num_loc_pose_factors == 0) {
    num_loc_pose_factors = AddLocPoseFactor(matched_projections_measurement, factors);
  }
  return num_loc_pose_factors + num_loc_projection_factors;
}

template <typename PoseNodeAdderType>
bool LocFactorAdder<PoseNodeAdderType>::CanAddFactor(
  const localization_measurements::MatchedProjectionsMeasurement& measurement) const {
  return node_adder_->CanAddNode(measurement.timestamp);
}

template <typename PoseNodeAdderType>
int LocFactorAdder<PoseNodeAdderType>::AddLocProjectionFactor(
  const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
  gtsam::NonlinearFactorGraph& factors) {
  double noise_scale = params_.projection_noise_scale;
  if (params_.scale_projection_noise_with_num_landmarks) {
    const int num_landmarks = matched_projections_measurement.matched_projections.size();
    noise_scale *= std::pow((num_landmarks_averager_.average() / static_cast<double>(num_landmarks)), 2);
  }
  const auto pose_key = AddPoseNode(matched_projections_measurement.timestamp, factors);
  if (!pose_key) {
    LogError("AddLocProjectionFactors: Failed to get pose key.");
    return 0;
  }
  const auto world_T_body = node_adder_->nodes().template Value<gtsam::Pose3>(*pose_key);
  if (!world_T_body) {
    LogError("AddLocProjectionFactors: Failed to get world_T_body at timestamp "
             << matched_projections_measurement.timestamp << ".");
    return 0;
  }

  int num_loc_projection_factors = 0;
  for (const auto& matched_projection : matched_projections_measurement.matched_projections) {
    gtsam::SharedNoiseModel noise;
    // Use the landmark distance from the camera to inversely scale the noise if desired.
    if (params_.scale_projection_noise_with_landmark_distance) {
      const Eigen::Vector3d& world_t_landmark = matched_projection.map_point;
      const Eigen::Isometry3d nav_cam_T_world =
        localization_common::EigenPose(*world_T_body * params_.body_T_cam).inverse();
      const gtsam::Point3 nav_cam_t_landmark = nav_cam_T_world * world_t_landmark;
      // Don't use robust cost here to more effectively correct a drift occurance
      noise = gtsam::SharedIsotropic(
        gtsam::noiseModel::Isotropic::Sigma(2, params_.projection_noise_scale * 1.0 / nav_cam_t_landmark.z()));
    } else {
      noise = localization_common::Robust(
        gtsam::SharedIsotropic(gtsam::noiseModel::Isotropic::Sigma(2, noise_scale * params_.cam_noise->sigma())),
        params_.huber_k);
    }
    gtsam::LocProjectionFactor<>::shared_ptr loc_projection_factor(
      new gtsam::LocProjectionFactor<>(matched_projection.image_point, matched_projection.map_point, noise, *pose_key,
                                       params_.cam_intrinsics, params_.body_T_cam));
    // Check for errors, discard factor if too large of projection error occurs
    // or a cheirality error occurs
    const auto error = (loc_projection_factor->evaluateError(*world_T_body)).norm();
    if (error > params_.max_valid_projection_error) continue;
    const auto cheirality_error = loc_projection_factor->cheiralityError(*world_T_body);
    if (cheirality_error) continue;
    factors.push_back(loc_projection_factor);
    ++num_loc_projection_factors;
    if (num_loc_projection_factors >= params_.max_num_projection_factors) break;
  }

  LogDebug("AddFactorsForSingleMeasurement: Added " << num_loc_projection_factors
                                                    << " loc projection factors at timestamp "
                                                    << matched_projections_measurement.timestamp << ".");
  return num_loc_projection_factors;
}

template <typename PoseNodeAdderType>
int LocFactorAdder<PoseNodeAdderType>::AddLocPoseFactor(
  const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
  gtsam::NonlinearFactorGraph& factors) {
  double noise_scale = params_.pose_noise_scale;
  if (params_.scale_pose_noise_with_num_landmarks) {
    const int num_landmarks = matched_projections_measurement.matched_projections.size();
    noise_scale *= std::pow((num_landmarks_averager_.average() / static_cast<double>(num_landmarks)), 2);
  }
  const auto pose_key = AddPoseNode(matched_projections_measurement.timestamp, factors);
  if (!pose_key) {
    LogError("AddLocProjectionFactors: Failed to get pose key.");
    return 0;
  }

  const auto pose_noise = localization_common::Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(noise_scale * pose_prior_noise_sigmas_)),
    params_.huber_k);

  gtsam::LocPoseFactor::shared_ptr pose_prior_factor(new gtsam::LocPoseFactor(
    *pose_key, matched_projections_measurement.global_T_cam * params_.body_T_cam.inverse(), pose_noise));
  factors.push_back(pose_prior_factor);
  LogDebug("AddLocPoseFactor: Added loc pose prior factor at timestamp " << matched_projections_measurement.timestamp
                                                                         << ".");
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
