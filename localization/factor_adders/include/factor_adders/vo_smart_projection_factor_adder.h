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

#ifndef FACTOR_ADDERS_VO_SMART_PROJECTION_FACTOR_ADDER_H_
#define FACTOR_ADDERS_VO_SMART_PROJECTION_FACTOR_ADDER_H_

#include <factor_adders/measurement_based_factor_adder.h>
#include <factor_adders/vo_smart_projection_factor_adder_params.h>
#include <graph_factors/robust_smart_projection_pose_factor.h>
#include <localization_common/utilities.h>
#include <localization_measurements/feature_points_measurement.h>
#include <vision_common/spaced_feature_tracker.h>
#include <vision_common/utilities.h>

#include <vector>

namespace factor_adders {
using SmartFactorCalibration = gtsam::Cal3_S2;
using SmartFactorCamera = gtsam::PinholePose<SmartFactorCalibration>;
using RobustSmartFactor = gtsam::RobustSmartProjectionPoseFactor<SmartFactorCalibration>;
using SharedRobustSmartFactor = boost::shared_ptr<RobustSmartFactor>;
template <typename PoseNodeAdderType>
// Adds visual-odometry (VO) smart factors for feature tracks using GTSAM smart factors.
// Smart factors avoid bundle-adjustment by marginalizing out triangulated feature track
// landmark points before adding for optimization, making them much more efficient at the expense
// of accuracy. Since the landmark position for the point is not included during optimization,
// GTSAM performs retriangulation of the landmark during optimization if poses in the smart
// factor have changed by more than a set threshold. Retriangulation also triggers relinearization of the smart factor.
// Measurements are downsampled in the feature tracker, avoiding issues of downsampling
// from the latest measurement which may introduce many new timestamped pose nodes to
// the graph in addition to the latest measurement's timestamp.
class VoSmartProjectionFactorAdder
    : public MeasurementBasedFactorAdder<localization_measurements::FeaturePointsMeasurement> {
  using Base = MeasurementBasedFactorAdder<localization_measurements::FeaturePointsMeasurement>;

 public:
  VoSmartProjectionFactorAdder(const VoSmartProjectionFactorAdderParams& params,
                               std::shared_ptr<PoseNodeAdderType> node_adder);

  // Default constructor for serialization only.
  VoSmartProjectionFactorAdder() = default;

  // Const accesor to feature tracker
  const vision_common::SpacedFeatureTracker& feature_tracker() const;

 private:
  // Add factors using either set measurement spacing or max spacing.
  int AddMeasurementBasedFactors(const localization_common::Time oldest_allowed_time,
                                 const localization_common::Time newest_allowed_time,
                                 gtsam::NonlinearFactorGraph& factors) final;

  // Add factors using downsampled measurements, determined by the measurement_spacing param.
  // Measurements are in affect downsampled before being used for feature tracks.
  int AddFactorsUsingDownsampledMeasurements(gtsam::NonlinearFactorGraph& factors) const;

  // Helper function to add a smart factor given a set of feature track points.
  // Assumes points are ordered from oldest to latest, so oldest points are
  // added first and prioritized over later points given a max number of points to add.
  bool AddSmartFactor(const vision_common::FeaturePoints& feature_track_points,
                      gtsam::NonlinearFactorGraph& factors) const;

  // Checks the average distance from the mean and the number of points in the track.
  bool ValidTrack(const vision_common::FeaturePoints& feature_track) const;

  // Fixes invalid smart factors (that suffer cheirality errors) by removing individual measurements and if that fails
  // by removing measurement sequences.
  // If both fail, the smart factor is left in the graph in case it becomes
  // valid over the course of optimization.
  void FixSmartFactors(const gtsam::Values& values, gtsam::NonlinearFactorGraph& factors) const;

  // Attempts to fix smart factors by removing individual measurements.
  boost::optional<SharedRobustSmartFactor> FixSmartFactorByRemovingIndividualMeasurements(
    const gtsam::Values& values, const RobustSmartFactor& smart_factor) const;

  // Attempts to fix smart factors by removing measurement sequences.
  boost::optional<SharedRobustSmartFactor> FixSmartFactorByRemovingMeasurementSequence(
    const gtsam::Values& values, const RobustSmartFactor& smart_factor) const;

  // Helper function for creating and testing a fixed smart factor
  boost::optional<SharedRobustSmartFactor> FixedSmartFactor(
    const gtsam::Values& values, gtsam::PinholePose<gtsam::Cal3_S2>::MeasurementVector& measurements,
    gtsam::KeyVector& keys) const;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(node_adder_);
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(feature_tracker_);
  }

  std::shared_ptr<PoseNodeAdderType> node_adder_;
  VoSmartProjectionFactorAdderParams params_;
  std::shared_ptr<vision_common::SpacedFeatureTracker> feature_tracker_;
};

// Implementation
template <typename PoseNodeAdderType>
VoSmartProjectionFactorAdder<PoseNodeAdderType>::VoSmartProjectionFactorAdder(
  const VoSmartProjectionFactorAdderParams& params, std::shared_ptr<PoseNodeAdderType> node_adder)
    : Base(params), params_(params), node_adder_(node_adder) {
  feature_tracker_.reset(new vision_common::SpacedFeatureTracker(params.spaced_feature_tracker));
}

template <typename PoseNodeAdderType>
const vision_common::SpacedFeatureTracker& VoSmartProjectionFactorAdder<PoseNodeAdderType>::feature_tracker() const {
  return *feature_tracker_;
}

template <typename PoseNodeAdderType>
int VoSmartProjectionFactorAdder<PoseNodeAdderType>::AddMeasurementBasedFactors(
  const localization_common::Time oldest_allowed_time, const localization_common::Time newest_allowed_time,
  gtsam::NonlinearFactorGraph& factors) {
  // Update feature tracker with new measurements and remove these from the measurement buffer if nodes can be created
  // for them.
  ProcessMeasurements(oldest_allowed_time, newest_allowed_time,
                      [this](const localization_measurements::FeaturePointsMeasurement& measurement) {
                        // Don't add measurements until pose nodes can be created at their timestamp
                        if (!node_adder_->CanAddNode(measurement.timestamp)) return false;
                        feature_tracker_->Update(measurement.feature_points);
                        return true;
                      });
  // Remove old feature track points before adding factors
  feature_tracker_->RemoveOldPoints(oldest_allowed_time);
  // Remove old smart factors before adding new ones, since otherwise there would be repeat factors
  // for already existing feature tracks
  localization_common::DeleteFactors<RobustSmartFactor>(factors);

  // Create smart factors based on feature tracks
  int num_added_factors = 0;
  num_added_factors = AddFactorsUsingDownsampledMeasurements(factors);
  // Attempt to fix broken factors if enabled
  if (params_.fix_invalid_factors) FixSmartFactors(node_adder_->nodes().gtsam_values(), factors);
  return num_added_factors;
}

template <typename PoseNodeAdderType>
int VoSmartProjectionFactorAdder<PoseNodeAdderType>::AddFactorsUsingDownsampledMeasurements(
  gtsam::NonlinearFactorGraph& factors) const {
  int num_added_factors = 0;
  const auto spaced_feature_tracks = feature_tracker_->SpacedFeatureTracks();
  for (const auto& feature_track : spaced_feature_tracks) {
    if (num_added_factors >= params_.max_num_factors) break;
    if (ValidTrack(feature_track)) {
      if (AddSmartFactor(feature_track, factors)) ++num_added_factors;
    }
  }
  return num_added_factors;
}

template <typename PoseNodeAdderType>
bool VoSmartProjectionFactorAdder<PoseNodeAdderType>::AddSmartFactor(
  const vision_common::FeaturePoints& feature_track_points, gtsam::NonlinearFactorGraph& factors) const {
  SharedRobustSmartFactor smart_factor;
  const int num_feature_track_points = feature_track_points.size();
  // Optionally scale the noise with the number of points, as a longer track the relies on    a
  // longer history of poses tends to have higher error
  const double noise_scale =
    params_.scale_noise_with_num_points ? params_.noise_scale * num_feature_track_points : params_.noise_scale;
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, noise_scale * params_.cam_noise->sigma());
  smart_factor =
    boost::make_shared<RobustSmartFactor>(noise, params_.cam_intrinsics, params_.body_T_cam, params_.smart_factor,
                                          params_.rotation_only_fallback, params_.robust, params_.huber_k);

  for (int i = 0; i < static_cast<int>(feature_track_points.size()); ++i) {
    // Start in reverse so most recent measurements are added first
    const auto& feature_point = feature_track_points[feature_track_points.size() - 1 - i];
    if (i >= params_.max_num_points_per_factor) break;
    const auto& timestamp = feature_point.timestamp;
    // Add or get pose key
    if (!node_adder_->AddNode(timestamp, factors)) {
      LogError("AddSmartFactor: Failed to add node for timestamp " << std::setprecision(15) << timestamp << ".");
      return false;
    }
    const auto keys = node_adder_->Keys(timestamp);
    if (keys.empty()) {
      LogError("AddSmartFactor: Failed to get keys for timestamp " << std::setprecision(15) << timestamp << ".");
      return false;
    }
    // Assumes first key is pose
    const auto& pose_key = keys[0];

    smart_factor->add(SmartFactorCamera::Measurement(feature_point.image_point), pose_key);
  }
  factors.push_back(smart_factor);
  return true;
}

template <typename PoseNodeAdderType>
bool VoSmartProjectionFactorAdder<PoseNodeAdderType>::ValidTrack(
  const vision_common::FeaturePoints& feature_track) const {
  const double average_distance_from_mean = vision_common::AverageDistanceFromMean(feature_track);
  if (static_cast<int>(feature_track.size()) < params_.min_num_points_per_factor) return false;
  return (average_distance_from_mean >= params_.min_avg_distance_from_mean);
}

template <typename PoseNodeAdderType>
void VoSmartProjectionFactorAdder<PoseNodeAdderType>::FixSmartFactors(const gtsam::Values& values,
                                                                      gtsam::NonlinearFactorGraph& factors) const {
  for (auto& factor : factors) {
    auto smart_factor = dynamic_cast<RobustSmartFactor*>(factor.get());
    if (!smart_factor) continue;
    // Can't remove measurements if there are only 2 or fewer
    if (smart_factor->measured().size() <= 2) continue;
    const auto point = smart_factor->triangulateSafe(smart_factor->cameras(values));
    if (point.valid()) continue;
    // Invalid factor, first attempt to fix by removing individiual measurements.
    {
      const auto fixed_smart_factor = FixSmartFactorByRemovingIndividualMeasurements(values, *smart_factor);
      if (fixed_smart_factor) {
        factor = *fixed_smart_factor;
        continue;
      }
    }
    // If removing individiual measurements fails, attempt to fix by removing measurement
    // sequences.
    {
      const auto fixed_smart_factor = FixSmartFactorByRemovingMeasurementSequence(values, *smart_factor);
      if (fixed_smart_factor) {
        factor = *fixed_smart_factor;
        continue;
      }
    }
    // Don't remove factor in case it becomes valid over the course of optimization.
    LogDebug("SplitSmartFactorsIfNeeded: Failed to fix smart factor");
  }
}

template <typename PoseNodeAdderType>
boost::optional<SharedRobustSmartFactor>
VoSmartProjectionFactorAdder<PoseNodeAdderType>::FixSmartFactorByRemovingIndividualMeasurements(
  const gtsam::Values& values, const RobustSmartFactor& smart_factor) const {
  // TODO(rsoussan): Make this more efficient by enabled removal of measurements and keys in smart factor
  const auto original_measurements = smart_factor.measured();
  const auto original_keys = smart_factor.keys();
  int measurement_index_to_remove;
  // Start with latest measurement
  for (measurement_index_to_remove = original_measurements.size() - 1; measurement_index_to_remove >= 0;
       --measurement_index_to_remove) {
    gtsam::PinholePose<gtsam::Cal3_S2>::MeasurementVector measurements_to_add;
    gtsam::KeyVector keys_to_add;
    for (int i = 0; i < static_cast<int>(original_measurements.size()); ++i) {
      if (i == measurement_index_to_remove) continue;
      measurements_to_add.emplace_back(original_measurements[i]);
      keys_to_add.emplace_back(original_keys[i]);
    }
    const auto fixed_smart_factor = FixedSmartFactor(values, measurements_to_add, keys_to_add);
    if (fixed_smart_factor) {
      LogDebug("FixSmartFactorByRemovingIndividualMeasurements: Fixed by removing measurement "
               << measurement_index_to_remove << ", num original measurements: " << original_measurements.size());
      return fixed_smart_factor;
    }
  }
  return boost::none;
}

template <typename PoseNodeAdderType>
boost::optional<SharedRobustSmartFactor>
VoSmartProjectionFactorAdder<PoseNodeAdderType>::FixSmartFactorByRemovingMeasurementSequence(
  const gtsam::Values& values, const RobustSmartFactor& smart_factor) const {
  constexpr int min_num_measurements = 2;
  // TODO(rsoussan): Make this more efficient by enabled removal of measurements and keys in smart factor
  const auto original_measurements = smart_factor.measured();
  const auto original_keys = smart_factor.keys();
  int num_measurements_to_add = original_measurements.size() - 1;
  // Try to remove min number of most recent measurements
  while (num_measurements_to_add >= min_num_measurements) {
    gtsam::PinholePose<gtsam::Cal3_S2>::MeasurementVector measurements_to_add;
    gtsam::KeyVector keys_to_add;
    for (int i = 0; i < num_measurements_to_add; ++i) {
      measurements_to_add.emplace_back(original_measurements[i]);
      keys_to_add.emplace_back(original_keys[i]);
    }
    const auto fixed_smart_factor = FixedSmartFactor(values, measurements_to_add, keys_to_add);
    if (fixed_smart_factor) {
      LogDebug(
        "FixSmartFactorByRemovingMeasurementSequence: Fixed smart factor by removing most recent "
        "measurements. Original "
        "measurement size: "
        << original_measurements.size() << ", new size: " << num_measurements_to_add);
      return fixed_smart_factor;
    } else {
      --num_measurements_to_add;
    }
  }
  num_measurements_to_add = original_measurements.size() - 1;
  // Try to remove min number of oldest measurements
  while (num_measurements_to_add >= min_num_measurements) {
    gtsam::PinholePose<gtsam::Cal3_S2>::MeasurementVector measurements_to_add;
    gtsam::KeyVector keys_to_add;
    for (int i = original_measurements.size() - num_measurements_to_add;
         i < static_cast<int>(original_measurements.size()); ++i) {
      measurements_to_add.emplace_back(original_measurements[i]);
      keys_to_add.emplace_back(original_keys[i]);
    }
    const auto fixed_smart_factor = FixedSmartFactor(values, measurements_to_add, keys_to_add);
    if (fixed_smart_factor) {
      LogDebug(
        "FixSmartFactorByRemovingMeasurementSequence: Fixed smart factor by removing oldest measurements. "
        "Original "
        "measurement size: "
        << original_measurements.size() << ", new size: " << num_measurements_to_add);
      return fixed_smart_factor;
    } else {
      --num_measurements_to_add;
    }
  }
  // Failed to fix smart factor
  return boost::none;
  // TODO(rsoussan): delete factor if fail to find acceptable new one?
  // TODO(rsoussan): attempt to make a second factor with remaining measuremnts!!!
}

template <typename PoseNodeAdderType>
boost::optional<SharedRobustSmartFactor> VoSmartProjectionFactorAdder<PoseNodeAdderType>::FixedSmartFactor(
  const gtsam::Values& values, gtsam::PinholePose<gtsam::Cal3_S2>::MeasurementVector& measurements,
  gtsam::KeyVector& keys) const {
  auto new_smart_factor = boost::make_shared<RobustSmartFactor>(
    params_.cam_noise, params_.cam_intrinsics, params_.body_T_cam, params_.smart_factor, params_.rotation_only_fallback,
    params_.robust, params_.huber_k);
  new_smart_factor->add(measurements, keys);
  const auto new_point = new_smart_factor->triangulateSafe(new_smart_factor->cameras(values));
  if (!new_point.valid()) return boost::none;
  return new_smart_factor;
}
}  // namespace factor_adders
#endif  // FACTOR_ADDERS_VO_SMART_PROJECTION_FACTOR_ADDER_H_
