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
#ifndef FACTOR_ADDERS_RELATIVE_POSE_FACTOR_ADDER_H_
#define FACTOR_ADDERS_RELATIVE_POSE_FACTOR_ADDER_H_

#include <factor_adders/relative_pose_factor_adder_params.h>
#include <factor_adders/single_measurement_based_factor_adder.h>
#include <localization_common/time.h>
#include <localization_measurements/relative_pose_with_covariance_measurement.h>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>

namespace factor_adders {
// Adds GTSAM Pose Between factors for relative pose measurements.
// Adds pose nodes using PoseNodeAdder at the same timestamps as the measurements.
template <class PoseNodeAdderType>
class RelativePoseFactorAdder
    : public SingleMeasurementBasedFactorAdder<localization_measurements::RelativePoseWithCovarianceMeasurement> {
 public:
  RelativePoseFactorAdder(const RelativePoseFactorAdderParams& params,
                          const std::shared_ptr<PoseNodeAdderType> node_adder);

 private:
  // Creates a pose between factor and pose nodes for the given measurement.
  int AddFactorsForSingleMeasurement(
    const localization_measurements::RelativePoseWithCovarianceMeasurement& measurement,
    gtsam::NonlinearFactorGraph& factors) final;

  bool CanAddFactor(const localization_measurements::RelativePoseWithCovarianceMeasurement& measurement) const final;

  std::shared_ptr<PoseNodeAdderType> node_adder_;
  RelativePoseFactorAdderParams params_;
};

template <class PoseNodeAdderType>
RelativePoseFactorAdder<PoseNodeAdderType>::RelativePoseFactorAdder(const RelativePoseFactorAdderParams& params,
                                                                    const std::shared_ptr<PoseNodeAdderType> node_adder)
    : SingleMeasurementBasedFactorAdder<localization_measurements::RelativePoseWithCovarianceMeasurement>(params),
      params_(params),
      node_adder_(node_adder) {}

template <class PoseNodeAdderType>
int RelativePoseFactorAdder<PoseNodeAdderType>::AddFactorsForSingleMeasurement(
  const localization_measurements::RelativePoseWithCovarianceMeasurement& measurement,
  gtsam::NonlinearFactorGraph& factors) {
  if (!node_adder_->AddNode(measurement.timestamp_a, factors) ||
      !node_adder_->AddNode(measurement.timestamp_b, factors)) {
    LogError("AddFactorsForSingleMeasurement: Failed to add nodes at respective times.");
    return 0;
  }

  const auto keys_a = node_adder_->Keys(measurement.timestamp_a);
  // First key is pose key
  const auto& pose_key_a = keys_a[0];
  const auto keys_b = node_adder_->Keys(measurement.timestamp_b);
  const auto& pose_key_b = keys_b[0];
  const auto relative_pose_noise =
    gtsam::noiseModel::Gaussian::Covariance(params_.covariance_scale * measurement.covariance);
  const gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr pose_between_factor(
    new gtsam::BetweenFactor<gtsam::Pose3>(pose_key_a, pose_key_b, measurement.relative_pose, relative_pose_noise));
  factors.push_back(pose_between_factor);
  return 1;
}

template <class PoseNodeAdderType>
bool RelativePoseFactorAdder<PoseNodeAdderType>::CanAddFactor(
  const localization_measurements::RelativePoseWithCovarianceMeasurement& measurement) const {
  return node_adder_->CanAddNode(measurement.timestamp_a) && node_adder_->CanAddNode(measurement.timestamp_b);
}
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_RELATIVE_POSE_FACTOR_ADDER_H_
