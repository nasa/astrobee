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
#ifndef FACTOR_ADDERS_POSE_FACTOR_ADDER_H_
#define FACTOR_ADDERS_POSE_FACTOR_ADDER_H_

#include <factor_adders/single_measurement_based_factor_adder.h>
#include <localization_common/time.h>
#include <localization_measurements/pose_with_covariance_measurement.h>
#include <node_adders/pose_node_adder.h>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>

namespace factor_adders {
using PoseFactorAdderParams = FactorAdderParams;

// Adds GTSAM Pose Prior factors for absolute pose measurements.
// Adds pose nodes using PoseNodeAdder at the same timestamps as the measurements.
template <class PoseNodeAdderType>
class PoseFactorAdder
    : public SingleMeasurementBasedFactorAdder<localization_measurements::PoseWithCovarianceMeasurement> {
 public:
  PoseFactorAdder(const PoseFactorAdderParams& params, const std::shared_ptr<PoseNodeAdderType> node_adder);

 private:
  // Creates a pose factor and pose node for the given measurement.
  int AddFactorsForSingleMeasurement(const localization_measurements::PoseWithCovarianceMeasurement& measurement,
                                     gtsam::NonlinearFactorGraph& factors) final;

  // Able to add a factor if the node adder can create a node for the provided measurement.
  bool CanAddFactor(const localization_measurements::PoseWithCovarianceMeasurement& measurement) const final;

  std::shared_ptr<PoseNodeAdderType> node_adder_;
  PoseFactorAdderParams params_;
};

template <class PoseNodeAdderType>
PoseFactorAdder<PoseNodeAdderType>::PoseFactorAdder(const PoseFactorAdderParams& params,
                                                    const std::shared_ptr<PoseNodeAdderType> node_adder)
    : SingleMeasurementBasedFactorAdder<localization_measurements::PoseWithCovarianceMeasurement>(params),
      params_(params),
      node_adder_(node_adder) {}

template <class PoseNodeAdderType>
int PoseFactorAdder<PoseNodeAdderType>::AddFactorsForSingleMeasurement(
  const localization_measurements::PoseWithCovarianceMeasurement& measurement, gtsam::NonlinearFactorGraph& factors) {
  node_adder_->AddNode(measurement.timestamp, factors);
  const auto keys = node_adder_->Keys(measurement.timestamp);
  // First key is pose key
  const auto& pose_key = keys[0];
  const auto pose_noise = gtsam::noiseModel::Gaussian::Covariance(measurement.covariance);
  const gtsam::PriorFactor<gtsam::Pose3>::shared_ptr pose_prior_factor(
    new gtsam::PriorFactor<gtsam::Pose3>(pose_key, measurement.pose, pose_noise));
  factors.push_back(pose_prior_factor);
  return 1;
}

template <class PoseNodeAdderType>
bool PoseFactorAdder<PoseNodeAdderType>::CanAddFactor(
  const localization_measurements::PoseWithCovarianceMeasurement& measurement) const {
  return node_adder_->CanAddNode(measurement.timestamp);
}
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_POSE_FACTOR_ADDER_H_
