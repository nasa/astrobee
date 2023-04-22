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

#ifndef TUTORIAL_EXAMPLES_ABSOLUTE_POSE_FACTOR_ADDER_H_
#define TUTORIAL_EXAMPLES_ABSOLUTE_POSE_FACTOR_ADDER_H_

#include <factor_adders/single_measurement_based_factor_adder.h>
#include <localization_common/time.h>
#include <localization_measurements/timestamped_pose_with_covariance.h>
#include <tutorial_examples/relative_pose_node_adder.h>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>

namespace tutorial_examples {
using AbsolutePoseFactorAdderParams = factor_adders::FactorAdderParams;

class AbsolutePoseFactorAdder : public factor_adders::SingleMeasurementBasedFactorAdder<
                                  localization_measurements::PoseWithCovarianceMeasurement> {
 public:
  AbsolutePoseFactorAdder(const AbsolutePoseFactorAdderParams& params,
                          const std::shared_ptr<RelativePoseNodeAdder> node_adder)
      : factor_adders::SingleMeasurementBasedFactorAdder<localization_measurements::PoseWithCovarianceMeasurement>(
          params),
        params_(params),
        node_adder_(node_adder) {}

 private:
  int AddFactorsForSingleMeasurement(const localization_measurements::PoseWithCovarianceMeasurement& measurement,
                                     gtsam::NonlinearFactorGraph& factors) final {
    node_adder_->AddNode(measurement.time, factors);
    const auto keys = node_adder_->Keys(measurement.time);
    // First key is pose key
    const auto& pose_key = keys[0];
    const auto pose_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
    const gtsam::PriorFactor<gtsam::Pose3>::shared_ptr pose_prior_factor(
      new gtsam::PriorFactor<gtsam::Pose3>(pose_key, localization_common::GtPose(measurement.pose), pose_noise));
    factors.push_back(pose_prior_factor);
  }

  bool CanAddFactor(const localization_common::Time time) const final { return node_adder_->CanAddNode(time); }

  std::shared_ptr<RelativePoseNodeAdder> node_adder_;
  AbsolutePoseFactorAdderParams params_;
};
}  // namespace tutorial_examples

#endif  // TUTORIAL_EXAMPLES_ABSOLUTE_POSE_FACTOR_ADDER_H_
