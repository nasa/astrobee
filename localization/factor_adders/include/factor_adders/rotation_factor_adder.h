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

#ifndef FACTOR_ADDERS_ROTATION_FACTOR_ADDER_H_
#define FACTOR_ADDERS_ROTATION_FACTOR_ADDER_H_

#include <factor_adders/feature_tracker.h>
#include <factor_adders/rotation_factor_adder_params.h>
#include <graph_optimizer/factor_adder.h>
#include <localization_measurements/feature_points_measurement.h>

#include <vector>

namespace factor_adders {
class RotationFactorAdder : public graph_optimizer::FactorAdder<localization_measurements::FeaturePointsMeasurement,
                                                                RotationFactorAdderParams> {
  using Base =
    graph_optimizer::FactorAdder<localization_measurements::FeaturePointsMeasurement, RotationFactorAdderParams>;

 public:
  RotationFactorAdder(const RotationFactorAdderParams& params, std::shared_ptr<const FeatureTracker> feature_tracker);

  std::vector<graph_optimizer::FactorsToAdd> AddFactors(
    const localization_measurements::FeaturePointsMeasurement& measurement) final;

 private:
  std::shared_ptr<const FeatureTracker> feature_tracker_;
};
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_ROTATION_FACTOR_ADDER_H_
