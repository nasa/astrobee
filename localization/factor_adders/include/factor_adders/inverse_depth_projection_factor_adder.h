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

#ifndef FACTOR_ADDERS_INVERSE_DEPTH_PROJECTION_FACTOR_ADDER_H_
#define FACTOR_ADDERS_INVERSE_DEPTH_PROJECTION_FACTOR_ADDER_H_

#include <factor_adders/feature_point_graph_values.h>
#include <factor_adders/inverse_depth_projection_factor_adder_params.h>
#include <graph_optimizer/factor_adder.h>
#include <localization_measurements/feature_points_measurement.h>
#include <vision_common/feature_tracker.h>

#include <vector>

namespace factor_adders {
class InverseDepthProjectionFactorAdder
    : public graph_optimizer::FactorAdder<localization_measurements::FeaturePointsMeasurement,
                                          InverseDepthProjectionFactorAdderParams> {
  using Base = graph_optimizer::FactorAdder<localization_measurements::FeaturePointsMeasurement,
                                            InverseDepthProjectionFactorAdderParams>;

 public:
  InverseDepthProjectionFactorAdder(const InverseDepthProjectionFactorAdderParams& params,
                                    std::shared_ptr<const vision_common::FeatureTracker> feature_tracker,
                                    std::shared_ptr<const FeaturePointGraphValues> feature_point_graph_values);

  std::vector<graph_optimizer::FactorsToAdd> AddFactors(
    const localization_measurements::FeaturePointsMeasurement& feature_points_measurement) final;

 private:
  std::shared_ptr<const vision_common::FeatureTracker> feature_tracker_;
  std::shared_ptr<const FeaturePointGraphValues> feature_point_graph_values_;
};
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_INVERSE_DEPTH_PROJECTION_FACTOR_ADDER_H_
