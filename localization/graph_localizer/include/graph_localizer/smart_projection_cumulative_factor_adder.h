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

#ifndef GRAPH_LOCALIZER_SMART_PROJECTION_CUMULATIVE_FACTOR_ADDER_H_
#define GRAPH_LOCALIZER_SMART_PROJECTION_CUMULATIVE_FACTOR_ADDER_H_

#include <graph_localizer/cumulative_factor_adder.h>
#include <graph_localizer/feature_tracker.h>
#include <graph_localizer/smart_projection_factor_adder_params.h>

#include <gtsam/slam/SmartFactorParams.h>

#include <vector>

namespace graph_localizer {
class SmartProjectionCumulativeFactorAdder : public CumulativeFactorAdder<SmartProjectionFactorAdderParams> {
  using Base = CumulativeFactorAdder<SmartProjectionFactorAdderParams>;

 public:
  SmartProjectionCumulativeFactorAdder(const SmartProjectionFactorAdderParams& params,
                                       std::shared_ptr<const FeatureTracker> feature_tracker);

  std::vector<FactorsToAdd> AddFactors() final;

 private:
  void AddSmartFactor(const std::vector<lm::FeaturePoint>& feature_track_points,
                      FactorsToAdd& smart_factors_to_add) const;

  std::shared_ptr<const FeatureTracker> feature_tracker_;
  gtsam::SmartProjectionParams smart_projection_params_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_SMART_PROJECTION_CUMULATIVE_FACTOR_ADDER_H_
