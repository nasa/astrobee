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

#ifndef FACTOR_ADDERS_SMART_PROJECTION_CUMULATIVE_FACTOR_ADDER_H_
#define FACTOR_ADDERS_SMART_PROJECTION_CUMULATIVE_FACTOR_ADDER_H_

#include <vision_common/feature_tracker.h>
#include <factor_adders/smart_projection_factor_adder_params.h>
#include <graph_optimizer/cumulative_factor_adder.h>

#include <gtsam/slam/SmartFactorParams.h>

#include <unordered_map>
#include <vector>

namespace factor_adders {
class SmartProjectionCumulativeFactorAdder
    : public graph_optimizer::CumulativeFactorAdder<SmartProjectionFactorAdderParams> {
  using Base = graph_optimizer::CumulativeFactorAdder<SmartProjectionFactorAdderParams>;

 public:
  SmartProjectionCumulativeFactorAdder(const SmartProjectionFactorAdderParams& params,
                                       std::shared_ptr<const vision_common::FeatureTracker> feature_tracker);

  std::vector<graph_optimizer::FactorsToAdd> AddFactors() final;
  void AddFactors(const vision_common::FeatureTrackLengthMap& feature_tracks, const int spacing,
                  const double feature_track_min_separation, graph_optimizer::FactorsToAdd& smart_factors_to_add,
                  std::unordered_map<vision_common::FeatureId, vision_common::FeaturePoint>& added_points);
  void AddAllowedFactors(
    const vision_common::FeatureTrackLengthMap& feature_tracks, const double feature_track_min_separation,
    graph_optimizer::FactorsToAdd& smart_factors_to_add,
    std::unordered_map<vision_common::FeatureId, vision_common::FeaturePoint>& added_points);

  const gtsam::SmartProjectionParams& smart_projection_params() const;

 private:
  void AddSmartFactor(const std::vector<vision_common::FeaturePoint>& feature_track_points,
                      graph_optimizer::FactorsToAdd& smart_factors_to_add) const;

  bool TooClose(const std::unordered_map<vision_common::FeatureId, vision_common::FeaturePoint>& added_points,
                const vision_common::FeaturePoint& point,
                const double feature_track_min_separation) const;

  std::shared_ptr<const vision_common::FeatureTracker> feature_tracker_;
  gtsam::SmartProjectionParams smart_projection_params_;
};
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_SMART_PROJECTION_CUMULATIVE_FACTOR_ADDER_H_