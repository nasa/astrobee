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

#ifndef GRAPH_LOCALIZER_PROJECTION_FACTOR_ADDER_H_
#define GRAPH_LOCALIZER_PROJECTION_FACTOR_ADDER_H_

#include <graph_localizer/feature_tracker.h>
#include <graph_localizer/projection_factor_adder_params.h>
#include <graph_optimizer/factor_adder.h>
#include <graph_optimizer/graph_action_completer.h>
#include <graph_optimizer/graph_values.h>
#include <localization_measurements/feature_points_measurement.h>

#include <gtsam/geometry/triangulation.h>

#include <vector>

namespace graph_localizer {
class ProjectionFactorAdder : public graph_optimizer::FactorAdder<localization_measurements::FeaturePointsMeasurement,
                                                                  ProjectionFactorAdderParams>,
                              public graph_optimizer::GraphActionCompleter {
  using Base =
    graph_optimizer::FactorAdder<localization_measurements::FeaturePointsMeasurement, ProjectionFactorAdderParams>;

 public:
  ProjectionFactorAdder(const ProjectionFactorAdderParams& params,
                        std::shared_ptr<const FeatureTracker> feature_tracker,
                        std::shared_ptr<const graph_optimizer::GraphValues> graph_values);

  std::vector<graph_optimizer::FactorsToAdd> AddFactors(
    const localization_measurements::FeaturePointsMeasurement& feature_points_measurement) final;

  bool DoAction(graph_optimizer::FactorsToAdd& factors_to_add, gtsam::NonlinearFactorGraph& graph_factors,
                graph_optimizer::GraphValues& graph_values) final;

  graph_optimizer::GraphActionCompleterType type() const final;

 private:
  bool TriangulateNewPoint(graph_optimizer::FactorsToAdd& factors_to_add, gtsam::NonlinearFactorGraph& graph_factors,
                           graph_optimizer::GraphValues& graph_values);

  std::shared_ptr<const FeatureTracker> feature_tracker_;
  std::shared_ptr<const graph_optimizer::GraphValues> graph_values_;
  gtsam::TriangulationParameters projection_triangulation_params_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_PROJECTION_FACTOR_ADDER_H_
