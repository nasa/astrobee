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

#ifndef GRAPH_LOCALIZER_PROJECTION_GRAPH_ACTION_COMPLETER_H_
#define GRAPH_LOCALIZER_PROJECTION_GRAPH_ACTION_COMPLETER_H_

#include <graph_localizer/projection_factor_adder_params.h>
#include <graph_optimizer/graph_action_completer.h>
#include <graph_optimizer/graph_values.h>

#include <gtsam/geometry/triangulation.h>

namespace graph_localizer {
class ProjectionGraphActionCompleter : public graph_optimizer::GraphActionCompleter {
 public:
  ProjectionGraphActionCompleter(const ProjectionFactorAdderParams& params,
                                 std::shared_ptr<const FeatureTracker> feature_tracker,
                                 std::shared_ptr<const graph_optimizer::GraphValues> graph_values);

  bool DoAction(graph_optimizer::FactorsToAdd& factors_to_add, gtsam::NonlinearFactorGraph& graph_factors) final;

  graph_optimizer::GraphActionCompleterType type() const final;

 private:
  bool TriangulateNewPoint(graph_optimizer::FactorsToAdd& factors_to_add, gtsam::NonlinearFactorGraph& graph_factors,
                           graph_optimizer::GraphValues& graph_values);

  ProjectionFactorAdderParams params_;
  std::shared_ptr<const graph_optimizer::GraphValues> graph_values_;
  gtsam::TriangulationParameters projection_triangulation_params_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_PROJECTION_GRAPH_ACTION_COMPLETER_H_
