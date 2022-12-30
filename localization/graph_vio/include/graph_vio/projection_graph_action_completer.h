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

#ifndef GRAPH_VIO_PROJECTION_GRAPH_ACTION_COMPLETER_H_
#define GRAPH_VIO_PROJECTION_GRAPH_ACTION_COMPLETER_H_

#include <graph_vio/combined_nav_state_graph_values.h>
#include <graph_vio/projection_factor_adder_params.h>
#include <graph_vio/feature_point_graph_values.h>
#include <graph_optimizer/graph_action_completer.h>

#include <gtsam/geometry/triangulation.h>

namespace graph_vio {
class ProjectionGraphActionCompleter : public graph_optimizer::GraphActionCompleter {
 public:
  ProjectionGraphActionCompleter(const ProjectionFactorAdderParams& params,
                                 std::shared_ptr<const CombinedNavStateGraphValues> graph_values,
                                 std::shared_ptr<FeaturePointGraphValues> feature_point_graph_values);

  bool DoAction(graph_optimizer::FactorsToAdd& factors_to_add, gtsam::NonlinearFactorGraph& graph_factors) final;

  graph_optimizer::GraphActionCompleterType type() const final;

 private:
  bool TriangulateNewPoint(graph_optimizer::FactorsToAdd& factors_to_add, gtsam::NonlinearFactorGraph& graph_factors);

  ProjectionFactorAdderParams params_;
  std::shared_ptr<const CombinedNavStateGraphValues> graph_values_;
  std::shared_ptr<FeaturePointGraphValues> feature_point_graph_values_;
  gtsam::TriangulationParameters projection_triangulation_params_;
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_PROJECTION_GRAPH_ACTION_COMPLETER_H_
