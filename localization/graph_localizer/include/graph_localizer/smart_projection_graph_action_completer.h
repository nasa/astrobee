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

#ifndef GRAPH_LOCALIZER_SMART_PROJECTION_GRAPH_ACTION_COMPLETER_H_
#define GRAPH_LOCALIZER_SMART_PROJECTION_GRAPH_ACTION_COMPLETER_H_

#include <graph_localizer/smart_projection_factor_adder_params.h>
#include <graph_optimizer/graph_action_completer.h>
#include <graph_optimizer/graph_values.h>

#include <gtsam/slam/SmartFactorParams.h>

namespace graph_localizer {
class SmartProjectionGraphActionCompleter : public graph_optimizer::GraphActionCompleter {
 public:
  SmartProjectionGraphActionCompleter(const SmartProjectionFactorAdderParams& params,
                                      std::shared_ptr<const graph_optimizer::GraphValues> graph_values);

  bool DoAction(graph_optimizer::FactorsToAdd& factors_to_add, gtsam::NonlinearFactorGraph& graph_factors) final;

  graph_optimizer::GraphActionCompleterType type() const final;

  const gtsam::SmartProjectionParams& smart_projection_params() const;

 private:
  void SplitSmartFactorsIfNeeded(const graph_optimizer::GraphValues& graph_values,
                                 graph_optimizer::FactorsToAdd& factors_to_add);

  SmartProjectionFactorAdderParams params_;
  std::shared_ptr<const graph_optimizer::GraphValues> graph_values_;
  gtsam::SmartProjectionParams smart_projection_params_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_SMART_PROJECTION_GRAPH_ACTION_COMPLETER_H_
