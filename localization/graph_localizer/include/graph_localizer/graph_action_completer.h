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

#ifndef GRAPH_LOCALIZER_GRAPH_ACTION_COMPLETER_H_
#define GRAPH_LOCALIZER_GRAPH_ACTION_COMPLETER_H_

#include <graph_localizer/graph_action_completer_type.h>
#include <graph_localizer/graph_values.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace graph_localizer {
class GraphActionCompleter {
 public:
  virtual ~GraphActionCompleter() {}
  virtual bool DoAction(FactorsToAdd& factors_to_add, gtsam::NonlinearFactorGraph& graph_factors,
                        GraphValues& graph_values) = 0;
  virtual GraphActionCompleterType type() const = 0;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_ACTION_COMPLETER_H_
