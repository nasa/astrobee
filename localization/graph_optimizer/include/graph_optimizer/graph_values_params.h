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
#ifndef GRAPH_OPTIMIZER_GRAPH_VALUES_PARAMS_H_
#define GRAPH_OPTIMIZER_GRAPH_VALUES_PARAMS_H_

#include <gtsam/geometry/Pose3.h>

namespace graph_optimizer {
struct GraphValuesParams {
  // Only kept if there are at least min_num_states and not more than max_num_states
  double ideal_duration;
  int min_num_states;
  int max_num_states;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_GRAPH_VALUES_PARAMS_H_
