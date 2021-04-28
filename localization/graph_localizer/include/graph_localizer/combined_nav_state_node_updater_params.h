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
#ifndef GRAPH_LOCALIZER_COMBINED_NAV_STATE_NODE_UPDATER_PARAMS_H_
#define GRAPH_LOCALIZER_COMBINED_NAV_STATE_NODE_UPDATER_PARAMS_H_

#include <localization_common/combined_nav_state.h>

namespace graph_localizer {
struct CombinedNavStateNodeUpdaterParams {
  localization_common::CombinedNavState global_N_body_start;
  localization_common::CombinedNavStateNoise global_N_body_start_noise;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_COMBINED_NAV_STATE_NODE_UPDATER_PARAMS_H_
