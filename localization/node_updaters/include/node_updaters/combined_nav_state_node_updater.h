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

#ifndef NODE_UPDATERS_COMBINED_NAV_STATE_NODE_UPDATER_H_
#define NODE_UPDATERS_COMBINED_NAV_STATE_NODE_UPDATER_H_

#include <node_updaters/combined_nav_state_node_update_model.h>
#include <node_updaters/combined_nav_state_nodes.h>
#include <node_updaters/timestamped_node_updater.h>

namespace node_updaters {
using CombinedNavStateNodeUpdater = TimestampedNodeUpdater<CombinedNavStateNodes, CombinedNavStateNodeUpdateModel>;

template <>
graph_optimizer::NodeUpdaterType CombinedNavStateNodeUpdater::type() const final {
  return go::NodeUpdaterType::CombinedNavState;
}
}  // namespace node_updaters

#endif  // NODE_UPDATERS_COMBINED_NAV_STATE_NODE_UPDATER_H_
