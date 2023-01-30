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
#ifndef NODE_UPDATERS_COMBINED_NAV_STATE_NODES_H_
#define NODE_UPDATERS_COMBINED_NAV_STATE_NODES_H_

#include <graph_optimizer/timestamped_combined_nodes.h>
#include <localization_common/combined_nav_state.h>

namespace graph_optimizer {
using CombinedNavStateNodes = TimestampedCombinedNodes<localization_common::CombinedNavState>;

// Specializations required for combined node
template <>
gtsam::KeyVector CombinedNavStateNodes::Add(const localization_common::CombinedNavState& combined_nav_state) {
  const auto key_pose = nodes_->Add(combined_nav_state.pose());
  const auto key_velocity = nodes_->Add(combined_nav_state.velocity());
  const auto key_bias = nodes_->Add(combined_nav_state.bias());
  return {key_pose, key_velocity, key_bias};
}

template <>
boost::optional<localization_common::CombinedNavState> CombinedNavStateNodes::Node(
  const gtsam::KeyVector& keys, const localization_common::Time timestamp) const {
  const auto pose = nodes_->Node<gtsam::Pose3>(keys[0]);
  const auto velocity = nodes_->Node<gtsam::Velocity3>(keys[1]);
  const auto bias = nodes_->Node<gtsam::imuBias::ConstantBias>(keys[2]);
  if (!pose || !velocity || !bias) return boost::none;
  return localization_common::CombinedNavState(*pose, *velocity, *bias, timestamp);
}
}  // namespace graph_optimizer

#endif  // NODE_UPDATERS_COMBINED_NAV_STATE_NODES_H_
