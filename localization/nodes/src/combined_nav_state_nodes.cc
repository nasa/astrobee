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

#include <nodes/combined_nav_state_nodes.h>

#include <gtsam/geometry/Pose3.h>

namespace nodes {
namespace lc = localization_common;

CombinedNavStateNodes::CombinedNavStateNodes(std::shared_ptr<Values> values)
    : TimestampedCombinedNodes<lc::CombinedNavState>(values) {}

gtsam::KeyVector CombinedNavStateNodes::AddNode(const lc::CombinedNavState& combined_nav_state) {
  const auto key_pose = values_->Add(combined_nav_state.pose());
  const auto key_velocity = values_->Add(combined_nav_state.velocity());
  const auto key_bias = values_->Add(combined_nav_state.bias());
  return {key_pose, key_velocity, key_bias};
}

boost::optional<lc::CombinedNavState> CombinedNavStateNodes::GetNode(const gtsam::KeyVector& keys,
                                                                     const lc::Time timestamp) const {
  const auto pose = Base::Value<gtsam::Pose3>(keys[0]);
  const auto velocity = Base::Value<gtsam::Velocity3>(keys[1]);
  const auto bias = Base::Value<gtsam::imuBias::ConstantBias>(keys[2]);
  if (!pose || !velocity || !bias) return boost::none;
  return lc::CombinedNavState(*pose, *velocity, *bias, timestamp);
}
}  // namespace nodes
