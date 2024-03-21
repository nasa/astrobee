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
#ifndef NODES_COMBINED_NAV_STATE_NODES_H_
#define NODES_COMBINED_NAV_STATE_NODES_H_

#include <localization_common/combined_nav_state.h>
#include <nodes/timestamped_combined_nodes.h>

namespace nodes {
class CombinedNavStateNodes : public TimestampedCombinedNodes<localization_common::CombinedNavState> {
  using Base = TimestampedCombinedNodes<localization_common::CombinedNavState>;

 public:
  explicit CombinedNavStateNodes(std::shared_ptr<Values> values);

  // For serialization only
  CombinedNavStateNodes() = default;

 private:
  gtsam::KeyVector AddNode(const localization_common::CombinedNavState& node) final;

  boost::optional<localization_common::CombinedNavState> GetNode(const gtsam::KeyVector& keys,
                                                                 const localization_common::Time timestamp) const final;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};
}  // namespace nodes

#endif  // NODES_COMBINED_NAV_STATE_NODES_H_
