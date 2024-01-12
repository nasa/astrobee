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
#ifndef NODES_TIMESTAMPED_NODES_H_
#define NODES_TIMESTAMPED_NODES_H_

#include <nodes/timestamped_combined_nodes.h>

namespace nodes {
template <typename NodeType>
// Container for timestamped nodes with a single value per node.
// Stores the templated NodeType at various timestamps for use with a graph optimizer.
// Child class of TimestampedCombinedNodes which handles multiple values per node/timestamp.
class TimestampedNodes : public TimestampedCombinedNodes<NodeType> {
  using Base = TimestampedCombinedNodes<NodeType>;

 public:
  explicit TimestampedNodes(std::shared_ptr<Values> values);

  // For serialization only
  TimestampedNodes() = default;

 private:
  gtsam::KeyVector AddNode(const NodeType& node) final;

  boost::optional<NodeType> GetNode(const gtsam::KeyVector& keys,
                                    const localization_common::Time timestamp) const final;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};

// Implementation
template <typename NodeType>
TimestampedNodes<NodeType>::TimestampedNodes(std::shared_ptr<Values> values)
    : TimestampedCombinedNodes<NodeType>(values) {}

template <typename NodeType>
gtsam::KeyVector TimestampedNodes<NodeType>::AddNode(const NodeType& node) {
  const auto key = this->values_->Add(node);
  return {key};
}

template <typename NodeType>
boost::optional<NodeType> TimestampedNodes<NodeType>::GetNode(const gtsam::KeyVector& keys,
                                                              const localization_common::Time timestamp) const {
  // Assumes keys only has a single key since using non-combined type
  return Base::template Value<NodeType>(keys[0]);
}
}  // namespace nodes

#endif  // NODES_TIMESTAMPED_NODES_H_
