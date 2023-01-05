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

#ifndef GRAPH_OPTIMIZER_NODES_H_
#define GRAPH_OPTIMIZER_NODES_H_

#include <localization_common/logger.h>

#include <gtsam/base/serialization.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>

namespace graph_optimizer {
class Nodes {
 public:
  explicit Nodes(std::shared_ptr<gtsam::Values> values = std::make_shared<gtsam::Values>());

  template <typename NodeType>
  boost::optional<NodeType> Node(const gtsam::Key& key) const;

  // Returns key for newly added node
  template <typename NodeType>
  gtsam::Key Add(const NodeType& node);

  bool Remove(const gtsam::Key& key);

  bool Contains(const gtsam::Key& key) const;

  size_t size() const;

  const gtsam::Values& values() const { return *values_; }

  gtsam::Values& values() { return *values_; }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/);

  std::shared_ptr<gtsam::Values> values_;
  gtsam::Key latest_key_;
};

// Implementation
template <typename NodeType>
boost::optional<NodeType> Nodes::Node(const gtsam::Key& key) const {
  try {
    return values_->at<NodeType>(key);
  } catch (...) {
    return boost::none;
  }
}

template <typename NodeType>
gtsam::Key Nodes::Add(const NodeType& node) {
  // Since latest_key_ is always incremented when a new key is added,
  // we don't need to worry about it already being in values_.
  values_->insert(++latest_key_, node);
  return latest_key_;
}

template <class ARCHIVE>
void Nodes::serialize(ARCHIVE& ar, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(values_);
  ar& BOOST_SERIALIZATION_NVP(latest_key_);
}
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_NODES_H_
