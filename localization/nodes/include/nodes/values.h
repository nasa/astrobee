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

#ifndef NODES_VALUES_H_
#define NODES_VALUES_H_

#include <localization_common/logger.h>

#include <gtsam/base/serialization.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>

namespace nodes {
// Wrapper class around GTSAM values.
// Use one instance of this per graph and pass the same Values shared_pointer
// to Nodes classes used in the same graph.
class Values {
 public:
  explicit Values(std::shared_ptr<gtsam::Values> values = std::make_shared<gtsam::Values>());

  template <typename ValueType>
  boost::optional<ValueType> Value(const gtsam::Key& key) const;

  // Returns key for newly added value
  template <typename ValueType>
  gtsam::Key Add(const ValueType& value);

  bool Remove(const gtsam::Key& key);

  // TODO(rsoussan): Test this
  bool Remove(const gtsam::KeyVector& keys);

  bool Contains(const gtsam::Key& key) const;

  size_t size() const;

  const gtsam::Values& gtsam_values() const { return *values_; }

  gtsam::Values& gtsam_values() { return *values_; }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/);

  std::shared_ptr<gtsam::Values> values_;
  gtsam::Key latest_key_;
};

// Implementation
template <typename ValueType>
boost::optional<ValueType> Values::Value(const gtsam::Key& key) const {
  try {
    return values_->at<ValueType>(key);
  } catch (...) {
    return boost::none;
  }
}

template <typename ValueType>
gtsam::Key Values::Add(const ValueType& value) {
  // Since latest_key_ is always incremented when a new key is added,
  // we don't need to worry about it already being in values_.
  values_->insert(++latest_key_, value);
  return latest_key_;
}

template <class ARCHIVE>
void Values::serialize(ARCHIVE& ar, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(values_);
  ar& BOOST_SERIALIZATION_NVP(latest_key_);
}
}  // namespace nodes

#endif  // NODES_VALUES_H_
