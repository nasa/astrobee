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

#ifndef GRAPH_OPTIMIZER_GRAPH_VALUES_H_
#define GRAPH_OPTIMIZER_GRAPH_VALUES_H_

#include <graph_optimizer/key_info.h>
#include <localization_common/logger.h>
#include <localization_common/time.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>
#include <boost/serialization/unordered_map.hpp>

#include <map>
#include <unordered_map>
#include <utility>

namespace graph_optimizer {
namespace sym = gtsam::symbol_shorthand;
class GraphValues {
 public:
  GraphValues(std::shared_ptr<gtsam::Values> values = std::shared_ptr<gtsam::Values>(new gtsam::Values()));

  // Returns the oldest time that will be in graph values once the window is slid using params
  virtual boost::optional<localization_common::Time> SlideWindowNewOldestTime() const = 0;

  virtual gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                                   const gtsam::NonlinearFactorGraph& graph) const = 0;

  virtual boost::optional<gtsam::Key> GetKey(KeyCreatorFunction key_creator_function,
                                             const localization_common::Time timestamp) const = 0;

  virtual boost::optional<localization_common::Time> OldestTimestamp() const = 0;

  virtual boost::optional<localization_common::Time> LatestTimestamp() const = 0;

  template <typename ValueType>
  boost::optional<ValueType> at(const gtsam::Key& key) const {
    if (!values_->exists(key)) {
      LogError("at: Key not present in values.");
      return boost::none;
    }

    return values_->at<ValueType>(key);
  }

  const gtsam::Values& values() const;

  gtsam::Values& values();

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(values_);
  }

  std::shared_ptr<gtsam::Values> values_;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_GRAPH_VALUES_H_
