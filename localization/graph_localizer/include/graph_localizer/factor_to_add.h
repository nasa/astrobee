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

#ifndef GRAPH_LOCALIZER_FACTOR_TO_ADD_H_
#define GRAPH_LOCALIZER_FACTOR_TO_ADD_H_

#include <graph_localizer/graph_action.h>
#include <graph_localizer/key_info.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <vector>

namespace graph_localizer {
struct FactorToAdd {
  FactorToAdd(const KeyInfos& key_infos, boost::shared_ptr<gtsam::NonlinearFactor> factor)
      : factor(factor), key_infos(key_infos) {}

  boost::shared_ptr<gtsam::NonlinearFactor> factor;
  KeyInfos key_infos;
};

class FactorsToAdd {
 public:
  FactorsToAdd(const localization_common::Time timestamp, const std::vector<FactorToAdd>& factors_to_add,
               const GraphAction graph_action = GraphAction::kNone)
      : timestamp_(timestamp), factors_to_add_(factors_to_add), graph_action_(graph_action) {}

  explicit FactorsToAdd(const GraphAction graph_action = GraphAction::kNone) : graph_action_(graph_action) {}

  void reserve(const int size) { factors_to_add_.reserve(size); }
  void push_back(FactorToAdd&& factor_to_add) { factors_to_add_.emplace_back(std::move(factor_to_add)); }  // NOLINT
  void push_back(const FactorToAdd& factor_to_add) { factors_to_add_.push_back(factor_to_add); }
  void SetTimestamp(const localization_common::Time timestamp) { timestamp_ = timestamp; }

  localization_common::Time timestamp() const { return timestamp_; }
  // TODO(rsoussan): rename this?
  const std::vector<FactorToAdd>& Get() const { return factors_to_add_; }
  std::vector<FactorToAdd>& Get() { return factors_to_add_; }
  GraphAction graph_action() const { return graph_action_; }

 private:
  // Timestamp used to sort factors when adding to graph.
  localization_common::Time timestamp_;
  std::vector<FactorToAdd> factors_to_add_;
  GraphAction graph_action_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_FACTOR_TO_ADD_H_
