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

#ifndef GRAPH_LOCALIZER_TIMESTAMPED_NODE_UPDATER_H_
#define GRAPH_LOCALIZER_TIMESTAMPED_NODE_UPDATER_H_

#include <graph_localizer/factor_to_add.h>
#include <graph_localizer/graph_values.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace graph_localizer {
template <typename NodeType, typename NoiseType, typename Params>
class TimestampedNodeUpdater {
 public:
  explicit TimestampedNodeUpdater(const Params& params) : params_(params) {}

  virtual ~TimestampedNodeUpdater() {}

  virtual void AddInitialValuesAndPriors(const NodeType& node, const NoiseType& noise,
                                         gtsam::NonlinearFactorGraph& graph, GraphValues& graph_values) = 0;

  virtual void AddPriors(const NodeType& node, const NoiseType& noise, const GraphValues& graph_values,
                         gtsam::NonlinearFactorGraph& factors) = 0;

  virtual void AddFactors(const FactorToAdd& measurement, gtsam::NonlinearFactorGraph& graph,
                          GraphValues& graph_values) = 0;

  virtual void SlideWindow(const localization_common::Timestamp oldest_allowed_timestamp,
                           gtsam::NonlinearFactorGraph& factors, GraphValues& graph_values) = 0;

 private:
  Params params_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_TIMESTAMPED_NODE_UPDATER_H_
