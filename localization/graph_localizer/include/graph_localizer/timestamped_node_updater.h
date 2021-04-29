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

#include <graph_localizer/graph_values.h>
#include <graph_localizer/node_updater_type.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace graph_localizer {
class TimestampedNodeUpdater {
 public:
  virtual ~TimestampedNodeUpdater() {}

  virtual bool Update(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors,
                      GraphValues& graph_values) = 0;

  virtual bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                           const boost::optional<gtsam::Marginals>& marginals, const double huber_k,
                           gtsam::NonlinearFactorGraph& factors, GraphValues& graph_values) = 0;

  virtual NodeUpdaterType type() const = 0;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_TIMESTAMPED_NODE_UPDATER_H_
