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

#ifndef GRAPH_OPTIMIZER_NODE_UPDATER_H_
#define GRAPH_OPTIMIZER_NODE_UPDATER_H_

#include <graph_optimizer/key_info.h>
#include <graph_optimizer/node_updater_type.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace graph_optimizer {
class NodeUpdater {
 public:
  virtual ~NodeUpdater() {}

  virtual bool Update(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) = 0;

  virtual bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                           const boost::optional<gtsam::Marginals>& marginals, const gtsam::KeyVector& old_keys,
                           const double huber_k, gtsam::NonlinearFactorGraph& factors) = 0;

  // Returns the oldest time that will be in graph values once the window is slid using params
  virtual boost::optional<localization_common::Time> SlideWindowNewOldestTime() const = 0;

  // TODO(rsoussan): consolidate these with graph values base?
  virtual gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                                   const gtsam::NonlinearFactorGraph& graph) const = 0;

  virtual boost::optional<gtsam::Key> GetKey(KeyCreatorFunction key_creator_function,
                                             const localization_common::Time timestamp) const = 0;

  virtual boost::optional<localization_common::Time> OldestTimestamp() const = 0;

  virtual boost::optional<localization_common::Time> LatestTimestamp() const = 0;

  virtual NodeUpdaterType type() const = 0;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_NODE_UPDATER_H_
