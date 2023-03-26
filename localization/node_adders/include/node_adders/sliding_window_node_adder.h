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

#ifndef NODE_ADDERS_SLIDING_WINDOW_NODE_ADDER_H_
#define NODE_ADDERS_SLIDING_WINDOW_NODE_ADDER_H_

#include <localization_common/time.h>
#include <node_adders/node_adder.h>

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace node_adders {
// Base class node adder that maintains a sliding window of nodes.
class SlidingWindowNodeAdder : public NodeAdder {
 public:
  virtual ~SlidingWindowNodeAdder() {}

  // Slides the window, removes nodes older than oldest allowed time.
  // Adds priors to the oldest remaining nodes using their marginalized covariances
  // and removes old priors containing any key in old keys if param use_priors is true.
  // Note: Old factor removal (other than starting priors) is handled in the graph optimizer.
  // The oldest allowed timestamp is also determing by the graph optimizer based on
  // the ideal oldest allowed timestamps of each node adder used in the graph.
  virtual bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                           const boost::optional<const gtsam::Marginals&>& marginals, const gtsam::KeyVector& old_keys,
                           const double huber_k, gtsam::NonlinearFactorGraph& factors) = 0;

  // Returns the oldest node time that should remain after SlideWindow is called.
  virtual boost::optional<localization_common::Time> SlideWindowNewStartTime() const = 0;

  // Returns old node keys older than the oldest_allowed_time.
  virtual gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                                   const gtsam::NonlinearFactorGraph& graph) const = 0;

  // Starting (oldest) timestamp of nodes in the node adder
  virtual boost::optional<localization_common::Time> StartTime() const = 0;

  // End (latest) timestamp of nodes in the node adder
  virtual boost::optional<localization_common::Time> EndTime() const = 0;
};
}  // namespace node_adders

#endif  // NODE_ADDERS_SLIDING_WINDOW_NODE_ADDER_H_
