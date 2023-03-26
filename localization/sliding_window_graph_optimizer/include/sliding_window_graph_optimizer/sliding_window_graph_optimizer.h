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

#ifndef SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_H_
#define SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_H_

#include <graph_optimizer/graph_optimizer.h>
#include <localization_common/time.h>
#include <node_adders/sliding_window_node_adder.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer_params.h>

#include <boost/serialization/serialization.hpp>

#include <vector>

namespace sliding_window_graph_optimizer {
// GraphOptimizer that performs sliding window optimization.
// Calculates the desired window size in duration and number of nodes for each sliding window node adder
// using the SlidingWindowNodeAdders in the graph.
// Removes old nodes outside of window size along with the factors that depend on them after optimization.
// Adds either prior factors using marginalized covariances for new start nodes or adds
// marginal factors consisting of linearized errors for each factor removed (set desired behavior using params).
// Add sliding window node adders using the AddSlidingWindowNodeAdder() function (instead of AddNodeAdder() function) to
// ensure SlideWindow() is called for the sliding window node adder.
class SlidingWindowGraphOptimizer : public graph_optimizer::GraphOptimizer {
 public:
  SlidingWindowGraphOptimizer(const SlidingWindowGraphOptimizerParams& params,
                              std::unique_ptr<optimizers::Optimizer> optimizer);

  virtual ~SlidingWindowGraphOptimizer();

  // Default constructor for serialization only
  SlidingWindowGraphOptimizer() {}

  // Add sliding window node adders with this to ensure SlideWindow() is called
  // for the node adder when sliding the window for the graph.
  // If this function is used, the node adder does not need to be added with
  // the GraphOptimizer::AddNodeAdder() function.
  void AddSlidingWindowNodeAdder(std::shared_ptr<node_adders::SlidingWindowNodeAdder> sliding_window_node_adder);

  // Optimizes the graph and slides the window.
  // See SlideWindow() function comments for more details on sliding the window.
  bool Update();

 private:
  // Calculates new start time for the graph using sliding window node adders and
  // removes nodes older than this start time.
  // Removes any factors depending on a removed node and optionally adds marginalized
  // factors containing linearized errors for each removed factors.
  // Optionally adds prior factors using marginalized covariances to new start nodes.
  bool SlideWindow(const gtsam::Marginals& marginals, const localization_common::Time end_time);

  // Creates marginal factors using linearized errors of removed factors.
  gtsam::NonlinearFactorGraph MarginalFactors(const gtsam::NonlinearFactorGraph& old_factors,
                                              const gtsam::KeyVector& old_keys,
                                              const gtsam::GaussianFactorGraph::Eliminate& eliminate_function) const;

  // New start time that will constrain the graph to the desired window size.
  // Calculated using the oldest of the desired start times for each sliding window node adder.
  // TODO(rsoussan): Use latest of the desired start times?
  boost::optional<localization_common::Time> SlideWindowNewStartTime() const;

  // Old keys for nodes. Calculated and accumulated for each sliding window node adder.
  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time) const;

  // Current end time of the sliding window. Calcualted using the latest end time
  // of the sliding window node adders.
  boost::optional<localization_common::Time> EndTime() const;

  // Add averagers and timers for logging
  void AddAveragersAndTimers();

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(GraphOptimizer);
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(sliding_window_node_adders_);
    ar& BOOST_SERIALIZATION_NVP(end_time_);
    ar& BOOST_SERIALIZATION_NVP(update_timer_);
  }

  SlidingWindowGraphOptimizerParams params_;
  std::vector<std::shared_ptr<node_adders::SlidingWindowNodeAdder>> sliding_window_node_adders_;
  boost::optional<localization_common::Time> end_time_;

  // Logging
  localization_common::Timer update_timer_ = localization_common::Timer("Update");
};
}  // namespace sliding_window_graph_optimizer

#endif  // SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_H_
