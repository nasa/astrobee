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
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer_params.h>

#include <gtsam/nonlinear/Marginals.h>

#include <boost/serialization/serialization.hpp>

namespace sliding_window_graph_optimizer {
// GraphOptimizer that performs sliding window optimization.
// Calculates the desired window size in duration and number of nodes for each node adder
// using NodeAdders in the graph.
// Removes old nodes outside of window size along with the factors that depend on them after optimization.
// Adds either prior factors using marginalized covariances for new start nodes or adds
// marginal factors consisting of linearized errors for each factor removed (set desired behavior using params).
// Add sliding window node adder using AddSlidingWindowNodeAdder() function (instead of AddNodeAdder() function) to
// ensure SlideWindow() is called for the node adder.
class SlidingWindowGraphOptimizer : public graph_optimizer::GraphOptimizer {
 public:
  explicit SlidingWindowGraphOptimizer(const SlidingWindowGraphOptimizerParams& params);

  virtual ~SlidingWindowGraphOptimizer();

  // Default constructor for serialization only
  SlidingWindowGraphOptimizer() {}

  // Add sliding window node adders with this to ensure SlideWindow() is called
  // for the node adder when sliding the window for the graph.
  // If this function is used, the node adder does not need to be added with
  // the GraphOptimizer::AddNodeAdder() function.
  void AddSlidingWindowNodeAdder();

  // Performs graph optimization and slides the window for each SlidingWindowNodeAdder.
  // Adds either marignal factors or prior factor using covariances to new start nodes.
  bool Update();

 private:
  // Removes nodes and factors outside of sliding window using
  // provided params.
  // Removes any factors depending on removed nodes.
  // Optionally adds marginalized factors encapsulating linearized error of removed factors.
  // Optionally adds priors using marginalized covariances for new start nodes.
  bool SlideWindow(const boost::optional<gtsam::Marginals>& marginals, const localization_common::Time end_time);

  // Creates marginal factors using linearized errors of removed factors.
  gtsam::NonlinearFactorGraph MarginalFactors(const gtsam::NonlinearFactorGraph& old_factors,
                                              const gtsam::KeyVector& old_keys,
                                              const gtsam::GaussianFactorGraph::Eliminate& eliminate_function) const;

  boost::optional<localization_common::Time> SlideWindowNewStartTime() const;

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time) const;

  boost::optional<localization_common::Time> EndTime() const;

  // Solve for marginals, required for calculating covariances
  void CalculateMarginals();

  // Order nodes in the graph for keys that will be marginalized.
  // Allows for more efficient optimization.
  void SetOrdering();

  // Set solver used for calculating marginals
  void SetMaringalsFactorization();

  // Add averagers and timers for logging
  void AddAveragersAndTimers();

  // Returns if an Update() call has been performed.
  bool UpdateOccured() const;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(GraphOptimizer);
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(marginals_);
    ar& BOOST_SERIALIZATION_NVP(marginals_factorization_);
    ar& BOOST_SERIALIZATION_NVP(end_time_);
  }

  SlidingWindowGraphOptimizerParams params_;
  boost::optional<gtsam::Marginals> marginals_;
  gtsam::Marginals::Factorization marginals_factorization_;
  boost::optional<localization_common::Time> end_time_;

  // Logging
  localization_common::Timer update_timer_ = localization_common::Timer("Update");
};
}  // namespace sliding_window_graph_optimizer

#endif  // SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_H_
