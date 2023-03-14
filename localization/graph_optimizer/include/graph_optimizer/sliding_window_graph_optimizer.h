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

#ifndef GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_H_
#define GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_H_

#include <graph_optimizer/graph_optimizer.h>
#include <graph_optimizer/sliding_window_graph_optimizer_params.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/Marginals.h>

#include <boost/serialization/serialization.hpp>

namespace graph_optimizer {
class SlidingWindowGraphOptimizer : public GraphOptimizer {
 public:
  explicit SlidingWindowGraphOptimizer(const SlidingWindowGraphOptimizerParams& params);

  // Default constructor/destructor for serialization only
  SlidingWindowGraphOptimizer() {}
  ~SlidingWindowGraphOptimizer();

  // Performs graph optimization and slides the window.
  bool Update() override;

  // Checks whether a measurement is too old to be inserted into graph
  // virtual bool MeasurementRecentEnough(const localization_common::Time timestamp) const;

  const SlidingWindowGraphOptimizerParams& params() const;

  const boost::optional<gtsam::Marginals>& marginals() const;

 private:
  // Creates marginal factors using linearized error of removed factors.
  gtsam::NonlinearFactorGraph MarginalFactors(const gtsam::NonlinearFactorGraph& old_factors,
                                              const gtsam::KeyVector& old_keys,
                                              const gtsam::GaussianFactorGraph::Eliminate& eliminate_function) const;

  // Removes nodes and factors outside of sliding window using
  // provided params.
  // Removes any factors depending on removed nodes.
  // Optionally adds marginalized factors encapsulating linearized error of removed factors.
  // Optionally adds priors using marginalized covariances for new start nodes.
  bool SlideWindow(const boost::optional<gtsam::Marginals>& marginals, const localization_common::Time last_end_time);

  boost::optional<localization_common::Time> SlideWindowNewOldestTime() const;

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time) const;

  boost::optional<localization_common::Time> OldestTimestamp() const;

  boost::optional<localization_common::Time> LatestTimestamp() const;

  // Order nodes in the graph for keys that will be marginalized.
  // Allows for more efficient optimization.
  void SetOrdering();

  // Solve for marginals, required for calculating covariances
  void CalculateMarginals();

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(GraphOptimizer);
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(marginals_);
    ar& BOOST_SERIALIZATION_NVP(marginals_factorization_);
    ar& BOOST_SERIALIZATION_NVP(last_end_time_);
  }

  SlidingWindowGraphOptimizerParams params_;
  boost::optional<gtsam::Marginals> marginals_;
  gtsam::Marginals::Factorization marginals_factorization_;
  boost::optional<localization_common::Time> last_end_time_;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_H_
