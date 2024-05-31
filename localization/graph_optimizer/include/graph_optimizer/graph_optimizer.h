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

#ifndef GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_H_
#define GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_H_

#include <factor_adders/factor_adder.h>
#include <graph_optimizer/graph_optimizer_params.h>
#include <optimizers/optimizer.h>
#include <node_adders/node_adder.h>
#include <nodes/values.h>
#include <localization_common/stats_logger.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/serialization/serialization.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace graph_optimizer {
// Time-based factor graph optimizer that uses FactorAdders to generate and add factors for a given time range to the
// graph and NodeAdders to provide corresponding nodes for these factors at required timestamps. Generally FactorAdders
// are measurement-based and generate factors for a certain type of measurement at given timestamps. NodeAdders are also
// typically measurement-based and create linked nodes using relative factors that span the graph. As an example, a
// Visual-Inertial Odometry (VIO) graph may contain an image-based FactorAdder that generates visual-odometry factors
// using image measurements. A VIO graph will solve for Pose/Velocity/IMU Bias (PVB) values at different timestamps, so
// it will also contain a PVB NodeAdder that creates timestamped PVB nodes for the visual-odometry factors and links
// these nodes using IMU measurements. See the FactorAdder and NodeAdder packages for more information and different
// types of FactorAdders and NodeAdders. Maintains a set of values used for optimization. All NodeAdders added to the
// GraphOptimizer should be constructed using the GraphOptimizer's values (accessable with the values() member
// function). Acts as a base class for the SlidingWindowGraphOptimizer.
class GraphOptimizer {
 public:
  // Construct GraphOptimizer with provided optimizer.
  GraphOptimizer(const GraphOptimizerParams& params, std::unique_ptr<optimizers::Optimizer> optimizer);

  // Default constructor for serialization only
  GraphOptimizer() {}

  virtual ~GraphOptimizer() = default;

  // Adds node adder used for graph optimization.
  // Initializes the nodes and priors for the node adder.
  void AddNodeAdder(std::shared_ptr<node_adders::NodeAdder> node_adder);

  // Adds factor adder used for graph optimization.
  void AddFactorAdder(std::shared_ptr<factor_adders::FactorAdder> factor_adder);

  // Adds factors for each factor in valid time range to the graph.
  // Since factor adders are linked to the node adder they use, this also adds required
  // nodes for these factors.
  // Return the number of factors added by the factor adders, not including the relative factors added by the node
  // adders.
  // TODO(rsoussan): Return both?
  int AddFactors(const localization_common::Time start_time, const localization_common::Time end_time);

  // Wrapper for AddFactors passing a pair of timestamps.
  int AddFactors(const std::pair<localization_common::Time, localization_common::Time>& start_and_end_time);

  // Performs optimization on the factor graph.
  bool Optimize();

  // Calculates the covariance matrix for the provided node's key.
  // Requires a successful round of optimization to have been performed.
  boost::optional<gtsam::Matrix> Covariance(const gtsam::Key& key) const;

  // Calculates the covariance matrix wrt two nodes for the provided keys.
  // Requires a successful round of optimization to have been performed.
  boost::optional<gtsam::Matrix> Covariance(const gtsam::Key& key_a, const gtsam::Key& key_b) const;

  // Returns set of factors currently in the graph.
  const gtsam::NonlinearFactorGraph& factors() const;

  // Returns set of factors currently in the graph.
  gtsam::NonlinearFactorGraph& factors();

  // Returns set of factors of FactorType currently in the graph.
  template <typename FactorType>
  std::vector<boost::shared_ptr<const FactorType>> Factors() const;

  // Returns number of factors currently in the graph.
  int num_factors() const;

  // Returns number of factors of provided FactorType currently in the graph.
  template <typename FactorType>
  int NumFactors() const;

  // Returns number of values currently in the graph.
  int num_values() const;

  // Graph optimizer params.
  const GraphOptimizerParams& params() const;

  // Returns a shared pointer to the values used by the graph optimizer.
  // All node adders added to the graph optimizer should be constructed
  // with these values.
  std::shared_ptr<nodes::Values> values();

  // Returns a const reference to the gtsam Values used within the Values object.
  const gtsam::Values& gtsam_values() const;

  // Returns a reference to the stats logger
  localization_common::StatsLogger& stats_logger();

  // Returns a const reference to the optimization timer
  const localization_common::Timer& optimization_timer() const;

  // Returns a const reference to the optimization_iterations averager
  const localization_common::Averager& optimization_iterations_averager() const;

  // Sum of factor errors for each factor in the graph
  double TotalGraphError() const;

  // Optional validity check for graph before optimizing.
  // If this fails, no optimization is performed.
  // Default behavior always returns true.
  virtual bool ValidGraph() const;

  // Prints factor graph information and logs stats.
  virtual void Print() const;

  // Saves the graph to a dot file, can be used to generate visual representation
  // of the graph.
  void SaveGraphDotFile(const std::string& output_path = "graph.dot") const;

  // Returns marginals if they have been calculated.
  boost::optional<const gtsam::Marginals&> marginals() const;

 private:
  // Add averagers and timers for logging.
  void AddAveragersAndTimers();

  // Returns a reference to the gtsam Values used within the Values object.
  gtsam::Values& gtsam_values();

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(optimizer_);
    ar& BOOST_SERIALIZATION_NVP(factors_);
    ar& BOOST_SERIALIZATION_NVP(values_);
    ar& BOOST_SERIALIZATION_NVP(factor_adders_);
    ar& BOOST_SERIALIZATION_NVP(node_adders_);
    ar& BOOST_SERIALIZATION_NVP(stats_logger_);
    ar& BOOST_SERIALIZATION_NVP(optimization_timer_);
    ar& BOOST_SERIALIZATION_NVP(optimization_timer_);
    ar& BOOST_SERIALIZATION_NVP(optimization_iterations_averager_);
    ar& BOOST_SERIALIZATION_NVP(total_error_averager_);
  }

  GraphOptimizerParams params_;
  std::unique_ptr<optimizers::Optimizer> optimizer_;
  gtsam::NonlinearFactorGraph factors_;
  std::shared_ptr<nodes::Values> values_;
  std::vector<std::shared_ptr<factor_adders::FactorAdder>> factor_adders_;
  std::vector<std::shared_ptr<node_adders::NodeAdder>> node_adders_;
  localization_common::StatsLogger stats_logger_;

  // Logging
  localization_common::Timer optimization_timer_ = localization_common::Timer("Optimization");
  localization_common::Averager optimization_iterations_averager_ =
    localization_common::Averager("Optimization Iterations");
  localization_common::Averager total_error_averager_ = localization_common::Averager("Total Factor Error");
};

// Implementation
template <typename FactorType>
std::vector<boost::shared_ptr<const FactorType>> GraphOptimizer::Factors() const {
  std::vector<boost::shared_ptr<const FactorType>> factors_vector;
  for (const auto& factor : factors()) {
    const auto casted_factor = boost::dynamic_pointer_cast<const FactorType>(factor);
    if (casted_factor) factors_vector.emplace_back(casted_factor);
  }
  return factors_vector;
}

template <typename FactorType>
int GraphOptimizer::NumFactors() const {
  int num_factors = 0;
  for (const auto& factor : factors()) {
    const auto casted_factor = boost::dynamic_pointer_cast<const FactorType>(factor);
    if (casted_factor) ++num_factors;
  }
  return num_factors;
}
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_H_
