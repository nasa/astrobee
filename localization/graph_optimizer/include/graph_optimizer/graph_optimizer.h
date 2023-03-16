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
#include <graph_optimizer/graph_stats.h>
#include <node_adders/node_adder.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/serialization/serialization.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace graph_optimizer {
// Time-based factor graph optimizer that performs optimization using
// Levenberg-Marquardt nonlinear least squares via GTSAM.
// Uses FactorAdders to generate and add factors for a given time range to the graph
// and NodeAdders to provide corresponding nodes for these factors at required timestamps.
// Generally FactorAdders are measurement-based and generate factors for a certain type of measurement at given
// timestamps. NodeAdders are also typically measurement-based and create linked nodes using relative factors that span
// the graph. As an example, a Visual-Inertial Odometry (VIO) graph may contain an image-based FactorAdder that
// generates visual-odometry factors using image measurements. A VIO graph will solve for Pose/Velocity/IMU Bias (PVB)
// values at different timestamps, so it will also contain a PVB NodeAdder that creates timestamped PVB nodes for the
// visual-odometry factors and links these nodes using IMU measurements. See the FactorAdder and NodeAdder packages for
// more information and different types of FactorAdders and NodeAdders. Acts as a base class for the
// SlidingWindowGraphOptimizer.
class GraphOptimizer {
 public:
  explicit GraphOptimizer(const GraphOptimizerParams& params);

  // Default constructor/destructor for serialization only
  GraphOptimizer() {}
  ~GraphOptimizer();

  // Adds node adder used for graph optimization.
  void AddNodeAdder(std::shared_ptr<node_adders::NodeAdder> node_adder);

  // Adds factor adder used for graph optimization.
  void AddFactorAdder(std::shared_ptr<factor_adders::FactorAdder> factor_adder);

  // Adds factors for each factor in valid time range to the graph.
  // Since factor adders are linked to the node adder they use, this also adds required
  // nodes for these factors.
  // Return the number of factors added by the factor adders, not including the relative factors added by the node
  // adders.
  // TODO(rsoussan): Return both?
  int AddFactors(const localization_common::Time start_time, const localization_common end_time);

  // Performs Levenberg-Marquardt nonlinear optimization using GTSAM on the factor graph.
  bool Optimize();

  // Adds factors and nodes to the graph within the provided time range and optimizes the graph.
  virtual bool AddFactorsAndOptimize(const localization_common::Time start_time, const localization_common end_time);

  // Returns set of factors currently in the graph.
  const gtsam::NonlinearFactorGraph& factors() const;

  // Returns set of factors currently in the graph.
  gtsam::NonlinearFactorGraph& factors();

  // Returns number of factors currently in the graph.
  const int num_factors() const;

  // Saves the graph to a dot file, can be used to generate visual representation
  // of the graph.
  void SaveGraphDotFile(const std::string& output_path = "graph.dot") const;

  // Graph optimizer params.
  const GraphOptimizerParams& params() const;

  // Prints logging statistics when graph optimizer is destructed, useful
  // for debugging.
  void LogOnDestruction(const bool log_on_destruction);

 private:
  // Optional validity check for graph before optimizing.
  // If this fails, no optimization is performed.
  // Default behavior always returns true.
  virtual bool ValidGraph() const;

  // Prints information for factors in the graph after adding factors and optimizing.
  // Default behavior does nothing.
  // TODO(rsoussan): Change default behavior to perform print() for each factor?
  virtual void PrintFactorDebugInfo() const;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(graph_stats_);
    ar& BOOST_SERIALIZATION_NVP(levenberg_marquardt_params_);
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(graph_);
    ar& BOOST_SERIALIZATION_NVP(log_on_destruction_);
    ar& BOOST_SERIALIZATION_NVP(node_adders_);
    ar& BOOST_SERIALIZATION_NVP(factor_adders_);
  }

  GraphOptimizerParams params_;
  gtsam::LevenbergMarquardtParams levenberg_marquardt_params_;
  gtsam::NonlinearFactorGraph graph_;
  std::vector<std::shared_ptr<factor_adders::FactorAdder>> factor_adders_;
  std::vector<std::shared_ptr<node_adders::NodeAdder>> node_adders_;
  GraphStats graph_stats_;
  bool log_on_destruction_;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_H_
