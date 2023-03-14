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

#include <graph_optimizer/graph_optimizer_params.h>
#include <graph_optimizer/graph_stats.h>
#include <graph_optimizer/node_updater.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/serialization/serialization.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace graph_optimizer {
class GraphOptimizer {
 public:
  explicit GraphOptimizer(const GraphOptimizerParams& params);

  // Default constructor/destructor for serialization only
  GraphOptimizer() {}
  ~GraphOptimizer();

  // Adds node updaters.
  void AddNodeUpdater(std::shared_ptr<NodeUpdater> node_updater);
  // Optimizes the graph.
  virtual bool Update();
  const gtsam::NonlinearFactorGraph& factors() const;
  gtsam::NonlinearFactorGraph& factors();
  const int num_factors() const;
  void SaveGraphDotFile(const std::string& output_path = "graph.dot") const;
  const GraphOptimizerParams& params() const;
  void LogOnDestruction(const bool log_on_destruction);
  const GraphStats& graph_stats() const;
  GraphStats& graph_stats();
  const boost::optional<gtsam::Marginals>& marginals() const;

 private:
  // Optional validity check for graph before optimizing
  virtual bool ValidGraph() const;

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
    ar& BOOST_SERIALIZATION_NVP(node_updaters_);
  }

  GraphStats graph_stats_;
  bool log_on_destruction_;
  GraphOptimizerParams params_;
  gtsam::LevenbergMarquardtParams levenberg_marquardt_params_;
  gtsam::NonlinearFactorGraph graph_;
  std::vector<std::shared_ptr<NodeUpdater>> node_updaters_;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_H_
