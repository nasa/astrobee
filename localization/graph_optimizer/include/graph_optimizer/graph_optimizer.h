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

#include <graph_optimizer/factor_to_add.h>
#include <graph_optimizer/graph_action_completer.h>
#include <graph_optimizer/graph_optimizer_params.h>
#include <graph_optimizer/graph_stats.h>
#include <graph_optimizer/key_info.h>
#include <graph_optimizer/node_updater.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/serialization/serialization.hpp>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace graph_optimizer {
class GraphOptimizer {
 public:
  GraphOptimizer(const GraphOptimizerParams& params,
                 std::unique_ptr<GraphStats> graph_stats = std::unique_ptr<GraphStats>(new GraphStats()));
  // Default constructor/destructor for serialization only
  GraphOptimizer() {}
  ~GraphOptimizer();

  // Registers graph action completer.  Called after adding buffered factors and updating nodes
  void AddGraphActionCompleter(std::shared_ptr<GraphActionCompleter> graph_action_completer);
  // Registers node updater. Called when adding buffered factors
  void AddNodeUpdater(std::shared_ptr<NodeUpdater> node_updater);
  // Adds factors to buffered list which is sorted by time
  void BufferFactors(const std::vector<FactorsToAdd>& factors_to_add_vec);
  // Adds buffered factors and optimizes graph.  Calls DoPostOptimizeActions afterwards
  bool Update();
  const gtsam::NonlinearFactorGraph& graph_factors() const;
  gtsam::NonlinearFactorGraph& graph_factors();
  void SaveGraphDotFile(const std::string& output_path = "graph.dot") const;
  const GraphOptimizerParams& params() const;
  void LogOnDestruction(const bool log_on_destruction);
  const GraphStats* const graph_stats() const;
  GraphStats* graph_stats();
  const boost::optional<gtsam::Marginals>& marginals() const;

  std::shared_ptr<gtsam::Values> values();

 private:
  gtsam::NonlinearFactorGraph MarginalFactors(const gtsam::NonlinearFactorGraph& old_factors,
                                              const gtsam::KeyVector& old_keys,
                                              const gtsam::GaussianFactorGraph::Eliminate& eliminate_function) const;

  // Removes Keys and Values outside of sliding window.
  // Removes any factors depending on removed values
  // Optionally adds marginalized factors encapsulating linearized error of removed factors
  // Optionally adds priors using marginalized covariances for new oldest states
  bool SlideWindow(const boost::optional<gtsam::Marginals>& marginals,
                   const localization_common::Time last_latest_time);

  boost::optional<localization_common::Time> SlideWindowNewOldestTime() const;

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time) const;

  std::pair<gtsam::KeyVector, gtsam::NonlinearFactorGraph> OldKeysAndFactors(
    const localization_common::Time oldest_allowed_time);

  // Called after SlideWindow
  virtual void DoPostSlideWindowActions(const localization_common::Time oldest_allowed_time,
                                        const boost::optional<gtsam::Marginals>& marginals);

  // void UpdatePointPriors(const gtsam::Marginals& marginals);

  // Creates factors for cumulative factor adders and adds them to buffered list
  virtual void BufferCumulativeFactors();

  // Removes old measurements from cumulative factors that do not fit in sliding window so cumulative factors are still
  // valid after the SlideWindow call
  virtual void RemoveOldMeasurementsFromCumulativeFactors(const gtsam::KeyVector& old_keys);

  // Adds buffered factors to graph_factors for future optimization
  // Only adds factors that pass ReadyToAddFactors(timestamp) check
  int AddBufferedFactors();

  // Calls Update for each registered NodeUpdater to create required nodes while inserting new factors
  bool UpdateNodes(const KeyInfo& key_info);

  // Calls DoAction for each registered GraphActionCompleter after inserting new factors
  bool DoGraphAction(FactorsToAdd& factors_to_add);

  boost::optional<gtsam::Key> GetKey(KeyCreatorFunction key_creator_function,
                                     const localization_common::Time timestamp) const;

  // Updates keys in factors after new nodes are created/updated as required by registered NodeUpdater
  bool Rekey(FactorToAdd& factor_to_add);

  // Checks whether the graph is ready for insertion of a factor with timestamp
  virtual bool ReadyToAddFactors(const localization_common::Time timestamp) const;

  boost::optional<localization_common::Time> OldestTimestamp() const;

  boost::optional<localization_common::Time> LatestTimestamp() const;

  // Checks whether a measurement is too old to be inserted into graph
  virtual bool MeasurementRecentEnough(const localization_common::Time timestamp) const;

  // Removes buffered factors that are too old for insertion into graph
  void RemoveOldBufferedFactors(const localization_common::Time oldest_allowed_timestamp);

  virtual void PrintFactorDebugInfo() const;

  // Called after optimizing graph
  virtual bool DoPostOptimizeActions();

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(graph_);
    ar& BOOST_SERIALIZATION_NVP(values_);
  }

  std::unique_ptr<GraphStats> graph_stats_;
  std::shared_ptr<gtsam::Values> values_;
  bool log_on_destruction_;
  GraphOptimizerParams params_;
  gtsam::LevenbergMarquardtParams levenberg_marquardt_params_;
  gtsam::NonlinearFactorGraph graph_;
  boost::optional<gtsam::Marginals> marginals_;
  std::multimap<localization_common::Time, FactorsToAdd> buffered_factors_to_add_;

  std::vector<std::shared_ptr<NodeUpdater>> node_updaters_;
  std::vector<std::shared_ptr<GraphActionCompleter>> graph_action_completers_;
  gtsam::Marginals::Factorization marginals_factorization_;
  boost::optional<localization_common::Time> last_latest_time_;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_H_
