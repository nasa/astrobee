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

#ifndef GRAPH_LOCALIZER_GRAPH_OPTIMIZER_H_
#define GRAPH_LOCALIZER_GRAPH_OPTIMIZER_H_

#include <graph_localizer/factor_to_add.h>
#include <graph_localizer/graph_action_completer.h>
#include <graph_localizer/graph_optimizer_params.h>
#include <graph_localizer/graph_stats.h>
#include <graph_localizer/graph_values.h>
#include <graph_localizer/key_info.h>
#include <graph_localizer/timestamped_node_updater.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/serialization/serialization.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace graph_localizer {
// namespace sym = gtsam::symbol_shorthand;

class GraphOptimizer {
 public:
  explicit GraphOptimizer(const GraphOptimizerParams& params);
  // For Serialization Only
  GraphOptimizer() {}
  ~GraphOptimizer();
  bool Update();
  const GraphValues& graph_values() const;
  const gtsam::NonlinearFactorGraph& factor_graph() const;
  void SaveGraphDotFile(const std::string& output_path = "graph.dot") const;
  const GraphOptimizerParams& params() const;

  template <typename FactorType>
  int NumFactors() const {
    int num_factors = 0;
    for (const auto& factor : graph_) {
      if (dynamic_cast<const FactorType*>(factor.get())) {
        ++num_factors;
      }
    }
    return num_factors;
  }

  void LogOnDestruction(const bool log_on_destruction);

  const GraphStatsBase& graph_stats() const;

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

  // void UpdatePointPriors(const gtsam::Marginals& marginals);

  void BufferFactors(const std::vector<FactorsToAdd>& factors_to_add_vec);

  virtual void BufferCumulativeFactors();

  virtual void RemoveOldMeasurementsFromCumulativeFactors(const gtsam::KeyVector& old_keys);

  int AddBufferedFactors();

  bool UpdateNodes(const KeyInfo& key_info);

  bool DoGraphAction(FactorsToAdd& factors_to_add);

  bool Rekey(FactorToAdd& factor_to_add);

  virtual bool ReadyToAddMeasurement(const localization_common::Time timestamp) const;

  virtual bool MeasurementRecentEnough(const localization_common::Time timestamp) const;

  void RemoveOldBufferedFactors(const localization_common::Time oldest_allowed_timestamp);

  virtual void PrintFactorDebugInfo() const;

  virtual void PostOptimizeActions();

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    // ar& BOOST_SERIALIZATION_NVP(feature_tracker_);
    ar& BOOST_SERIALIZATION_NVP(graph_);
    ar& BOOST_SERIALIZATION_NVP(graph_values_);
  }

  std::shared_ptr<GraphValues> graph_values_;
  bool log_on_destruction_;
  GraphOptimizerParams params_;
  gtsam::LevenbergMarquardtParams levenberg_marquardt_params_;
  gtsam::NonlinearFactorGraph graph_;
  boost::optional<gtsam::Marginals> marginals_;
  std::multimap<localization_common::Time, FactorsToAdd> buffered_factors_to_add_;

  std::vector<std::shared_ptr<TimestampedNodeUpdater>> timestamped_node_updaters_;
  std::vector<std::shared_ptr<GraphActionCompleter>> graph_action_completers_;
  gtsam::Marginals::Factorization marginals_factorization_;
  boost::optional<localization_common::Time> last_latest_time_;
  GraphStatsBase graph_stats_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_OPTIMIZER_H_
