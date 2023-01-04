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

#ifndef GRAPH_LOCALIZER_GRAPH_LOCALIZER_H_
#define GRAPH_LOCALIZER_GRAPH_LOCALIZER_H_

#include <graph_localizer/depth_odometry_factor_adder.h>
#include <graph_localizer/handrail_factor_adder.h>
#include <graph_localizer/graph_localizer_params.h>
#include <graph_localizer/graph_localizer_stats.h>
#include <graph_localizer/loc_factor_adder.h>
#include <graph_localizer/loc_graph_action_completer.h>
#include <graph_localizer/rotation_factor_adder.h>
#include <graph_optimizer/graph_optimizer.h>
#include <localization_common/time.h>
#include <localization_measurements/depth_odometry_measurement.h>
#include <localization_measurements/handrail_points_measurement.h>
#include <localization_measurements/matched_projections_measurement.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/serialization/serialization.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace graph_localizer {
namespace sym = gtsam::symbol_shorthand;
using Calibration = gtsam::Cal3_S2;
using Camera = gtsam::PinholePose<Calibration>;

class GraphLocalizer : public graph_optimizer::GraphOptimizer {
 public:
  explicit GraphLocalizer(const GraphLocalizerParams& params);
  // For Serialization Only
  GraphLocalizer() {}
  ~GraphLocalizer() = default;
  /*boost::optional<localization_common::CombinedNavState> LatestCombinedNavState() const;
  boost::optional<localization_common::CombinedNavState> GetCombinedNavState(
    const localization_common::Time time) const;
  boost::optional<std::pair<localization_common::CombinedNavState, localization_common::CombinedNavStateCovariances>>
  LatestCombinedNavStateAndCovariances() const;*/
  void AddARTagMeasurement(
    const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement);
  void AddSparseMappingMeasurement(
    const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement);
  void AddHandrailMeasurement(const localization_measurements::HandrailPointsMeasurement& handrail_points_measurement);
  void AddDepthOdometryMeasurement(
    const localization_measurements::DepthOdometryMeasurement& depth_odometry_measurement);
  bool DoPostOptimizeActions() final;

  boost::optional<localization_common::Time> LatestExtrapolatedPoseTime() const;

  const GraphLocalizerParams& params() const;

  const GraphLocalizerStats& graph_localizer_stats() const;

  /*const CombinedNavStateGraphValues& combined_nav_state_graph_values() const;

  const CombinedNavStateNodeUpdater& combined_nav_state_node_updater() const;*/

 private:
  void InitializeNodeUpdaters();
  void InitializeFactorAdders();
  void InitializeGraphActionCompleters();
  void DoPostSlideWindowActions(const localization_common::Time oldest_allowed_time,
                                const boost::optional<gtsam::Marginals>& marginals) final;

  boost::optional<std::pair<localization_common::CombinedNavState, localization_common::CombinedNavStateCovariances>>
  LatestCombinedNavStateAndCovariances(const gtsam::Marginals& marginals) const;

  void BufferCumulativeFactors() final;

  void RemoveOldMeasurementsFromCumulativeFactors(const gtsam::KeyVector& old_keys) final;

  bool ValidGraph() const final;

  bool ReadyToAddFactors(const localization_common::Time timestamp) const final;

  bool MeasurementRecentEnough(const localization_common::Time timestamp) const final;

  void PrintFactorDebugInfo() const final;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(graph_optimizer::GraphOptimizer);
    // ar& BOOST_SERIALIZATION_NVP(combined_nav_state_node_updater_);
  }

  GraphLocalizerParams params_;

  // Factor Adders
  std::shared_ptr<LocFactorAdder> ar_tag_loc_factor_adder_;
  std::shared_ptr<DepthOdometryFactorAdder> depth_odometry_factor_adder_;
  std::shared_ptr<HandrailFactorAdder> handrail_factor_adder_;
  std::shared_ptr<LocFactorAdder> loc_factor_adder_;
  std::shared_ptr<RotationFactorAdder> rotation_factor_adder_;

  // Node Updaters
//  std::shared_ptr<CombinedNavStateNodeUpdater> combined_nav_state_node_updater_;

  // Graph Action Completers
  std::shared_ptr<LocGraphActionCompleter> ar_tag_loc_graph_action_completer_;
  std::shared_ptr<LocGraphActionCompleter> loc_graph_action_completer_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_H_
