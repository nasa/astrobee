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

#ifndef GRAPH_VIO_GRAPH_VIO_H_
#define GRAPH_VIO_GRAPH_VIO_H_

#include <factor_adders/standstill_factor_adder.h>
#include <factor_adders/projection_factor_adder.h>
#include <factor_adders/smart_projection_cumulative_factor_adder.h>
#include <graph_factors/robust_smart_projection_pose_factor.h>
#include <graph_vio/graph_vio_params.h>
#include <graph_vio/graph_vio_stats.h>
#include <graph_vio/projection_graph_action_completer.h>
#include <graph_vio/smart_projection_graph_action_completer.h>
#include <graph_optimizer/graph_optimizer.h>
#include <imu_integration/latest_imu_integrator.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <localization_common/time.h>
#include <localization_measurements/fan_speed_mode.h>
#include <localization_measurements/feature_points_measurement.h>
#include <node_updaters/combined_nav_state_node_updater.h>
#include <node_updaters/combined_nav_state_node_updater_params.h>
#include <node_updaters/feature_point_node_updater.h>
#include <vision_common/feature_tracker.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <boost/serialization/serialization.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace graph_vio {
namespace sym = gtsam::symbol_shorthand;
using Calibration = gtsam::Cal3_S2;
using Camera = gtsam::PinholePose<Calibration>;
using RobustSmartFactor = gtsam::RobustSmartProjectionPoseFactor<Calibration>;
using SharedRobustSmartFactor = boost::shared_ptr<RobustSmartFactor>;
using ProjectionFactor = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3>;

class GraphVIO : public graph_optimizer::GraphOptimizer {
 public:
  explicit GraphVIO(const GraphVIOParams& params);
  // For Serialization Only
  GraphVIO() {}
  ~GraphVIO() = default;
  void AddImuMeasurement(const localization_measurements::ImuMeasurement& imu_measurement);
  boost::optional<localization_common::CombinedNavState> LatestCombinedNavState() const;
  boost::optional<std::pair<localization_common::CombinedNavState, localization_common::CombinedNavStateCovariances>>
  LatestCombinedNavStateAndCovariances() const;
  bool AddOpticalFlowMeasurement(
    const localization_measurements::FeaturePointsMeasurement& optical_flow_feature_points_measurement);
  void CheckForStandstill();
  bool DoPostOptimizeActions() final;
  const vision_common::FeatureTrackIdMap& feature_tracks() const { return feature_tracker_->feature_tracks(); }

  boost::optional<std::pair<gtsam::imuBias::ConstantBias, localization_common::Time>> LatestBiases() const;

  int NumFeatures() const;

  int NumOFFactors(const bool check_valid = true) const;

  int NumProjectionFactors(const bool check_valid = true) const;

  bool standstill() const;

  const GraphVIOParams& params() const;

  const GraphVIOStats& graph_vio_stats() const;

  void SetFanSpeedMode(const localization_measurements::FanSpeedMode fan_speed_mode);

  const localization_measurements::FanSpeedMode fan_speed_mode() const;

  const graph_values::CombinedNavStateGraphValues& combined_nav_state_graph_values() const;

  const node_updaters::CombinedNavStateNodeUpdater& combined_nav_state_node_updater() const;

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
    ar& BOOST_SERIALIZATION_NVP(feature_tracker_);
    ar& BOOST_SERIALIZATION_NVP(combined_nav_state_node_updater_);
  }

  std::shared_ptr<vision_common::FeatureTracker> feature_tracker_;
  std::shared_ptr<imu_integration::LatestImuIntegrator> latest_imu_integrator_;
  GraphVIOParams params_;
  boost::optional<localization_measurements::FeaturePointsMeasurement> last_optical_flow_measurement_;

  // Factor Adders
  std::shared_ptr<factor_adders::ProjectionFactorAdder> projection_factor_adder_;
  std::shared_ptr<factor_adders::SmartProjectionCumulativeFactorAdder> smart_projection_cumulative_factor_adder_;
  std::shared_ptr<factor_adders::StandstillFactorAdder> standstill_factor_adder_;

  // Node Updaters
  std::shared_ptr<node_updaters::CombinedNavStateNodeUpdater> combined_nav_state_node_updater_;
  std::shared_ptr<node_updaters::FeaturePointNodeUpdater> feature_point_node_updater_;

  // Graph Action Completers
  std::shared_ptr<ProjectionGraphActionCompleter> projection_graph_action_completer_;
  std::shared_ptr<SmartProjectionGraphActionCompleter> smart_projection_graph_action_completer_;

  boost::optional<bool> standstill_;
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_GRAPH_VIO_H_