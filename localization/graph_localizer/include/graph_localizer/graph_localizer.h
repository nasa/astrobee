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

#include <graph_localizer/factor_to_add.h>
#include <graph_localizer/feature_tracker.h>
#include <graph_localizer/graph_action.h>
#include <graph_localizer/graph_localizer_params.h>
#include <graph_localizer/graph_values.h>
#include <graph_localizer/key_info.h>
#include <imu_integration/latest_imu_integrator.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <localization_common/time.h>
#include <localization_common/timer.h>
#include <localization_measurements/feature_points_measurement.h>
#include <localization_measurements/matched_projections_measurement.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <boost/serialization/serialization.hpp>

#include <map>
#include <string>
#include <utility>

namespace graph_localizer {
namespace sym = gtsam::symbol_shorthand;
using Calibration = gtsam::Cal3_S2;
using Camera = gtsam::PinholeCamera<Calibration>;
using SmartFactor = gtsam::SmartProjectionPoseFactor<Calibration>;
using SharedSmartFactor = boost::shared_ptr<SmartFactor>;

class GraphLocalizer {
 public:
  explicit GraphLocalizer(const GraphLocalizerParams& params);
  // For Serialization Only
  GraphLocalizer() {}
  void AddImuMeasurement(const localization_measurements::ImuMeasurement& imu_measurement);
  boost::optional<localization_common::CombinedNavState> LatestCombinedNavState() const;
  boost::optional<localization_common::CombinedNavState> GetCombinedNavState(
      const localization_common::Time time) const;
  boost::optional<std::pair<localization_common::CombinedNavState, localization_common::CombinedNavStateCovariances>>
  LatestCombinedNavStateAndCovariances() const;
  bool AddOpticalFlowMeasurement(
      const localization_measurements::FeaturePointsMeasurement& optical_flow_feature_points_measurement);
  void AddARTagMeasurement(
      const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement);
  void AddSparseMappingMeasurement(
      const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement);
  void AddProjectionMeasurement(
      const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
      const gtsam::Pose3& body_T_cam, const boost::shared_ptr<gtsam::Cal3_S2>& cam_intrinsics,
      const gtsam::SharedNoiseModel& cam_noise, const GraphAction& graph_action = GraphAction::kNone);

  bool Update();
  const FeatureTrackMap& feature_tracks() const { return feature_tracker_.feature_tracks(); }

  boost::optional<std::pair<gtsam::imuBias::ConstantBias, localization_common::Time>> LatestBiases() const;

  int NumOFFactors() const;

  int NumVLFactors() const;

  boost::optional<std::pair<gtsam::Pose3, localization_common::Time>> estimated_world_T_dock() const;

  const GraphValues& graph_values() const;

  const gtsam::NonlinearFactorGraph& factor_graph() const;

  void SaveGraphDotFile(const std::string& output_path = "graph.dot") const;

 private:
  // Removes Keys and Values outside of sliding window.
  // Removes any factors depending on removed values
  bool SlideWindow(const boost::optional<gtsam::Marginals>& marginals);
  // Integrates latest imu measurements up to timestamp and adds imu factor and
  // new combined nav state
  bool CreateAndAddLatestImuFactorAndCombinedNavState(const localization_common::Time timestamp);

  bool AddOrSplitImuFactorIfNeeded(const localization_common::Time timestamp);

  bool SplitOldImuFactorAndAddCombinedNavState(const localization_common::Time timestamp);

  void AddStartingPriors(const localization_common::CombinedNavState& global_N_body_start, const int key_index,
                         const gtsam::Values& values, gtsam::NonlinearFactorGraph& graph);

  void AddPriors(const localization_common::CombinedNavState& global_N_body,
                 const localization_common::CombinedNavStateNoise& noise, const int key_index,
                 const gtsam::Values& values, gtsam::NonlinearFactorGraph& graph);

  void AddSmartFactor(const FeatureTrack& feature_track, FactorsToAdd& smart_factors_to_add);

  boost::optional<std::pair<localization_common::CombinedNavState, localization_common::CombinedNavStateCovariances>>
  LatestCombinedNavStateAndCovariances(const gtsam::Marginals& marginals) const;

  bool CreateAndAddImuFactorAndPredictedCombinedNavState(const localization_common::CombinedNavState& global_N_body,
                                                         const gtsam::PreintegratedCombinedMeasurements& pim);

  void BufferFactors(const FactorsToAdd& factors_to_add);

  void AddBufferedFactors();

  bool DoGraphAction(FactorsToAdd& factors_to_add);

  bool Rekey(FactorToAdd& factor_to_add);

  bool ReadyToAddMeasurement(const localization_common::Time timestamp) const;

  bool TransformARMeasurementAndUpdateDockTWorld(FactorsToAdd& factors_to_add);

  void AddStandstillVelocityPriorFactor(const localization_common::Time timestamp,
                                        FactorsToAdd& standstill_prior_factors_to_add);

  void AddCombinedImuFactorAndBiasPrior(const gtsam::CombinedImuFactor::shared_ptr& combined_imu_factor);

  bool MeasurementRecentEnough(const localization_common::Time timestamp) const;

  void RemoveOldBufferedFactors(const localization_common::Time oldest_allowed_timestamp);

  template <typename FactorType>
  void DeleteFactors() {
    int num_removed_factors = 0;
    for (auto factor_it = graph_.begin(); factor_it != graph_.end();) {
      if (dynamic_cast<FactorType*>(factor_it->get())) {
        factor_it = graph_.erase(factor_it);
        ++num_removed_factors;
        continue;
      }
      ++factor_it;
    }
    VLOG(2) << "DeleteFactors: Num removed factors: " << num_removed_factors;
  }

  // TODO(rsoussan): make a static and dynamic key index?
  static int GenerateKeyIndex() {
    static int key_index = 0;
    return key_index++;
  }

  void PrintFactorDebugInfo() const;

  template <typename FactorType>
  int NumFactors() const {
    int num_factors = 0;
    for (auto factor_it = graph_.begin(); factor_it != graph_.end(); ++factor_it) {
      if (dynamic_cast<const FactorType*>(factor_it->get())) {
        ++num_factors;
      }
    }
    return num_factors;
  }

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(graph_);
    ar& BOOST_SERIALIZATION_NVP(graph_values_);
  }

  GraphLocalizerParams params_;
  gtsam::LevenbergMarquardtParams levenberg_marquardt_params_;
  imu_integration::LatestImuIntegrator latest_imu_integrator_;
  gtsam::NonlinearFactorGraph graph_;
  GraphValues graph_values_;
  FeatureTracker feature_tracker_;
  boost::optional<gtsam::Marginals> marginals_;
  gtsam::SmartProjectionParams smart_projection_params_;
  boost::optional<std::pair<gtsam::Pose3, localization_common::Time>> estimated_world_T_dock_;
  std::map<localization_common::Time, gtsam::Pose3> dock_cam_T_dock_estimates_;
  std::multimap<localization_common::Time, FactorsToAdd> buffered_factors_to_add_;
  localization_common::Timer optimization_timer_;
  localization_common::Timer marginals_timer_;
  gtsam::Marginals::Factorization marginals_factorization_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_H_
