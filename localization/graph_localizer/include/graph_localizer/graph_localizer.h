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

#include <graph_localizer/feature_tracker.h>
#include <graph_localizer/graph_localizer_params.h>
#include <graph_localizer/graph_values.h>
#include <imu_integration/latest_imu_integrator.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <localization_common/time.h>
#include <localization_measurements/feature_points_measurement.h>
#include <localization_measurements/matched_projections_measurement.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/SmartFactorParams.h>

#include <ros/console.h>
#include <sensor_msgs/Imu.h>

#include <boost/serialization/serialization.hpp>

#include <map>
#include <memory>
#include <utility>

namespace graph_localizer {
// TODO(rsoussan): is this kosher?
namespace sym = gtsam::symbol_shorthand;
class GraphLocalizer {
 public:
  explicit GraphLocalizer(const GraphLocalizerParams& params);
  void AddImuMeasurement(const localization_measurements::ImuMeasurement& imu_measurement);
  boost::optional<localization_common::CombinedNavState> LatestCombinedNavState() const;
  boost::optional<localization_common::CombinedNavState> GetCombinedNavState(
      const localization_common::Time time) const;
  boost::optional<std::pair<localization_common::CombinedNavState, localization_common::CombinedNavStateCovariances>>
  LatestCombinedNavStateAndCovariances() const;
  bool AddOpticalFlowMeasurement(
      const localization_measurements::FeaturePointsMeasurement& optical_flow_feature_points_measurement);
  void AddARTagMeasurement(
      const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
      const gtsam::Pose3& dock_cam_T_dock);
  void AddSparseMappingMeasurement(
      const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement);
  void AddProjectionMeasurement(
      const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement,
      const gtsam::Pose3& body_T_cam, const boost::shared_ptr<gtsam::Cal3_S2>& cam_intrinsics,
      const gtsam::SharedIsotropic& cam_noise);

  bool Update();
  const FeatureTrackMap& feature_tracks() const { return feature_tracker_.feature_tracks(); }

  boost::optional<std::pair<gtsam::imuBias::ConstantBias, localization_common::Time>> LatestBiases() const;

  // Helper function for filling in loc state msg
  int NumOFFactors() const;

  // Helper function for filling in loc state msg
  int NumVLFactors() const;

 private:
  // Removes Keys and Values outside of sliding window.
  // Removes any factors depending on removed values
  bool SlideWindow(const gtsam::Marginals& marginals);
  // Integrates latest imu measurements up to timestamp and adds imu factor and
  // new combined nav state
  bool CreateAndAddLatestImuFactorAndCombinedNavState(const localization_common::Time timestamp);

  bool AddOrSplitImuFactorIfNeeded(const localization_common::Time timestamp);

  bool SplitOldImuFactorAndAddCombinedNavState(const localization_common::Time timestamp);

  void AddStartingPriors(const localization_common::CombinedNavState& global_cgN_body_start, const int key_index,
                         const gtsam::Values& values, gtsam::NonlinearFactorGraph& graph);

  void AddPriors(const localization_common::CombinedNavState& global_cgN_body,
                 const localization_common::CombinedNavStateNoise& noise, const int key_index,
                 const gtsam::Values& values, gtsam::NonlinearFactorGraph& graph);
  boost::optional<std::pair<localization_common::CombinedNavState, localization_common::CombinedNavStateCovariances>>
  LatestCombinedNavStateAndCovariances(const gtsam::Marginals& marginals) const;

  bool CreateAndAddImuFactorAndPredictedCombinedNavState(const localization_common::CombinedNavState& global_cgN_body,
                                                         const gtsam::PreintegratedCombinedMeasurements& pim);

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
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // ar& BOOST_SERIALIZATION_NVP(graph_);
    // ar& BOOST_SERIALIZATION_NVP(graph_values_.values());
  }

  // TODO(rsoussan): put these somewhere else
  gtsam::NavState global_gN_body_start_;
  imu_integration::LatestImuIntegrator latest_imu_integrator_;
  gtsam::NonlinearFactorGraph graph_;
  GraphValues graph_values_;
  FeatureTracker feature_tracker_;
  std::unique_ptr<gtsam::Marginals> marginals_;
  gtsam::SmartProjectionParams smart_projection_params_;
  gtsam::Pose3 body_T_nav_cam_;
  gtsam::Pose3 body_T_dock_cam_;
  gtsam::Pose3 world_T_dock_;
  boost::shared_ptr<gtsam::Cal3_S2> nav_cam_intrinsics_;
  boost::shared_ptr<gtsam::Cal3_S2> dock_cam_intrinsics_;
  gtsam::SharedIsotropic nav_cam_noise_;
  gtsam::SharedIsotropic dock_cam_noise_;
  double min_of_avg_distance_from_mean_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_H_
