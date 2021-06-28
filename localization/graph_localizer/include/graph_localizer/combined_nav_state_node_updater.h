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

#ifndef GRAPH_LOCALIZER_COMBINED_NAV_STATE_NODE_UPDATER_H_
#define GRAPH_LOCALIZER_COMBINED_NAV_STATE_NODE_UPDATER_H_

#include <graph_localizer/combined_nav_state_graph_values.h>
#include <graph_localizer/combined_nav_state_node_updater_params.h>
#include <graph_optimizer/key_info.h>
#include <graph_optimizer/node_updater_with_priors.h>
#include <imu_integration/latest_imu_integrator.h>
#include <localization_common/combined_nav_state.h>

namespace graph_localizer {
class CombinedNavStateNodeUpdater
    : public graph_optimizer::NodeUpdaterWithPriors<localization_common::CombinedNavState,
                                                    localization_common::CombinedNavStateNoise> {
 public:
  CombinedNavStateNodeUpdater(const CombinedNavStateNodeUpdaterParams& params,
                              std::shared_ptr<imu_integration::LatestImuIntegrator> latest_imu_integrator,
                              std::shared_ptr<gtsam::Values> values);
  CombinedNavStateNodeUpdater() = default;

  void AddInitialValuesAndPriors(gtsam::NonlinearFactorGraph& factors);

  void AddInitialValuesAndPriors(const localization_common::CombinedNavState& global_N_body,
                                 const localization_common::CombinedNavStateNoise& noise,
                                 gtsam::NonlinearFactorGraph& factors) final;

  void AddPriors(const localization_common::CombinedNavState& global_N_body,
                 const localization_common::CombinedNavStateNoise& noise, gtsam::NonlinearFactorGraph& factors) final;

  bool Update(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final;

  bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                   const boost::optional<gtsam::Marginals>& marginals, const gtsam::KeyVector& old_keys,
                   const double huber_k, gtsam::NonlinearFactorGraph& factors) final;

  void ThresholdBiasUncertainty(gtsam::Matrix& bias_covariance) const;

  graph_optimizer::NodeUpdaterType type() const final;

  boost::optional<localization_common::Time> SlideWindowNewOldestTime() const final;

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                           const gtsam::NonlinearFactorGraph& graph) const final;

  boost::optional<gtsam::Key> GetKey(graph_optimizer::KeyCreatorFunction key_creator_function,
                                     const localization_common::Time timestamp) const final;

  boost::optional<localization_common::Time> OldestTimestamp() const final;

  boost::optional<localization_common::Time> LatestTimestamp() const final;

  std::shared_ptr<const CombinedNavStateGraphValues> shared_graph_values() const;

  std::shared_ptr<CombinedNavStateGraphValues> shared_graph_values();

  const CombinedNavStateGraphValues& graph_values() const;

 private:
  void RemovePriors(const int key_index, gtsam::NonlinearFactorGraph& factors);
  int GenerateKeyIndex();
  bool AddOrSplitImuFactorIfNeeded(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors,
                                   CombinedNavStateGraphValues& graph_values);
  bool CreateAndAddLatestImuFactorAndCombinedNavState(const localization_common::Time timestamp,
                                                      gtsam::NonlinearFactorGraph& factors,
                                                      CombinedNavStateGraphValues& graph_values);
  bool CreateAndAddImuFactorAndPredictedCombinedNavState(const localization_common::CombinedNavState& global_N_body,
                                                         const gtsam::PreintegratedCombinedMeasurements& pim,
                                                         gtsam::NonlinearFactorGraph& factors,
                                                         CombinedNavStateGraphValues& graph_values);
  bool SplitOldImuFactorAndAddCombinedNavState(const localization_common::Time timestamp,
                                               gtsam::NonlinearFactorGraph& factors,
                                               CombinedNavStateGraphValues& graph_values);

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(params_);
    // ar& BOOST_SERIALIZATION_NVP(latest_imu_integrator_);
    ar& BOOST_SERIALIZATION_NVP(graph_values_);
    ar& BOOST_SERIALIZATION_NVP(key_index_);
    ar& BOOST_SERIALIZATION_NVP(global_N_body_start_noise_);
  }

  CombinedNavStateNodeUpdaterParams params_;
  std::shared_ptr<imu_integration::LatestImuIntegrator> latest_imu_integrator_;
  std::shared_ptr<CombinedNavStateGraphValues> graph_values_;
  int key_index_;
  localization_common::CombinedNavStateNoise global_N_body_start_noise_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_COMBINED_NAV_STATE_NODE_UPDATER_H_
