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

#include <graph_localizer/combined_nav_state_node_updater_params.h>
#include <graph_localizer/timestamped_node_updater.h>
#include <imu_integration/latest_imu_integrator.h>
#include <localization_common/combined_nav_state.h>

namespace graph_localizer {
class CombinedNavStateNodeUpdater
    : public TimestampedNodeUpdater<localization_common::CombinedNavState, localization_common::CombinedNavStateNoise> {
 public:
  CombinedNavStateNodeUpdater(const CombinedNavStateNodeUpdaterParams& params,
                              std::shared_ptr<imu_integration::LatestImuIntegrator> latest_imu_integrator);

  void AddInitialValuesAndPriors(gtsam::NonlinearFactorGraph& factors, GraphValues& graph_values);

  void AddInitialValuesAndPriors(const localization_common::CombinedNavState& global_N_body,
                                 const localization_common::CombinedNavStateNoise& noise,
                                 gtsam::NonlinearFactorGraph& factors, GraphValues& graph_values) final;

  void AddPriors(const localization_common::CombinedNavState& global_N_body,
                 const localization_common::CombinedNavStateNoise& noise, const GraphValues& graph_values,
                 gtsam::NonlinearFactorGraph& factors) final;

  bool Update(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors,
              GraphValues& graph_values) final;

  bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                   const boost::optional<gtsam::Marginals>& marginals, const double huber_k,
                   gtsam::NonlinearFactorGraph& factors, GraphValues& graph_values) final;

 private:
  void RemovePriors(const int key_index, gtsam::NonlinearFactorGraph& factors);
  int GenerateKeyIndex();
  bool AddOrSplitImuFactorIfNeeded(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors,
                                   GraphValues& graph_values);
  bool CreateAndAddLatestImuFactorAndCombinedNavState(const localization_common::Time timestamp,
                                                      gtsam::NonlinearFactorGraph& factors, GraphValues& graph_values);
  bool CreateAndAddImuFactorAndPredictedCombinedNavState(const localization_common::CombinedNavState& global_N_body,
                                                         const gtsam::PreintegratedCombinedMeasurements& pim,
                                                         gtsam::NonlinearFactorGraph& factors,
                                                         GraphValues& graph_values);
  bool SplitOldImuFactorAndAddCombinedNavState(const localization_common::Time timestamp,
                                               gtsam::NonlinearFactorGraph& factors, GraphValues& graph_values);

  CombinedNavStateNodeUpdaterParams params_;
  std::shared_ptr<imu_integration::LatestImuIntegrator> latest_imu_integrator_;
  int key_index_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_COMBINED_NAV_STATE_NODE_UPDATER_H_
