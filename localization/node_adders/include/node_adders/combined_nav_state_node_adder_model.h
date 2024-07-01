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

#ifndef NODE_ADDERS_COMBINED_NAV_STATE_NODE_ADDER_MODEL_H_
#define NODE_ADDERS_COMBINED_NAV_STATE_NODE_ADDER_MODEL_H_

#include <imu_integration/imu_integrator.h>
#include <localization_common/combined_nav_state.h>
#include <localization_measurements/imu_measurement.h>
#include <node_adders/combined_nav_state_node_adder_model_params.h>
#include <node_adders/measurement_based_timestamped_node_adder_model.h>
#include <nodes/combined_nav_state_nodes.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <utility>
#include <vector>

namespace node_adders {
class CombinedNavStateNodeAdderModel
    : public MeasurementBasedTimestampedNodeAdderModel<localization_measurements::ImuMeasurement,
                                                       localization_common::CombinedNavState,
                                                       nodes::CombinedNavStateNodes> {
 public:
  using NodeType = localization_common::CombinedNavState;
  using NodesType = nodes::CombinedNavStateNodes;
  using MeasurementType = localization_measurements::ImuMeasurement;
  using Params = CombinedNavStateNodeAdderModelParams;
  using Base = MeasurementBasedTimestampedNodeAdderModel<MeasurementType, NodeType, NodesType>;
  explicit CombinedNavStateNodeAdderModel(const Params& params);
  void AddPriors(const NodeType& node, const std::vector<gtsam::SharedNoiseModel>& noise_models,
                 const localization_common::Time timestamp, const NodesType& nodes,
                 gtsam::NonlinearFactorGraph& factors) const final;
  bool AddNodesAndRelativeFactors(const localization_common::Time timestamp_a,
                                  const localization_common::Time timestamp_b, NodesType& nodes,
                                  gtsam::NonlinearFactorGraph& factors) const final;
  bool AddRelativeFactors(const localization_common::Time timestamp_a, const localization_common::Time timestamp_b,
                          const NodesType& nodes, gtsam::NonlinearFactorGraph& factors) const final;

  void AddMeasurement(const localization_measurements::ImuMeasurement& measurement) final;
  void RemoveMeasurements(const localization_common::Time oldest_allowed_time) final;

  bool CanAddNode(const localization_common::Time timestamp) const final;

  bool RemoveRelativeFactors(const localization_common::Time timestamp_a, const localization_common::Time timestamp_b,
                             const NodesType& nodes, gtsam::NonlinearFactorGraph& factors) const final;

  void SetFanSpeedMode(const localization_measurements::FanSpeedMode& fan_speed_mode);

 private:
  bool AddRelativeFactors(const gtsam::KeyVector& keys_a, const localization_common::Time timestamp_a,
                          const gtsam::KeyVector& keys_b, const localization_common::Time timestamp_b,
                          gtsam::NonlinearFactorGraph& factors) const;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(imu_integrator_);
  }

  imu_integration::ImuIntegrator imu_integrator_;
};
}  // namespace node_adders

#endif  // NODE_ADDERS_COMBINED_NAV_STATE_NODE_ADDER_MODEL_H_
