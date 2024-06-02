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

#include <factor_adders/depth_odometry_factor_adder.h>
#include <factor_adders/standstill_factor_adder.h>
#include <factor_adders/vo_smart_projection_factor_adder.h>
#include <graph_vio/graph_vio_params.h>
#include <localization_common/marginals_pose_covariance_interpolater.h>
#include <localization_measurements/depth_odometry_measurement.h>
#include <localization_measurements/fan_speed_mode.h>
#include <localization_measurements/feature_points_measurement.h>
#include <localization_measurements/imu_measurement.h>
#include <localization_measurements/standstill_measurement.h>
#include <node_adders/combined_nav_state_node_adder.h>
#include <nodes/combined_nav_state_nodes.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer.h>
#include <vision_common/feature_tracker.h>

#include <boost/serialization/serialization.hpp>

namespace graph_vio {
// Siding window graph optimizer that uses IMU and optical flow feature point measurements
// with VO smart projection factors to perform VIO.
// Uses the CombinedNavStateNodeAdder to add relative IMU factors between graph nodes.
// Also adds standstill pose and velocity factors when standstill is detected
// using the history of feature point measurements.
class GraphVIO : public sliding_window_graph_optimizer::SlidingWindowGraphOptimizer {
 public:
  explicit GraphVIO(const GraphVIOParams& params);

  // For Serialization Only
  GraphVIO() {}

  // Adds imu measurement to combined nav state node adder.
  void AddImuMeasurement(const localization_measurements::ImuMeasurement& imu_measurement);

  // Sets the fan speed mode in the combined nav state node model's IMU integrator
  void SetFanSpeedMode(const localization_measurements::FanSpeedMode& fan_speed_mode);

  // Adds feature points measurement to vo smart projection factor adder.
  void AddFeaturePointsMeasurement(
    const localization_measurements::FeaturePointsMeasurement& feature_points_measurement);

  // Adds depth odometry measurement for depth odometry relative pose factor adder.
  void AddDepthOdometryMeasurement(
    const localization_measurements::DepthOdometryMeasurement& depth_odometry_measurement);

  // Returns a const reference to combined nav state nodes.
  const nodes::CombinedNavStateNodes& combined_nav_state_nodes() const;

  // Returns whether standstill is detected or not.
  bool standstill() const;

  // Const accesor to feature tracker used for smart factor creation
  const vision_common::SpacedFeatureTracker& feature_tracker() const;

  // Construct pose covariance interpolater using latest marginals and nodes
  std::shared_ptr<localization_common::MarginalsPoseCovarianceInterpolater<nodes::CombinedNavStateNodes>>
  MarginalsPoseCovarianceInterpolater();

 private:
  // Uses the latest feature track points to detect standstill.
  bool CheckForStandstill(const localization_common::Time oldest_allowed_time);

  // bool ValidGraph() const final;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(sliding_window_graph_optimizer::SlidingWindowGraphOptimizer);
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(vo_smart_projection_factor_adder_);
    ar& BOOST_SERIALIZATION_NVP(standstill_factor_adder_);
    ar& BOOST_SERIALIZATION_NVP(depth_odometry_factor_adder_);
    ar& BOOST_SERIALIZATION_NVP(combined_nav_state_node_adder_);
  }

  GraphVIOParams params_;
  std::shared_ptr<node_adders::CombinedNavStateNodeAdder> combined_nav_state_node_adder_;
  bool standstill_;

  // Factor Adders
  std::shared_ptr<factor_adders::DepthOdometryFactorAdder<node_adders::CombinedNavStateNodeAdder>>
    depth_odometry_factor_adder_;
  std::shared_ptr<factor_adders::VoSmartProjectionFactorAdder<node_adders::CombinedNavStateNodeAdder>>
    vo_smart_projection_factor_adder_;
  std::shared_ptr<factor_adders::StandstillFactorAdder<node_adders::CombinedNavStateNodeAdder>>
    standstill_factor_adder_;

  // Node Adders
  //  std::shared_ptr<node_adders::CombinedNavStateNodeAdder> combined_nav_state_node_adder_;
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_GRAPH_VIO_H_
