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

#include <graph_vio/graph_vio.h>
#include <vision_common/utilities.h>

namespace graph_vio {
namespace fa = factor_adders;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace na = node_adders;
namespace no = nodes;
namespace op = optimizers;
namespace vc = vision_common;

GraphVIO::GraphVIO(const GraphVIOParams& params)
    : SlidingWindowGraphOptimizer(params.sliding_window_graph_optimizer,
                                  std::make_unique<op::NonlinearOptimizer>(params.nonlinear_optimizer)),
      params_(params),
      standstill_(false) {
  // Initialize sliding window node adders
  combined_nav_state_node_adder_ = std::make_shared<na::CombinedNavStateNodeAdder>(
    params_.combined_nav_state_node_adder, params_.combined_nav_state_node_adder_model, values());
  AddSlidingWindowNodeAdder(combined_nav_state_node_adder_);
  // Initialize factor adders
  vo_smart_projection_factor_adder_ = std::make_shared<fa::VoSmartProjectionFactorAdder<na::CombinedNavStateNodeAdder>>(
    params_.vo_smart_projection_factor_adder, combined_nav_state_node_adder_);
  AddFactorAdder(vo_smart_projection_factor_adder_);
  standstill_factor_adder_ = std::make_shared<fa::StandstillFactorAdder<na::CombinedNavStateNodeAdder>>(
    params_.standstill_factor_adder, combined_nav_state_node_adder_);
  AddFactorAdder(standstill_factor_adder_);
  depth_odometry_factor_adder_ = std::make_shared<fa::DepthOdometryFactorAdder<na::CombinedNavStateNodeAdder>>(
    params_.depth_odometry_factor_adder, combined_nav_state_node_adder_);
  AddFactorAdder(depth_odometry_factor_adder_);
}

void GraphVIO::AddImuMeasurement(const lm::ImuMeasurement& imu_measurement) {
  combined_nav_state_node_adder_->AddMeasurement(imu_measurement);
}

void GraphVIO::SetFanSpeedMode(const localization_measurements::FanSpeedMode& fan_speed_mode) {
  combined_nav_state_node_adder_->node_adder_model().SetFanSpeedMode(fan_speed_mode);
}

void GraphVIO::AddFeaturePointsMeasurement(const lm::FeaturePointsMeasurement& feature_points_measurement) {
  if (params_.vo_smart_projection_factor_adder.enabled)
    vo_smart_projection_factor_adder_->AddMeasurement(feature_points_measurement);

  // Check for standstill and optionally add standstill measurement if detected.
  const auto& feature_tracks = vo_smart_projection_factor_adder_->feature_tracker().feature_tracks();
  standstill_ = vc::Standstill(feature_tracks, params_.standstill);
  if (standstill_ && params_.standstill_factor_adder.enabled) {
    const lc::Time latest_timestamp = feature_points_measurement.timestamp;
    const auto previous_timestamp = feature_tracks.cbegin()->second.SecondLatestTimestamp();
    standstill_factor_adder_->AddMeasurement(lm::StandstillMeasurement(latest_timestamp, *previous_timestamp));
  }
}

void GraphVIO::AddDepthOdometryMeasurement(
  const localization_measurements::DepthOdometryMeasurement& depth_odometry_measurement) {
  // Don't add depth odom measurements when at standstill to avoid
  // introducing pose and velocity noise
  if (params_.depth_odometry_factor_adder.enabled && !standstill_)
    depth_odometry_factor_adder_->AddMeasurement(depth_odometry_measurement);
}

const no::CombinedNavStateNodes& GraphVIO::combined_nav_state_nodes() const {
  return combined_nav_state_node_adder_->nodes();
}

bool GraphVIO::standstill() const { return standstill_; }

const vc::SpacedFeatureTracker& GraphVIO::feature_tracker() const {
  return vo_smart_projection_factor_adder_->feature_tracker();
}

std::shared_ptr<lc::MarginalsPoseCovarianceInterpolater<no::CombinedNavStateNodes>>
GraphVIO::MarginalsPoseCovarianceInterpolater() {
  return std::make_shared<lc::MarginalsPoseCovarianceInterpolater<no::CombinedNavStateNodes>>(
    combined_nav_state_node_adder_->nodes_ptr(), marginals());
}
}  // namespace graph_vio
