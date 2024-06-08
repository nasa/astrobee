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

#include <config_reader/config_reader.h>
#include <imu_integration/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/imu_measurement.h>
#include <localization_measurements/measurement_conversions.h>
#include <msg_conversions/msg_conversions.h>
#include <ros_pose_extrapolator/parameter_reader.h>
#include <ros_pose_extrapolator/ros_pose_extrapolator_wrapper.h>

namespace ros_pose_extrapolator {
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

RosPoseExtrapolatorWrapper::RosPoseExtrapolatorWrapper(const std::string& graph_config_path_prefix) {
  config_reader::ConfigReader config;
  config.AddFile("localization/imu_filter.config");
  config.AddFile("localization/imu_integrator.config");
  config.AddFile("localization/ros_pose_extrapolator.config");
  config.AddFile("transforms.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  RosPoseExtrapolatorParams params;
  LoadRosPoseExtrapolatorParams(config, params);
  Initialize(params);
}

RosPoseExtrapolatorWrapper::RosPoseExtrapolatorWrapper(const RosPoseExtrapolatorParams& params) { Initialize(params); }

void RosPoseExtrapolatorWrapper::Initialize(const RosPoseExtrapolatorParams& params) {
  params_ = params;
  imu_integrator_.reset(new ii::ImuIntegrator(params_.imu_integrator));
  odom_interpolator_ = lc::PoseInterpolater(params_.max_relative_vio_buffer_size);

  // Preintegration_helper_ is only being used to frame change and remove centrifugal acceleration, so body_T_imu is the
  // only parameter needed.
  boost::shared_ptr<gtsam::PreintegrationParams> preintegration_params(new gtsam::PreintegrationParams());
  preintegration_params->setBodyPSensor(params_.imu_integrator.body_T_imu);
  preintegration_helper_.reset(new gtsam::TangentPreintegration(preintegration_params, gtsam::imuBias::ConstantBias()));
}

void RosPoseExtrapolatorWrapper::GraphVIOStateCallback(const ff_msgs::GraphVIOState& graph_vio_state_msg) {
  latest_vio_msg_ = graph_vio_state_msg;
  const auto& latest_state_msg = graph_vio_state_msg.combined_nav_states.combined_nav_states.back();
  const auto latest_vio_state = lc::CombinedNavStateFromMsg(latest_state_msg);
  latest_vio_state_covariances_ = lc::CombinedNavStateCovariancesFromMsg(latest_state_msg);
  // Reset extrapolated vio state when receive new VIO message so IMU extrapolation
  // restarts using this state.
  latest_extrapolated_vio_state_ = latest_vio_state;
  odom_interpolator_.Add(latest_vio_state.timestamp(), lc::EigenPose(latest_vio_state.pose()));
  standstill_ = graph_vio_state_msg.standstill;
  // Remove old measurements no longer needed for extrapolation.
  imu_integrator_->RemoveOldValues(latest_vio_state.timestamp());
}

void RosPoseExtrapolatorWrapper::LocalizationStateCallback(const ff_msgs::GraphLocState& loc_msg) {
  latest_loc_msg_ = loc_msg;
  const auto world_T_body = lc::PoseFromMsg(loc_msg.pose.pose);
  // TODO(rsoussan): Store and use loc covariances
  const auto loc_timestamp = lc::TimeFromHeader(loc_msg.header);
  // Odom interpolator needs lower bound estimate for interpolation of first pose.
  odom_interpolator_.RemoveBelowLowerBoundValues(loc_timestamp);
  // Update world_T_odom estimate. Fix drift in odometry by getting odom_T_body
  // at the same timestamp as world_T_body and computing offset wrt world frame.
  const auto loc_timestamp_odom_T_body = odom_interpolator_.Interpolate(loc_timestamp);
  if (!loc_timestamp_odom_T_body) {
    LogError("LocalizationStateCallback: Failed to get odom_T_body for provided loc time.");
    return;
  }
  const auto loc_timestamp_body_T_odom = lc::GtPose(*loc_timestamp_odom_T_body).inverse();
  world_T_odom_ = world_T_body * loc_timestamp_body_T_odom;
}

bool RosPoseExtrapolatorWrapper::standstill() const {
  if (!params_.standstill_enabled) return false;
  // If uninitialized, return not at standstill
  if (!standstill_) return false;
  return *standstill_;
}

void RosPoseExtrapolatorWrapper::ImuCallback(const sensor_msgs::Imu& imu_msg) {
  imu_integrator_->AddImuMeasurement(lm::ImuMeasurement(imu_msg));
}

void RosPoseExtrapolatorWrapper::FlightModeCallback(const ff_msgs::FlightMode& flight_mode) {
  imu_integrator_->SetFanSpeedMode(lm::ConvertFanSpeedMode(flight_mode.speed));
}

boost::optional<std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>>
RosPoseExtrapolatorWrapper::LatestExtrapolatedStateAndCovariances() {
  if (!world_T_odom_ || !latest_extrapolated_vio_state_) {
    LogWarningEveryN(200,
                     "LatestExtrapolatedStateAndCovariances: Not enough information available to create desired data.");
    return boost::none;
  }

  // Extrapolate VIO data with latest IMU measurements.
  // Don't add IMU data if at standstill to avoid adding noisy IMU measurements to
  // extrapolated state. Avoid adding IMU data if too few measurements ( < 2) are in imu integrator.
  if (!standstill() && static_cast<int>(imu_integrator_->size()) > 1) {
    const auto latest_extrapolated_state = imu_integrator_->ExtrapolateLatest(*latest_extrapolated_vio_state_);
    if (!latest_extrapolated_state) {
      LogError("LatestExtrapolatedCombinedNavStateAndCovariances: Failed to extrapolate latest vio state.");
      return boost::none;
    }
    latest_extrapolated_vio_state_ = *latest_extrapolated_state;
  }

  // Convert from odom frame to world frame
  const gtsam::Pose3 extrapolated_world_T_body = (*world_T_odom_) * latest_extrapolated_vio_state_->pose();
  // Rotate body velocity from odom frame to world frame.
  const gtsam::Vector3 extrapolated_world_F_body_velocity =
    world_T_odom_->rotation() * latest_extrapolated_vio_state_->velocity();
  // Use latest bias estimate and use latest IMU time as extrapolated timestamp if available.
  // Even if at standstill, the timestamp should be the latest one available.
  const auto timestamp = imu_integrator_->LatestTimestamp() ? *(imu_integrator_->LatestTimestamp())
                                                            : latest_extrapolated_vio_state_->timestamp();
  const lc::CombinedNavState extrapolated_state(extrapolated_world_T_body, extrapolated_world_F_body_velocity,
                                                latest_extrapolated_vio_state_->bias(), timestamp);

  // TODO(rsoussan): propogate uncertainties from imu integrator, odom_interpolator, and localization estimate
  return std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances>{extrapolated_state,
                                                                          *latest_vio_state_covariances_};
}

boost::optional<ff_msgs::EkfState> RosPoseExtrapolatorWrapper::LatestExtrapolatedLocalizationMsg() {
  if (!latest_loc_msg_ || !latest_vio_msg_) {
    LogDebugEveryN(200, "LatestExtrapolatedLocalizationMsg: No latest loc msg available.");
    return boost::none;
  }

  const auto latest_extrapolated_state_and_covariances = LatestExtrapolatedStateAndCovariances();
  if (!latest_extrapolated_state_and_covariances) {
    LogWarningEveryN(
      200, "LatestExtrapolatedLocalizationMsg: Failed to get latest imu augmented nav state and covariances.");
    return boost::none;
  }

  // Get feature counts and other info from latest_loc_msg
  ff_msgs::EkfState latest_extrapolated_loc_msg;
  latest_extrapolated_loc_msg.header = latest_loc_msg_->header;
  latest_extrapolated_loc_msg.child_frame_id = latest_loc_msg_->child_frame_id;
  // Controller can't handle a confidence other than 0.
  latest_extrapolated_loc_msg.confidence = 0;
  // Prevent overflow of uin8_t
  latest_extrapolated_loc_msg.of_count = latest_vio_msg_->num_of_factors <= 255 ? latest_vio_msg_->num_of_factors : 255;
  latest_extrapolated_loc_msg.ml_count =
    latest_loc_msg_->num_ml_projection_factors <= 255 ? latest_loc_msg_->num_ml_projection_factors : 255;
  latest_extrapolated_loc_msg.estimating_bias = latest_vio_msg_->estimating_bias;

  // Update nav state and covariances with latest imu measurements
  lc::CombinedNavStateToLocMsg(latest_extrapolated_state_and_covariances->first, latest_extrapolated_loc_msg);
  lc::CombinedNavStateCovariancesToLocMsg(latest_extrapolated_state_and_covariances->second,
                                          latest_extrapolated_loc_msg);

  // Add latest bias corrected acceleration and angular velocity to loc msg
  const auto latest_imu_measurement = imu_integrator_->Latest();
  if (!latest_imu_measurement) {
    LogError("LatestExtrapolatedLocalizationMsg: Failed to get latest measurement.");
    return boost::none;
  }
  const auto& latest_bias = latest_extrapolated_state_and_covariances->first.bias();
  auto latest_bias_corrected_acceleration =
    latest_bias.correctAccelerometer(latest_imu_measurement->value.acceleration);
  const auto latest_bias_corrected_angular_velocity =
    latest_bias.correctGyroscope(latest_imu_measurement->value.angular_velocity);
  // Correct for gravity if needed
  if (!params_.imu_integrator.gravity.isZero()) {
    const gtsam::Pose3& global_T_body_latest = latest_extrapolated_state_and_covariances->first.pose();
    latest_bias_corrected_acceleration =
      lc::RemoveGravityFromAccelerometerMeasurement(params_.imu_integrator.gravity, params_.imu_integrator.body_T_imu,
                                                    global_T_body_latest, latest_bias_corrected_acceleration);
  }
  // Frame change measurements to body frame, correct for centripetal accel
  const auto corrected_measurements = preintegration_helper_->correctMeasurementsBySensorPose(
    latest_bias_corrected_acceleration, latest_bias_corrected_angular_velocity);

  mc::VectorToMsg(corrected_measurements.first, latest_extrapolated_loc_msg.accel);
  mc::VectorToMsg(corrected_measurements.second, latest_extrapolated_loc_msg.omega);
  return latest_extrapolated_loc_msg;
}
}  // namespace ros_pose_extrapolator
