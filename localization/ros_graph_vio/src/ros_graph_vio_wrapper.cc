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
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>
#include <msg_conversions/msg_conversions.h>
#include <parameter_reader/graph_vio.h>
#include <ros_graph_vio/parameter_reader.h>
#include <ros_graph_vio/ros_graph_vio_wrapper.h>

#include <Eigen/Core>

namespace ros_graph_vio {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;
namespace pr = parameter_reader;

RosGraphVIOWrapper::RosGraphVIOWrapper(const std::string& graph_config_path_prefix) {
  LoadConfigs(graph_config_path_prefix);
}

void RosGraphVIOWrapper::LoadConfigs(const std::string& graph_config_path_prefix) {
  config_reader::ConfigReader config;
  config.AddFile("localization/imu_bias_initializer.config");
  config.AddFile("localization/imu_filter.config");
  config.AddFile("localization/ros_graph_vio.config");
  lc::LoadGraphVIOConfig(config, graph_config_path_prefix);
  pr::LoadGraphVIOParams(config, params_);
  LoadRosGraphVIOWrapperParams(config, wrapper_params_);
  ImuBiasInitializerParams imu_bias_initializer_params;
  LoadImuBiasInitializerParams(config, imu_bias_initializer_params);
  imu_bias_initializer_.reset(new ImuBiasInitializer(imu_bias_initializer_params));
}

void RosGraphVIOWrapper::FeaturePointsCallback(const ff_msgs::Feature2dArray& feature_array_msg) {
  if (Initialized()) graph_vio_->AddFeaturePointsMeasurement(lm::MakeFeaturePointsMeasurement(feature_array_msg));
  ++feature_point_count_;
}

void RosGraphVIOWrapper::DepthOdometryCallback(const ff_msgs::DepthOdometry& depth_odometry_msg) {
  if (Initialized()) graph_vio_->AddDepthOdometryMeasurement(lm::MakeDepthOdometryMeasurement(depth_odometry_msg));
}

void RosGraphVIOWrapper::ImuCallback(const sensor_msgs::Imu& imu_msg) {
  const auto imu_measurement = lm::ImuMeasurement(imu_msg);
  imu_bias_initializer_->AddImuMeasurement(imu_measurement);
  if (!Initialized() && imu_bias_initializer_->Bias()) {
    // Set initial nav state. Use bias from initializer and
    // assume zero initial velocity. Set initial pose to identity.
    const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                             imu_bias_initializer_->Bias()->bias, imu_measurement.timestamp);
    params_.combined_nav_state_node_adder.start_node = initial_state;
    params_.combined_nav_state_node_adder.starting_time = initial_state.timestamp();
    // Set starting state noise using params
    const gtsam::Vector6 pose_noise_sigmas(
      (gtsam::Vector(6) << wrapper_params_.starting_pose_translation_stddev,
       wrapper_params_.starting_pose_translation_stddev, wrapper_params_.starting_pose_translation_stddev,
       wrapper_params_.starting_pose_quaternion_stddev, wrapper_params_.starting_pose_quaternion_stddev,
       wrapper_params_.starting_pose_quaternion_stddev)
        .finished());
    const auto pose_noise =
      lc::Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_noise_sigmas)),
                 params_.combined_nav_state_node_adder.huber_k);
    params_.combined_nav_state_node_adder.start_noise_models.emplace_back(pose_noise);
    // Set starting velocity noise using accel bias stddev - assumes stddev in accel measurements correlate
    // to uncertainty in initial velocity
    const auto& accel_bias_stddev = imu_bias_initializer_->Bias()->accelerometer_bias_stddev;
    const auto& gyro_bias_stddev = imu_bias_initializer_->Bias()->gyro_bias_stddev;
    const gtsam::Vector3 velocity_noise_sigmas = accel_bias_stddev * wrapper_params_.starting_velocity_stddev_scale;
    const auto velocity_noise =
      lc::Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_noise_sigmas)),
                 params_.combined_nav_state_node_adder.huber_k);
    params_.combined_nav_state_node_adder.start_noise_models.emplace_back(velocity_noise);

    // Set starting bias noise using scaled calculated bias stddevs
    const gtsam::Vector6 bias_noise_sigmas(
      (gtsam::Vector(6) << accel_bias_stddev * wrapper_params_.starting_accel_bias_stddev_scale,
       gyro_bias_stddev * wrapper_params_.starting_gyro_bias_stddev_scale)
        .finished());
    const auto bias_noise =
      lc::Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(bias_noise_sigmas)),
                 params_.combined_nav_state_node_adder.huber_k);
    params_.combined_nav_state_node_adder.start_noise_models.emplace_back(bias_noise);
    graph_vio_.reset(new graph_vio::GraphVIO(params_));
    LogInfo("ImuCallback: Initialized VIO.");
  }
  if (Initialized()) graph_vio_->AddImuMeasurement(imu_measurement);
}

void RosGraphVIOWrapper::FlightModeCallback(const ff_msgs::FlightMode& flight_mode) {
  const auto fan_speed_mode = lm::ConvertFanSpeedMode(flight_mode.speed);
  if (Initialized()) graph_vio_->SetFanSpeedMode(fan_speed_mode);
  imu_bias_initializer_->AddFanSpeedModeMeasurement(fan_speed_mode);
}

void RosGraphVIOWrapper::Update() {
  if (Initialized()) graph_vio_->Update();
}

bool RosGraphVIOWrapper::Initialized() const { return graph_vio_ != nullptr; }

void RosGraphVIOWrapper::ResetVIO() {
  LogInfo("ResetVIO: Resetting vio.");
  if (!Initialized()) {
    LogError("ResetVIO: VIO not initialized, nothing to do.");
    return;
  }
  const auto latest_combined_nav_state = graph_vio_->combined_nav_state_nodes().LatestNode();
  if (!latest_combined_nav_state) {
    LogError("ResetVIO: Failed to get latest combined nav state.");
    return;
  }
  const auto keys = graph_vio_->combined_nav_state_nodes().Keys(latest_combined_nav_state->timestamp());
  if (keys.empty() || keys.size() != 3) {
    LogError("ResetVIO: Failed to get latest keys.");
    return;
  }

  const auto imu_bias_covariance = graph_vio_->Covariance(keys[2]);
  if (!imu_bias_covariance) {
    LogError("ResetVIO: Failed to get latest combined nav state and IMU bias covariance.");
    return;
  }
  const Eigen::Vector3d accelerometer_bias_stddev(std::sqrt((*imu_bias_covariance)(0, 0)),
                                                  std::sqrt((*imu_bias_covariance)(1, 1)),
                                                  std::sqrt((*imu_bias_covariance)(2, 2)));
  const Eigen::Vector3d gyro_bias_stddev(std::sqrt((*imu_bias_covariance)(3, 3)),
                                         std::sqrt((*imu_bias_covariance)(4, 4)),
                                         std::sqrt((*imu_bias_covariance)(5, 5)));
  imu_bias_initializer_->UpdateBias(
    ImuBiasWithStddev(latest_combined_nav_state->bias(), accelerometer_bias_stddev, gyro_bias_stddev));
  graph_vio_.reset();
}

void RosGraphVIOWrapper::ResetBiasesAndVIO() {
  LogInfo("ResetBiasAndVIO: Resetting biases and vio.");
  imu_bias_initializer_->Reset();
  graph_vio_.reset();
}

void RosGraphVIOWrapper::ResetBiasesFromFileAndResetVIO() {
  LogInfo("ResetBiasAndVIO: Resetting biases from file and resetting vio.");
  imu_bias_initializer_->LoadFromFile();
  graph_vio_.reset();
}

boost::optional<ff_msgs::GraphVIOState> RosGraphVIOWrapper::GraphVIOStateMsg() {
  if (!Initialized()) {
    LogDebugEveryN(200, "Graph VIO not yet initialized.");
    return boost::none;
  }
  const auto& nodes = graph_vio_->combined_nav_state_nodes();
  const auto times = nodes.Timestamps();
  // Avoid sending msgs before enough optical flow measurements have been incorporated in the graph
  if (times.size() < 3) {
    LogWarningEveryN(200, "Too few nodes in Graph VIO, waiting for more optical flow measurements.");
    return boost::none;
  }

  // Avoid sending repeat msgs
  if (times.empty() || (latest_msg_time_ && times.back() == *latest_msg_time_)) {
    LogDebugEveryN(2000, "No new VIO states.");
    return boost::none;
  }
  const lc::Time latest_time = times.back();
  ff_msgs::GraphVIOState msg;

  // Only add latest state since this is used for localization relative factors
  const auto combined_nav_state = nodes.Node(latest_time);
  const auto keys = nodes.Keys(latest_time);
  if (!combined_nav_state || keys.empty() || keys.size() != 3) {
    LogError("CombinedNavStateArrayMsg: Failed to get combined nav state and keys.");
    return boost::none;
  }
  const auto pose_covariance = graph_vio_->Covariance(keys[0]);
  const auto velocity_covariance = graph_vio_->Covariance(keys[1]);
  const auto imu_bias_covariance = graph_vio_->Covariance(keys[2]);
  if (!pose_covariance || !velocity_covariance || !imu_bias_covariance) {
    LogError("CombinedNavStateArrayMsg: Failed to get combined nav state covariances.");
    return boost::none;
  }

  msg.combined_nav_states.combined_nav_states.push_back(
    lc::CombinedNavStateToMsg(*combined_nav_state, *pose_covariance, *velocity_covariance, *imu_bias_covariance));

  lc::TimeToHeader(*(nodes.LatestTimestamp()), msg.header);
  msg.child_frame_id = "odom";
  msg.standstill = graph_vio_->standstill();
  msg.estimating_bias = !Initialized() && !imu_bias_initializer_->Bias();
  msg.num_detected_of_features = graph_vio_->feature_tracker().size();
  msg.num_of_factors = graph_vio_->NumFactors<factor_adders::RobustSmartFactor>();
  msg.num_depth_factors = graph_vio_->NumFactors<gtsam::PointToPointBetweenFactor>();
  msg.optimization_iterations = graph_vio_->optimization_iterations_averager().last_value();
  msg.optimization_time = graph_vio_->optimization_timer().last_value();
  msg.update_time = graph_vio_->update_timer().last_value();
  // Divide values by three since each state contains three values (pose, velocity, biases)
  msg.num_states = graph_vio_->num_values() / 3.0;
  msg.duration = graph_vio_->Duration();
  latest_msg_time_ = latest_time;
  return msg;
}

const std::unique_ptr<graph_vio::GraphVIO>& RosGraphVIOWrapper::graph_vio() const { return graph_vio_; }

std::unique_ptr<graph_vio::GraphVIO>& RosGraphVIOWrapper::graph_vio() { return graph_vio_; }
}  // namespace ros_graph_vio
