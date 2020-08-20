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

#include <graph_localizer/graph_loc_initialization.h>
#include <graph_localizer/utilities.h>

#include <iostream>

namespace graph_localizer {
void GraphLocInitialization::SetBiases(const Eigen::Vector3d& accelerometer_bias, const Eigen::Vector3d& gyro_bias) {
  params_.SetBiases(accelerometer_bias, gyro_bias);
  has_biases_ = true;
  estimate_biases_ = false;
}

void GraphLocInitialization::SetStartPose(const Eigen::Isometry3d& global_T_body_start, const double timestamp) {
  params_.SetStartPose(global_T_body_start, timestamp);
  has_start_pose_ = true;
}

void GraphLocInitialization::SetCalibration(const Eigen::Isometry3d& body_T_imu,
                                            const Eigen::Isometry3d& body_T_nav_cam,
                                            const Eigen::Matrix3d& nav_cam_intrinsics,
                                            const Eigen::Isometry3d& body_T_dock_cam,
                                            const Eigen::Matrix3d& dock_cam_intrinsics,
                                            const Eigen::Isometry3d& world_T_dock, const Eigen::Vector3d& gravity) {
  params_.SetCalibration(body_T_imu, body_T_nav_cam, nav_cam_intrinsics, body_T_dock_cam, dock_cam_intrinsics,
                         world_T_dock, gravity);
  has_calibration_ = true;
}

void GraphLocInitialization::ResetBiasesAndStartPose() {
  ResetBiases();
  ResetStartPose();
}

void GraphLocInitialization::ResetStartPose() { has_start_pose_ = false; }

void GraphLocInitialization::ResetBiases() {
  has_biases_ = false;
  StartBiasEstimation();
}

void GraphLocInitialization::LoadSensorParams(config_reader::ConfigReader& config) {
  const Eigen::Isometry3d body_T_nav_cam = LoadTransform(config, "nav_cam_transform");
  const Eigen::Isometry3d body_T_dock_cam = LoadTransform(config, "dock_cam_transform");
  const camera::CameraParameters nav_cam_params(&config, "nav_cam");
  const camera::CameraParameters dock_cam_params(&config, "dock_cam");
  const Eigen::Isometry3d world_T_dock = LoadTransform(config, "world_dock_transform");
  const Eigen::Isometry3d body_T_imu = LoadTransform(config, "imu_transform");
  Eigen::Vector3d gravity;
  msg_conversions::config_read_vector(&config, "world_gravity_vector", &gravity);
  SetCalibration(body_T_imu, body_T_nav_cam, nav_cam_params.GetIntrinsicMatrix<camera::UNDISTORTED_C>(),
                 body_T_dock_cam, dock_cam_params.GetIntrinsicMatrix<camera::UNDISTORTED_C>(), world_T_dock, gravity);
}

void GraphLocInitialization::LoadGraphLocalizerParams(config_reader::ConfigReader& config) {
  double sliding_window_duration;
  if (!config.GetReal("sliding_window_duration", &sliding_window_duration))
    LOG(FATAL) << "Failed to load sliding window duration.";

  int min_num_sliding_window_states;
  if (!config.GetInt("min_num_sliding_window_states", &min_num_sliding_window_states))
    LOG(FATAL) << "Failed to load min_num_sliding_window_states.";

  double min_of_avg_distance_from_mean;
  if (!config.GetReal("min_of_avg_distance_from_mean", &min_of_avg_distance_from_mean))
    LOG(FATAL) << "Failed to load min_of_avg_distance_from_mean.";

  params_.SetParams(sliding_window_duration, min_num_sliding_window_states, min_of_avg_distance_from_mean);
  has_params_ = true;
}

bool GraphLocInitialization::ReadyToInitialize() const {
  return HasBiases() && HasStartPose() && HasCalibration() && HasParams();
}

void GraphLocInitialization::StartBiasEstimation() { estimate_biases_ = true; }

bool GraphLocInitialization::HasBiases() const { return has_biases_; }
bool GraphLocInitialization::HasStartPose() const { return has_start_pose_; }
bool GraphLocInitialization::HasCalibration() const { return has_calibration_; }
bool GraphLocInitialization::HasParams() const { return has_params_; }
bool GraphLocInitialization::EstimateBiases() const { return estimate_biases_; }

const GraphLocalizerParams& GraphLocInitialization::params() const { return params_; }

}  // namespace graph_localizer
