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

#include <graph_localizer/parameter_reader.h>
#include <localization_common/utilities.h>

namespace graph_localizer {
namespace lc = localization_common;
void LoadCalibrationParams(config_reader::ConfigReader& config, CalibrationParams& params) {
  params.body_T_dock_cam = lc::LoadTransform(config, "dock_cam_transform");
  params.body_T_nav_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.world_T_dock = lc::LoadTransform(config, "world_dock_transform");
  params.nav_cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam")));
  params.dock_cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "dock_cam")));
}

void LoadFactorParams(config_reader::ConfigReader& config, FactorParams& params) {
  params.min_valid_feature_track_avg_distance_from_mean =
      lc::LoadDouble(config, "min_valid_feature_track_avg_distance_from_mean");
  params.max_standstill_feature_track_avg_distance_from_mean =
      lc::LoadDouble(config, "max_standstill_feature_track_avg_distance_from_mean");
  params.optical_flow_standstill_pose_prior = lc::LoadBool(config, "optical_flow_standstill_pose_prior");
  params.optical_flow_standstill_velocity_prior = lc::LoadBool(config, "optical_flow_standstill_velocity_prior");
}

void LoadImuIntegrationParams(config_reader::ConfigReader& config, GraphInitializationParams& params) {
  params.gravity = lc::LoadVector3(config, "world_gravity_vector");
  params.body_T_imu = lc::LoadTransform(config, "imu_transform");
}

void LoadGraphValuesParams(config_reader::ConfigReader& config, GraphValuesParams& params) {
  params.sliding_window_duration = lc::LoadDouble(config, "sliding_window_duration");
  params.min_num_sliding_window_states = lc::LoadInt(config, "min_num_sliding_window_states");
}

void LoadNoiseParams(config_reader::ConfigReader& config, NoiseParams& params) {
  params.dock_cam_noise = gtsam::noiseModel::Isotropic::Sigma(2, lc::LoadDouble(config, "dock_cam_noise_stddev"));
  params.nav_cam_noise = gtsam::noiseModel::Isotropic::Sigma(2, lc::LoadDouble(config, "nav_cam_noise_stddev"));
  params.optical_flow_prior_translation_stddev = lc::LoadDouble(config, "optical_flow_prior_translation_stddev");
  params.optical_flow_prior_quaternion_stddev = lc::LoadDouble(config, "optical_flow_prior_quaternion_stddev");
  params.optical_flow_prior_velocity_stddev = lc::LoadDouble(config, "optical_flow_prior_velocity_stddev");
}

void LoadGraphLocalizerParams(config_reader::ConfigReader& config, GraphLocalizerParams& params) {
  LoadCalibrationParams(config, params.calibration);
  LoadImuIntegrationParams(config, params.graph_initialization);
  LoadFactorParams(config, params.factor);
  LoadGraphValuesParams(config, params.graph_values);
  LoadNoiseParams(config, params.noise);
}
}  // namespace graph_localizer
