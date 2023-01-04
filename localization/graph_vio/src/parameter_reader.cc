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

#include <graph_vio/parameter_reader.h>
#include <graph_vio/utilities.h>
#include <graph_optimizer/parameter_reader.h>
#include <imu_integration/utilities.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

namespace graph_vio {
namespace fa = factor_adders;
namespace go = graph_optimizer;
namespace ii = imu_integration;
namespace lc = localization_common;
namespace mc = msg_conversions;
namespace vc = vision_common;

// TODO(rsoussan): remove this?
void LoadCalibrationParams(config_reader::ConfigReader& config, CalibrationParams& params) {
  params.nav_cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam")));
}

void LoadFactorParams(config_reader::ConfigReader& config, FactorParams& params) {
  LoadProjectionFactorAdderParams(config, params.projection_adder);
  LoadSmartProjectionFactorAdderParams(config, params.smart_projection_adder);
  LoadStandstillFactorAdderParams(config, params.standstill_adder);
}

void LoadProjectionFactorAdderParams(config_reader::ConfigReader& config, fa::ProjectionFactorAdderParams& params) {
  params.enabled = mc::LoadBool(config, "projection_adder_enabled");
  params.huber_k = mc::LoadDouble(config, "huber_k");
  params.enable_EPI = mc::LoadBool(config, "projection_adder_enable_EPI");
  params.landmark_distance_threshold = mc::LoadDouble(config, "projection_adder_landmark_distance_threshold");
  params.dynamic_outlier_rejection_threshold =
    mc::LoadDouble(config, "projection_adder_dynamic_outlier_rejection_threshold");
  params.max_num_features = mc::LoadInt(config, "projection_adder_max_num_features");
  params.min_num_measurements_for_triangulation =
    mc::LoadInt(config, "projection_adder_min_num_measurements_for_triangulation");
  params.add_point_priors = mc::LoadBool(config, "projection_adder_add_point_priors");
  params.point_prior_translation_stddev = mc::LoadDouble(config, "projection_adder_point_prior_translation_stddev");
  params.body_T_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam")));
  params.cam_noise =
    gtsam::noiseModel::Isotropic::Sigma(2, mc::LoadDouble(config, "optical_flow_nav_cam_noise_stddev"));
}

void LoadSmartProjectionFactorAdderParams(config_reader::ConfigReader& config,
                                          fa::SmartProjectionFactorAdderParams& params) {
  params.enabled = mc::LoadBool(config, "smart_projection_adder_enabled");
  params.huber_k = mc::LoadDouble(config, "huber_k");
  params.min_avg_distance_from_mean = mc::LoadDouble(config, "smart_projection_adder_min_avg_distance_from_mean");
  params.enable_EPI = mc::LoadBool(config, "smart_projection_adder_enable_EPI");
  params.landmark_distance_threshold = mc::LoadDouble(config, "smart_projection_adder_landmark_distance_threshold");
  params.dynamic_outlier_rejection_threshold =
    mc::LoadDouble(config, "smart_projection_adder_dynamic_outlier_rejection_threshold");
  params.retriangulation_threshold = mc::LoadDouble(config, "smart_projection_adder_retriangulation_threshold");
  params.verbose_cheirality = mc::LoadBool(config, "smart_projection_adder_verbose_cheirality");
  params.robust = mc::LoadBool(config, "smart_projection_adder_robust");
  params.max_num_factors = mc::LoadInt(config, "smart_projection_adder_max_num_factors");
  params.min_num_points = mc::LoadInt(config, "smart_projection_adder_min_num_points");
  params.max_num_points_per_factor = mc::LoadInt(config, "smart_projection_adder_max_num_points_per_factor");
  params.measurement_spacing = mc::LoadInt(config, "smart_projection_adder_measurement_spacing");
  params.feature_track_min_separation = mc::LoadDouble(config, "smart_projection_adder_feature_track_min_separation");
  params.rotation_only_fallback = mc::LoadBool(config, "smart_projection_adder_rotation_only_fallback");
  params.splitting = mc::LoadBool(config, "smart_projection_adder_splitting");
  params.scale_noise_with_num_points = mc::LoadBool(config, "smart_projection_adder_scale_noise_with_num_points");
  params.noise_scale = mc::LoadDouble(config, "smart_projection_adder_noise_scale");
  params.use_allowed_timestamps = mc::LoadBool(config, "smart_projection_adder_use_allowed_timestamps");
  params.body_T_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam")));
  params.cam_noise =
    gtsam::noiseModel::Isotropic::Sigma(2, mc::LoadDouble(config, "optical_flow_nav_cam_noise_stddev"));
}

void LoadStandstillFactorAdderParams(config_reader::ConfigReader& config, fa::StandstillFactorAdderParams& params) {
  params.add_velocity_prior = mc::LoadBool(config, "standstill_adder_add_velocity_prior");
  params.add_pose_between_factor = mc::LoadBool(config, "standstill_adder_add_pose_between_factor");
  params.enabled = params.add_velocity_prior || params.add_pose_between_factor;
  params.prior_velocity_stddev = mc::LoadDouble(config, "standstill_adder_prior_velocity_stddev");
  params.pose_between_factor_translation_stddev =
    mc::LoadDouble(config, "standstill_adder_pose_between_factor_translation_stddev");
  params.pose_between_factor_rotation_stddev =
    mc::LoadDouble(config, "standstill_adder_pose_between_factor_rotation_stddev");
  params.huber_k = mc::LoadDouble(config, "huber_k");
}

void LoadFeatureTrackerParams(config_reader::ConfigReader& config, vc::FeatureTrackerParams& params) {
  params.sliding_window_duration = mc::LoadDouble(config, "feature_tracker_sliding_window_duration");
  params.smart_projection_adder_measurement_spacing = mc::LoadInt(config, "smart_projection_adder_measurement_spacing");
}

void LoadSanityCheckerParams(config_reader::ConfigReader& config, SanityCheckerParams& params) {
  params.check_position_covariance = mc::LoadBool(config, "check_position_covariance");
  params.check_orientation_covariance = mc::LoadBool(config, "check_orientation_covariance");
  params.position_covariance_threshold = mc::LoadDouble(config, "position_covariance_threshold");
  params.orientation_covariance_threshold = mc::LoadDouble(config, "orientation_covariance_threshold");
}

void LoadGraphInitializerParams(config_reader::ConfigReader& config, GraphInitializerParams& params) {
  ii::LoadImuIntegratorParams(config, params);
  params.imu_bias_filename = mc::LoadString(config, "imu_bias_file");
  params.num_bias_estimation_measurements = mc::LoadInt(config, "num_bias_estimation_measurements");
}

void LoadCombinedNavStateNodeUpdaterParams(config_reader::ConfigReader& config,
                                           CombinedNavStateNodeUpdaterParams& params) {
  // TODO(rsoussan): Still need these?
  params.starting_prior_translation_stddev = mc::LoadDouble(config, "starting_prior_translation_stddev");
  params.starting_prior_quaternion_stddev = mc::LoadDouble(config, "starting_prior_quaternion_stddev");
  params.starting_prior_velocity_stddev = mc::LoadDouble(config, "starting_prior_velocity_stddev");
  params.starting_prior_accel_bias_stddev = mc::LoadDouble(config, "starting_prior_accel_bias_stddev");
  params.starting_prior_gyro_bias_stddev = mc::LoadDouble(config, "starting_prior_gyro_bias_stddev");
  params.huber_k = mc::LoadDouble(config, "huber_k");
  params.add_priors = mc::LoadBool(config, "add_priors");
  params.threshold_bias_uncertainty = mc::LoadBool(config, "threshold_bias_uncertainty");
  params.accel_bias_stddev_threshold = mc::LoadDouble(config, "accel_bias_stddev_threshold");
  params.gyro_bias_stddev_threshold = mc::LoadDouble(config, "gyro_bias_stddev_threshold");
  LoadCombinedNavStateGraphValuesParams(config, params.graph_values);
}

void LoadCombinedNavStateGraphValuesParams(config_reader::ConfigReader& config,
                                           gv::CombinedNavStateGraphValuesParams& params) {
  params.ideal_duration = mc::LoadDouble(config, "ideal_duration");
  params.min_num_states = mc::LoadInt(config, "min_num_states");
  params.max_num_states = mc::LoadInt(config, "max_num_states");
}

void LoadFeaturePointNodeUpdaterParams(config_reader::ConfigReader& config, FeaturePointNodeUpdaterParams& params) {
  params.huber_k = mc::LoadDouble(config, "huber_k");
}

void LoadGraphVIOParams(config_reader::ConfigReader& config, GraphVIOParams& params) {
  LoadCalibrationParams(config, params.calibration);
  LoadCombinedNavStateNodeUpdaterParams(config, params.combined_nav_state_node_updater);
  LoadGraphInitializerParams(config, params.graph_initializer);
  LoadFactorParams(config, params.factor);
  LoadFeaturePointNodeUpdaterParams(config, params.feature_point_node_updater);
  LoadFeatureTrackerParams(config, params.feature_tracker);
  go::LoadGraphOptimizerParams(config, params.graph_optimizer);
  params.huber_k = mc::LoadDouble(config, "huber_k");
  params.max_standstill_feature_track_avg_distance_from_mean =
    mc::LoadDouble(config, "max_standstill_feature_track_avg_distance_from_mean");
  params.standstill_min_num_points_per_track = mc::LoadInt(config, "standstill_min_num_points_per_track");
  params.standstill_feature_track_duration = mc::LoadDouble(config, "standstill_feature_track_duration");
}

void LoadGraphVIONodeletParams(config_reader::ConfigReader& config, GraphVIONodeletParams& params) {
  params.max_imu_buffer_size = mc::LoadInt(config, "max_imu_buffer_size");
  params.max_optical_flow_buffer_size = mc::LoadInt(config, "max_optical_flow_buffer_size");
}
}  // namespace graph_vio
