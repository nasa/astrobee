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
#include <graph_localizer/utilities.h>
#include <imu_integration/utilities.h>
#include <localization_common/utilities.h>

namespace graph_localizer {
namespace ii = imu_integration;
namespace lc = localization_common;
void LoadCalibrationParams(config_reader::ConfigReader& config, CalibrationParams& params) {
  params.body_T_dock_cam = lc::LoadTransform(config, "dock_cam_transform");
  params.body_T_nav_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.world_T_dock = lc::LoadTransform(config, "world_dock_transform");
  params.nav_cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam")));
  params.dock_cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "dock_cam")));
}

void LoadFactorParams(config_reader::ConfigReader& config, FactorParams& params) {
  params.optical_flow_standstill_velocity_prior = lc::LoadBool(config, "optical_flow_standstill_velocity_prior");
  LoadLocFactorAdderParams(config, params.loc_adder);
  LoadARTagLocFactorAdderParams(config, params.ar_tag_loc_adder);
  LoadRotationFactorAdderParams(config, params.rotation_adder);
  LoadProjectionFactorAdderParams(config, params.projection_adder);
  LoadSmartProjectionFactorAdderParams(config, params.smart_projection_adder);
}

void LoadARTagLocFactorAdderParams(config_reader::ConfigReader& config, LocFactorAdderParams& params) {
  params.add_pose_priors = lc::LoadBool(config, "loc_adder_add_pose_priors");
  params.add_projections = lc::LoadBool(config, "loc_adder_add_projections");
  params.enabled = params.add_pose_priors || params.add_projections ? true : false;
  params.min_num_matches = lc::LoadInt(config, "loc_adder_min_num_matches");
  params.prior_translation_stddev = lc::LoadDouble(config, "loc_adder_prior_translation_stddev");
  params.prior_quaternion_stddev = lc::LoadDouble(config, "loc_adder_prior_quaternion_stddev");
  params.body_T_cam = lc::LoadTransform(config, "dock_cam_transform");
  params.cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "dock_cam")));
  params.cam_noise = gtsam::noiseModel::Isotropic::Sigma(2, lc::LoadDouble(config, "loc_dock_cam_noise_stddev"));
}

void LoadLocFactorAdderParams(config_reader::ConfigReader& config, LocFactorAdderParams& params) {
  params.add_pose_priors = lc::LoadBool(config, "loc_adder_add_pose_priors");
  params.add_projections = lc::LoadBool(config, "loc_adder_add_projections");
  params.enabled = params.add_pose_priors || params.add_projections ? true : false;
  params.min_num_matches = lc::LoadInt(config, "loc_adder_min_num_matches");
  params.prior_translation_stddev = lc::LoadDouble(config, "loc_adder_prior_translation_stddev");
  params.prior_quaternion_stddev = lc::LoadDouble(config, "loc_adder_prior_quaternion_stddev");
  params.scale_noise_with_num_landmarks = lc::LoadBool(config, "loc_adder_scale_noise_with_num_landmarks");
  params.noise_scale = lc::LoadDouble(config, "loc_adder_noise_scale");
  params.body_T_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam")));
  params.cam_noise = gtsam::noiseModel::Isotropic::Sigma(2, lc::LoadDouble(config, "loc_nav_cam_noise_stddev"));
}

void LoadRotationFactorAdderParams(config_reader::ConfigReader& config, RotationFactorAdderParams& params) {
  params.enabled = lc::LoadBool(config, "rotation_adder_enabled");
  params.min_avg_disparity = lc::LoadDouble(config, "rotation_adder_min_avg_disparity");
  params.rotation_stddev = lc::LoadDouble(config, "rotation_adder_rotation_stddev");
  params.max_percent_outliers = lc::LoadDouble(config, "rotation_adder_max_percent_outliers");
  params.body_T_nav_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.nav_cam_intrinsics = lc::LoadCameraIntrinsics(config, "nav_cam");
}

void LoadProjectionFactorAdderParams(config_reader::ConfigReader& config, ProjectionFactorAdderParams& params) {
  params.enabled = lc::LoadBool(config, "projection_adder_enabled");
  params.enable_EPI = lc::LoadBool(config, "projection_adder_enable_EPI");
  params.landmark_distance_threshold = lc::LoadDouble(config, "projection_adder_landmark_distance_threshold");
  params.dynamic_outlier_rejection_threshold =
    lc::LoadDouble(config, "projection_adder_dynamic_outlier_rejection_threshold");
  params.max_num_features = lc::LoadInt(config, "projection_adder_max_num_features");
  params.min_num_measurements_for_triangulation =
    lc::LoadInt(config, "projection_adder_min_num_measurements_for_triangulation");
  params.add_point_priors = lc::LoadBool(config, "projection_adder_add_point_priors");
  params.point_prior_translation_stddev = lc::LoadDouble(config, "projection_adder_point_prior_translation_stddev");
  params.body_T_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam")));
  params.cam_noise =
    gtsam::noiseModel::Isotropic::Sigma(2, lc::LoadDouble(config, "optical_flow_nav_cam_noise_stddev"));
}

void LoadSmartProjectionFactorAdderParams(config_reader::ConfigReader& config,
                                          SmartProjectionFactorAdderParams& params) {
  params.enabled = lc::LoadBool(config, "smart_projection_adder_enabled");
  params.min_avg_distance_from_mean = lc::LoadDouble(config, "smart_projection_adder_min_avg_distance_from_mean");
  params.enable_EPI = lc::LoadBool(config, "smart_projection_adder_enable_EPI");
  params.landmark_distance_threshold = lc::LoadDouble(config, "smart_projection_adder_landmark_distance_threshold");
  params.dynamic_outlier_rejection_threshold =
    lc::LoadDouble(config, "smart_projection_adder_dynamic_outlier_rejection_threshold");
  params.retriangulation_threshold = lc::LoadDouble(config, "smart_projection_adder_retriangulation_threshold");
  params.verbose_cheirality = lc::LoadBool(config, "smart_projection_adder_verbose_cheirality");
  params.robust = lc::LoadBool(config, "smart_projection_adder_robust");
  params.max_num_factors = lc::LoadInt(config, "smart_projection_adder_max_num_factors");
  params.rotation_only_fallback = lc::LoadBool(config, "smart_projection_adder_rotation_only_fallback");
  params.splitting = lc::LoadBool(config, "smart_projection_adder_splitting");
  params.body_T_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam")));
  params.cam_noise =
    gtsam::noiseModel::Isotropic::Sigma(2, lc::LoadDouble(config, "optical_flow_nav_cam_noise_stddev"));
}

void LoadFeatureTrackerParams(config_reader::ConfigReader& config, FeatureTrackerParams& params) {
  params.sliding_window_duration = lc::LoadDouble(config, "feature_tracker_sliding_window_duration");
}

void LoadGraphValuesParams(config_reader::ConfigReader& config, GraphValuesParams& params) {
  params.ideal_duration = lc::LoadDouble(config, "ideal_duration");
  params.min_num_states = lc::LoadInt(config, "min_num_states");
  params.max_num_states = lc::LoadInt(config, "max_num_states");
  params.min_num_factors_per_feature = lc::LoadInt(config, "min_num_factors_per_feature");
}

void LoadNoiseParams(config_reader::ConfigReader& config, NoiseParams& params) {
  params.optical_flow_nav_cam_noise =
    gtsam::noiseModel::Isotropic::Sigma(2, lc::LoadDouble(config, "optical_flow_nav_cam_noise_stddev"));
  params.optical_flow_prior_velocity_stddev = lc::LoadDouble(config, "optical_flow_prior_velocity_stddev");
  params.starting_prior_translation_stddev = lc::LoadDouble(config, "starting_prior_translation_stddev");
  params.starting_prior_quaternion_stddev = lc::LoadDouble(config, "starting_prior_quaternion_stddev");
  params.starting_prior_velocity_stddev = lc::LoadDouble(config, "starting_prior_velocity_stddev");
  params.starting_prior_accel_bias_stddev = lc::LoadDouble(config, "starting_prior_accel_bias_stddev");
  params.starting_prior_gyro_bias_stddev = lc::LoadDouble(config, "starting_prior_gyro_bias_stddev");
}

void LoadSanityCheckerParams(config_reader::ConfigReader& config, SanityCheckerParams& params) {
  params.num_consecutive_pose_difference_failures_until_insane =
    lc::LoadInt(config, "num_consecutive_pose_difference_failures_until_insane");
  params.max_sane_position_difference = lc::LoadDouble(config, "max_sane_position_difference");
  params.check_pose_difference = lc::LoadBool(config, "check_pose_difference");
  params.check_position_covariance = lc::LoadBool(config, "check_position_covariance");
  params.check_orientation_covariance = lc::LoadBool(config, "check_orientation_covariance");
  params.position_covariance_threshold = lc::LoadDouble(config, "position_covariance_threshold");
  params.orientation_covariance_threshold = lc::LoadDouble(config, "orientation_covariance_threshold");
}

void LoadGraphLocalizerParams(config_reader::ConfigReader& config, GraphLocalizerParams& params) {
  LoadCalibrationParams(config, params.calibration);
  ii::LoadImuIntegratorParams(config, params.graph_initialization);
  LoadFactorParams(config, params.factor);
  LoadFeatureTrackerParams(config, params.feature_tracker);
  LoadGraphValuesParams(config, params.graph_values);
  LoadNoiseParams(config, params.noise);
  params.verbose = lc::LoadBool(config, "verbose");
  params.fatal_failures = lc::LoadBool(config, "fatal_failures");
  params.print_factor_info = lc::LoadBool(config, "print_factor_info");
  params.use_ceres_params = lc::LoadBool(config, "use_ceres_params");
  params.max_iterations = lc::LoadInt(config, "max_iterations");
  params.marginals_factorization = lc::LoadString(config, "marginals_factorization");
  params.limit_imu_factor_spacing = lc::LoadBool(config, "limit_imu_factor_spacing");
  params.max_imu_factor_spacing = lc::LoadDouble(config, "max_imu_factor_spacing");
  params.add_priors = lc::LoadBool(config, "add_priors");
  params.add_marginal_factors = lc::LoadBool(config, "add_marginal_factors");
  params.max_standstill_feature_track_avg_distance_from_mean =
    lc::LoadDouble(config, "max_standstill_feature_track_avg_distance_from_mean");
}
}  // namespace graph_localizer
