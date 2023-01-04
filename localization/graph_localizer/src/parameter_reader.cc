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
#include <graph_optimizer/parameter_reader.h>
#include <imu_integration/utilities.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lc = localization_common;
namespace mc = msg_conversions;

void LoadCalibrationParams(config_reader::ConfigReader& config, CalibrationParams& params) {
  params.body_T_dock_cam = lc::LoadTransform(config, "dock_cam_transform");
  params.body_T_nav_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.body_T_perch_cam = lc::LoadTransform(config, "perch_cam_transform");
  params.world_T_dock = lc::LoadTransform(config, "world_dock_transform");
  params.nav_cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam")));
  params.dock_cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "dock_cam")));
}

void LoadFactorParams(config_reader::ConfigReader& config, FactorParams& params) {
  LoadHandrailFactorAdderParams(config, params.handrail_adder);
  LoadDepthOdometryFactorAdderParams(config, params.depth_odometry_adder);
  LoadLocFactorAdderParams(config, params.loc_adder);
  LoadARTagLocFactorAdderParams(config, params.ar_tag_loc_adder);
  LoadRotationFactorAdderParams(config, params.rotation_adder);
}

void LoadHandrailFactorAdderParams(config_reader::ConfigReader& config, HandrailFactorAdderParams& params) {
  params.enabled = mc::LoadBool(config, "handrail_adder_enabled");
  params.huber_k = mc::LoadDouble(config, "huber_k");
  params.min_num_line_matches = mc::LoadDouble(config, "handrail_adder_min_num_line_matches");
  params.min_num_plane_matches = mc::LoadDouble(config, "handrail_adder_min_num_plane_matches");
  params.point_to_line_stddev = mc::LoadDouble(config, "handrail_adder_point_to_line_stddev");
  params.point_to_plane_stddev = mc::LoadDouble(config, "handrail_adder_point_to_plane_stddev");
  params.body_T_perch_cam = lc::LoadTransform(config, "perch_cam_transform");
  params.use_silu_for_point_to_line_segment_factor =
    mc::LoadBool(config, "handrail_adder_use_silu_for_point_to_line_segment_factor");
}

void LoadARTagLocFactorAdderParams(config_reader::ConfigReader& config, LocFactorAdderParams& params) {
  params.add_pose_priors = mc::LoadBool(config, "ar_tag_loc_adder_add_pose_priors");
  params.add_projections = mc::LoadBool(config, "ar_tag_loc_adder_add_projections");
  params.enabled = params.add_pose_priors || params.add_projections ? true : false;
  params.huber_k = mc::LoadDouble(config, "huber_k");
  params.min_num_matches = mc::LoadInt(config, "ar_tag_loc_adder_min_num_matches");
  params.max_num_factors = mc::LoadInt(config, "ar_tag_loc_adder_max_num_factors");
  params.prior_translation_stddev = mc::LoadDouble(config, "ar_tag_loc_adder_prior_translation_stddev");
  params.prior_quaternion_stddev = mc::LoadDouble(config, "ar_tag_loc_adder_prior_quaternion_stddev");
  params.scale_pose_noise_with_num_landmarks =
    mc::LoadBool(config, "ar_tag_loc_adder_scale_pose_noise_with_num_landmarks");
  params.scale_projection_noise_with_num_landmarks =
    mc::LoadBool(config, "ar_tag_loc_adder_scale_projection_noise_with_num_landmarks");
  params.pose_noise_scale = mc::LoadDouble(config, "ar_tag_loc_adder_pose_noise_scale");
  params.projection_noise_scale = mc::LoadDouble(config, "ar_tag_loc_adder_projection_noise_scale");
  params.max_inlier_weighted_projection_norm =
    mc::LoadDouble(config, "ar_tag_loc_adder_max_inlier_weighted_projection_norm");
  params.weight_projections_with_distance = mc::LoadBool(config, "ar_tag_loc_adder_weight_projections_with_distance");
  params.add_prior_if_projections_fail = mc::LoadBool(config, "ar_tag_loc_adder_add_prior_if_projections_fail");
  params.body_T_cam = lc::LoadTransform(config, "dock_cam_transform");
  params.cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "dock_cam")));
  params.cam_noise = gtsam::noiseModel::Isotropic::Sigma(2, mc::LoadDouble(config, "loc_dock_cam_noise_stddev"));
}

void LoadDepthOdometryFactorAdderParams(config_reader::ConfigReader& config, DepthOdometryFactorAdderParams& params) {
  params.enabled = mc::LoadBool(config, "depth_odometry_adder_enabled");
  params.huber_k = mc::LoadDouble(config, "huber_k");
  params.noise_scale = mc::LoadDouble(config, "depth_odometry_adder_noise_scale");
  params.use_points_between_factor = mc::LoadBool(config, "depth_odometry_adder_use_points_between_factor");
  params.position_covariance_threshold = mc::LoadDouble(config, "depth_odometry_adder_position_covariance_threshold");
  params.orientation_covariance_threshold =
    mc::LoadDouble(config, "depth_odometry_adder_orientation_covariance_threshold");
  params.body_T_sensor = lc::LoadTransform(config, "haz_cam_transform");
  params.point_to_point_error_threshold = mc::LoadDouble(config, "depth_odometry_adder_point_to_point_error_threshold");
  params.pose_translation_norm_threshold =
    mc::LoadDouble(config, "depth_odometry_adder_pose_translation_norm_threshold");
  params.max_num_points_between_factors = mc::LoadDouble(config, "depth_odometry_adder_max_num_points_between_factors");
}

void LoadLocFactorAdderParams(config_reader::ConfigReader& config, LocFactorAdderParams& params) {
  params.add_pose_priors = mc::LoadBool(config, "loc_adder_add_pose_priors");
  params.add_projections = mc::LoadBool(config, "loc_adder_add_projections");
  params.enabled = params.add_pose_priors || params.add_projections ? true : false;
  params.huber_k = mc::LoadDouble(config, "huber_k");
  params.min_num_matches = mc::LoadInt(config, "loc_adder_min_num_matches");
  params.max_num_factors = mc::LoadInt(config, "loc_adder_max_num_factors");
  params.prior_translation_stddev = mc::LoadDouble(config, "loc_adder_prior_translation_stddev");
  params.prior_quaternion_stddev = mc::LoadDouble(config, "loc_adder_prior_quaternion_stddev");
  params.scale_pose_noise_with_num_landmarks = mc::LoadBool(config, "loc_adder_scale_pose_noise_with_num_landmarks");
  params.scale_projection_noise_with_num_landmarks =
    mc::LoadBool(config, "loc_adder_scale_projection_noise_with_num_landmarks");
  params.pose_noise_scale = mc::LoadDouble(config, "loc_adder_pose_noise_scale");
  params.projection_noise_scale = mc::LoadDouble(config, "loc_adder_projection_noise_scale");
  params.max_inlier_weighted_projection_norm = mc::LoadDouble(config, "loc_adder_max_inlier_weighted_projection_norm");
  params.weight_projections_with_distance = mc::LoadBool(config, "loc_adder_weight_projections_with_distance");
  params.add_prior_if_projections_fail = mc::LoadBool(config, "loc_adder_add_prior_if_projections_fail");
  params.body_T_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam")));
  params.cam_noise = gtsam::noiseModel::Isotropic::Sigma(2, mc::LoadDouble(config, "loc_nav_cam_noise_stddev"));
}

void LoadRotationFactorAdderParams(config_reader::ConfigReader& config, RotationFactorAdderParams& params) {
  params.enabled = mc::LoadBool(config, "rotation_adder_enabled");
  params.huber_k = mc::LoadDouble(config, "huber_k");
  params.min_avg_disparity = mc::LoadDouble(config, "rotation_adder_min_avg_disparity");
  params.rotation_stddev = mc::LoadDouble(config, "rotation_adder_rotation_stddev");
  params.max_percent_outliers = mc::LoadDouble(config, "rotation_adder_max_percent_outliers");
  params.body_T_nav_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.nav_cam_intrinsics = lc::LoadCameraIntrinsics(config, "nav_cam");
}

void LoadSanityCheckerParams(config_reader::ConfigReader& config, SanityCheckerParams& params) {
  params.num_consecutive_pose_difference_failures_until_insane =
    mc::LoadInt(config, "num_consecutive_pose_difference_failures_until_insane");
  params.max_sane_position_difference = mc::LoadDouble(config, "max_sane_position_difference");
  params.check_pose_difference = mc::LoadBool(config, "check_pose_difference");
  params.check_position_covariance = mc::LoadBool(config, "check_position_covariance");
  params.check_orientation_covariance = mc::LoadBool(config, "check_orientation_covariance");
  params.position_covariance_threshold = mc::LoadDouble(config, "position_covariance_threshold");
  params.orientation_covariance_threshold = mc::LoadDouble(config, "orientation_covariance_threshold");
}

void LoadGraphInitializerParams(config_reader::ConfigReader& config, GraphInitializerParams& params) {
}

void LoadCombinedNavStateNodeUpdaterParams(config_reader::ConfigReader& config,
                                           CombinedNavStateNodeUpdaterParams& params) {
  params.starting_prior_translation_stddev = mc::LoadDouble(config, "starting_prior_translation_stddev");
  params.starting_prior_quaternion_stddev = mc::LoadDouble(config, "starting_prior_quaternion_stddev");
  params.huber_k = mc::LoadDouble(config, "huber_k");
  params.add_priors = mc::LoadBool(config, "add_priors");
  LoadCombinedNavStateGraphValuesParams(config, params.graph_values);
}

void LoadCombinedNavStateGraphValuesParams(config_reader::ConfigReader& config,
                                           CombinedNavStateGraphValuesParams& params) {
  params.ideal_duration = mc::LoadDouble(config, "ideal_duration");
  params.min_num_states = mc::LoadInt(config, "min_num_states");
  params.max_num_states = mc::LoadInt(config, "max_num_states");
}

void LoadHandrailParams(config_reader::ConfigReader& config, HandrailParams& params) {
  params.length = mc::LoadDouble(config, "handrail_length");
  params.distance_to_wall = mc::LoadDouble(config, "handrail_wall_min_gap");
}

void LoadGraphLocalizerParams(config_reader::ConfigReader& config, GraphLocalizerParams& params) {
  LoadCalibrationParams(config, params.calibration);
  LoadCombinedNavStateNodeUpdaterParams(config, params.combined_nav_state_node_updater);
  LoadGraphInitializerParams(config, params.graph_initializer);
  LoadFactorParams(config, params.factor);
  LoadHandrailParams(config, params.handrail);
  go::LoadGraphOptimizerParams(config, params.graph_optimizer);
  params.huber_k = mc::LoadDouble(config, "huber_k");
  params.estimate_world_T_dock_using_loc = mc::LoadBool(config, "estimate_world_T_dock_using_loc");
}

void LoadGraphLocalizerNodeletParams(config_reader::ConfigReader& config, GraphLocalizerNodeletParams& params) {
  params.loc_adder_min_num_matches = mc::LoadInt(config, "loc_adder_min_num_matches");
  params.ar_tag_loc_adder_min_num_matches = mc::LoadInt(config, "ar_tag_loc_adder_min_num_matches");
  params.max_vl_buffer_size = mc::LoadInt(config, "max_vl_buffer_size");
  params.max_ar_buffer_size = mc::LoadInt(config, "max_ar_buffer_size");
  params.max_depth_odometry_buffer_size = mc::LoadInt(config, "max_depth_odometry_buffer_size");
  params.max_dl_buffer_size = mc::LoadInt(config, "max_dl_buffer_size");
}
}  // namespace graph_localizer
