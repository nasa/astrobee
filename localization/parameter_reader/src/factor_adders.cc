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

#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>
#include <parameter_reader/factor_adders.h>
#include <parameter_reader/vision_common.h>

namespace parameter_reader {
namespace fa = factor_adders;
namespace lc = localization_common;
namespace mc = msg_conversions;

void LoadFactorAdderParams(config_reader::ConfigReader& config, fa::FactorAdderParams& params,
                           const std::string& prefix) {
  LOAD_PARAM(params.enabled, config, prefix);
  LOAD_PARAM(params.huber_k, config, prefix);
}

void LoadLocFactorAdderParams(config_reader::ConfigReader& config, fa::LocFactorAdderParams& params,
                              const std::string& prefix) {
  LoadFactorAdderParams(config, params, prefix);
  LOAD_PARAM(params.add_pose_priors, config, prefix);
  LOAD_PARAM(params.add_projection_factors, config, prefix);
  LOAD_PARAM(params.add_prior_if_projection_factors_fail, config, prefix);
  LOAD_PARAM(params.prior_translation_stddev, config, prefix);
  LOAD_PARAM(params.prior_quaternion_stddev, config, prefix);
  LOAD_PARAM(params.scale_pose_noise_with_num_landmarks, config, prefix);
  LOAD_PARAM(params.scale_projection_noise_with_num_landmarks, config, prefix);
  LOAD_PARAM(params.scale_projection_noise_with_landmark_distance, config, prefix);
  LOAD_PARAM(params.pose_noise_scale, config, prefix);
  LOAD_PARAM(params.projection_noise_scale, config, prefix);
  LOAD_PARAM(params.max_num_projection_factors, config, prefix);
  LOAD_PARAM(params.min_num_matches_per_measurement, config, prefix);
  LOAD_PARAM(params.max_valid_projection_error, config, prefix);
  params.body_T_cam = lc::LoadTransform(config, "nav_cam_transform", prefix);
  // TODO(rsoussan): make not necessarily nav cam specific?
  params.cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam", prefix)));
  params.cam_noise = gtsam::noiseModel::Isotropic::Sigma(2, mc::LoadDouble(config, "nav_cam_noise_stddev", prefix));
}

void LoadStandstillFactorAdderParams(config_reader::ConfigReader& config, fa::StandstillFactorAdderParams& params,
                                     const std::string& prefix) {
  LoadFactorAdderParams(config, params, prefix);
  LOAD_PARAM(params.add_velocity_prior, config, prefix);
  LOAD_PARAM(params.add_pose_between_factor, config, prefix);
  LOAD_PARAM(params.prior_velocity_stddev, config, prefix);
  LOAD_PARAM(params.pose_between_factor_translation_stddev, config, prefix);
  LOAD_PARAM(params.pose_between_factor_rotation_stddev, config, prefix);
}

void LoadVoSmartProjectionFactorAdderParams(config_reader::ConfigReader& config,
                                            fa::VoSmartProjectionFactorAdderParams& params, const std::string& prefix) {
  LoadSpacedFeatureTrackerParams(config, params.spaced_feature_tracker, prefix);
  LOAD_PARAM(params.max_num_factors, config, prefix);
  LOAD_PARAM(params.min_num_points_per_factor, config, prefix);
  LOAD_PARAM(params.max_num_points_per_factor, config, prefix);
  LOAD_PARAM(params.min_avg_distance_from_mean, config, prefix);
  LOAD_PARAM(params.robust, config, prefix);
  LOAD_PARAM(params.fix_invalid_factors, config, prefix);
  LOAD_PARAM(params.scale_noise_with_num_points, config, prefix);
  LOAD_PARAM(params.noise_scale, config, prefix);
  // TODO(rsoussan): make not necessarily nav cam specific?
  params.body_T_cam = lc::LoadTransform(config, "nav_cam_transform", prefix);
  params.cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam", prefix)));
  params.cam_noise = gtsam::noiseModel::Isotropic::Sigma(2, mc::LoadDouble(config, "nav_cam_noise_stddev", prefix));
  LoadSmartProjectionParams(config, params.smart_factor, prefix);
}

void LoadSmartProjectionParams(config_reader::ConfigReader& config, gtsam::SmartProjectionParams& params,
                               const std::string& prefix) {
  // Load params
  const bool rotation_only_fallback = mc::LoadBool(config, "smart_projection_rotation_only_fallback");
  const bool enable_EPI = mc::LoadBool(config, "smart_projection_enable_EPI");
  const double landmark_distance_threshold = mc::LoadDouble(config, "smart_projection_landmark_distance_threshold");
  const double dynamic_outlier_rejection_threshold =
    mc::LoadDouble(config, "smart_projection_dynamic_outlier_rejection_threshold");
  const double retriangulation_threshold = mc::LoadDouble(config, "smart_projection_retriangulation_threshold", prefix);
  const bool verbose_cheirality = mc::LoadBool(config, "smart_projection_verbose_cheirality", prefix);

  // Set Params
  params.verboseCheirality = verbose_cheirality;
  params.setRankTolerance(1e-9);
  params.setLandmarkDistanceThreshold(landmark_distance_threshold);
  params.setDynamicOutlierRejectionThreshold(dynamic_outlier_rejection_threshold);
  params.setRetriangulationThreshold(retriangulation_threshold);
  if (rotation_only_fallback) params.setDegeneracyMode(gtsam::DegeneracyMode::HANDLE_INFINITY);
  params.setEnableEPI(enable_EPI);
}
}  // namespace parameter_reader
