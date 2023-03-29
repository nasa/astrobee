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

#include <parameter_reader/factor_adders.h>
#include <msg_conversions/msg_conversions.h>

namespace parameter_reader {
namespace mc = msg_conversions;
namespace fa = factor_adders;

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
  params.body_T_cam = mc::LoadEigenTransform(config, "body_T_cam", prefix);
  // TODO(rsoussan): add prefix option to these!
  // TODO(rsoussan): make not necessarily nav cam specific?
  params.cam_intrinsics.reset(new gtsam::Cal3_S2(lc::LoadCameraIntrinsics(config, "nav_cam", prefix)));
  params.cam_noise = gtsam::noiseModel::Isotropic::Sigma(2, mc::LoadDouble(config, "nav_cam_noise_stddev", prefix));
}

void LoadStandstillFactorAdderParams(config_reader::ConfigReader& config, fa::StandstillFactorAdderParams& params,
                                     const std::string& prefix) {}

void LoadVoSmartProjectionFactorAdderParams(config_reader::ConfigReader& config,
                                            fa::VoSmartProjectionFactorAdderParams& params, const std::string& prefix) {
}
}  // namespace parameter_reader
}  // namespace parameter_reader
