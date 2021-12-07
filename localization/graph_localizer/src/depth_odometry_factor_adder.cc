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

#include <graph_localizer/depth_odometry_factor_adder.h>
#include <graph_localizer/loc_pose_factor.h>
#include <graph_localizer/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
DepthOdometryFactorAdder::DepthOdometryFactorAdder(const DepthOdometryFactorAdderParams& params)
    : DepthOdometryFactorAdder::Base(params) {}

std::vector<go::FactorsToAdd> DepthOdometryFactorAdder::AddFactors(
  const lm::DepthOdometryMeasurement& depth_odometry_measurement) {
  std::vector<go::FactorsToAdd> factors_to_add_vector;
  go::FactorsToAdd relative_pose_factors_to_add;
  relative_pose_factors_to_add.reserve(1);
  const auto relative_pose_noise =
    Robust(gtsam::noiseModel::Gaussian::Covariance(
             depth_odometry_measurement.odometry.body_F_source_T_target.covariance * params().noise_scale, false),
           params().huber_k);
  const go::KeyInfo source_key_info(&sym::P, go::NodeUpdaterType::CombinedNavState,
                                    depth_odometry_measurement.odometry.source_time);
  const go::KeyInfo target_key_info(&sym::P, go::NodeUpdaterType::CombinedNavState,
                                    depth_odometry_measurement.odometry.target_time);
  gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr relative_pose_factor(new gtsam::BetweenFactor<gtsam::Pose3>(
    source_key_info.UninitializedKey(), target_key_info.UninitializedKey(),
    lc::GtPose(depth_odometry_measurement.odometry.body_F_source_T_target.pose), relative_pose_noise));
  relative_pose_factors_to_add.push_back({{source_key_info, target_key_info}, relative_pose_factor});
  relative_pose_factors_to_add.SetTimestamp(depth_odometry_measurement.odometry.target_time);
  LogDebug("AddFactors: Added 1 relative pose factor.");
  factors_to_add_vector.emplace_back(relative_pose_factors_to_add);
  return factors_to_add_vector;
}
}  // namespace graph_localizer
