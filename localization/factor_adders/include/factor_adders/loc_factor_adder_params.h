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

#ifndef FACTOR_ADDERS_LOC_FACTOR_ADDER_PARAMS_H_
#define FACTOR_ADDERS_LOC_FACTOR_ADDER_PARAMS_H_

#include <factor_adders/factor_adder_params.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

namespace factor_adders {
struct LocFactorAdderParams : public FactorAdderParams {
  // Add pose prior factors using estimated pose from matched
  // projections measurement.
  bool add_pose_priors;
  // Add projections factor using matched projections.
  bool add_projection_factors;
  // Add pose prior factors if all projection factors fail.
  bool add_prior_if_projection_factors_fail;
  // Translation stddev noise for prior factors.
  double prior_translation_stddev;
  // Quaternion stddev noise for prior factors.
  double prior_quaternion_stddev;
  // Inversely scale noise with the number of matched projections when creating a pose factor.
  bool scale_pose_noise_with_num_landmarks;
  // Scale projection noise with the square of the ratio of the average number of matches per measurement
  // to the current number of matches when creating a projections factor.
  // Yields lower noise for measurements with more than average matches.
  bool scale_projection_noise_with_num_landmarks;
  // Scale projection factor noise using the inverse of the distance to a landmark.
  bool scale_projection_noise_with_landmark_distance;
  // Relative pose noise scale.
  double pose_noise_scale;
  // Relative projections noise scale.
  double projection_noise_scale;
  // Max num projection factors to add for a matched projections measurement.
  int max_num_projection_factors;
  // Min number of matched projections to create a factor (prior or projection).
  int min_num_matches_per_measurement;
  // Max projection error norm for a projection factor to be valid.
  double max_valid_projection_error;
  // Camera extrinsics.
  gtsam::Pose3 body_T_cam;
  // Camera intrinsics.
  boost::shared_ptr<gtsam::Cal3_S2> cam_intrinsics;
  // Camera noise used for projection factor noise.
  gtsam::SharedIsotropic cam_noise;
};
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_LOC_FACTOR_ADDER_PARAMS_H_
