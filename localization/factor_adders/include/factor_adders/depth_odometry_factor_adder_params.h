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

#ifndef FACTOR_ADDERS_DEPTH_ODOMETRY_FACTOR_ADDER_PARAMS_H_
#define FACTOR_ADDERS_DEPTH_ODOMETRY_FACTOR_ADDER_PARAMS_H_

#include <graph_optimizer/factor_adder_params.h>

#include <gtsam/geometry/Pose3.h>

namespace factor_adders {
struct DepthOdometryFactorAdderParams : public graph_optimizer::FactorAdderParams {
  double noise_scale;
  bool use_points_between_factor;
  double position_covariance_threshold;
  double orientation_covariance_threshold;
  double point_to_point_error_threshold;
  double pose_translation_norm_threshold;
  int max_num_points_between_factors;
  gtsam::Pose3 body_T_sensor;
};
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_DEPTH_ODOMETRY_FACTOR_ADDER_PARAMS_H_
