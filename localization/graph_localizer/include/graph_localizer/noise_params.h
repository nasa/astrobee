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
#ifndef GRAPH_LOCALIZER_NOISE_PARAMS_H_
#define GRAPH_LOCALIZER_NOISE_PARAMS_H_

#include <gtsam/linear/NoiseModel.h>

namespace graph_localizer {
struct NoiseParams {
  gtsam::SharedIsotropic dock_cam_noise;
  gtsam::SharedIsotropic nav_cam_noise;
  double optical_flow_prior_translation_stddev;
  double optical_flow_prior_quaternion_stddev;
  double optical_flow_prior_velocity_stddev;
  double starting_prior_translation_stddev;
  double starting_prior_quaternion_stddev;
  double starting_prior_velocity_stddev;
  double starting_prior_accel_bias_stddev;
  double starting_prior_gyro_bias_stddev;
  // TODO(rsoussan): put this somewhere else?
  double prior_min_variance_val;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_NOISE_PARAMS_H_
