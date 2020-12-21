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

#ifndef GRAPH_LOCALIZER_LOC_FACTOR_ADDER_PARAMS_H_
#define GRAPH_LOCALIZER_LOC_FACTOR_ADDER_PARAMS_H_

#include <graph_localizer/factor_adder_params.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/linear/NoiseModel.h>

namespace graph_localizer {
struct LocFactorAdderParams : public FactorAdderParams {
  bool add_pose_priors;
  bool add_projections;
  double prior_translation_stddev;
  double prior_quaternion_stddev;
  int min_num_matches;
  bool scale_pose_noise_with_num_landmarks;
  bool scale_projection_noise_with_num_landmarks;
  double pose_noise_scale;
  double projection_noise_scale;
  double max_inlier_weighted_projection_norm;
  bool weight_projections_with_mahal_distance;
  gtsam::Pose3 body_T_cam;
  boost::shared_ptr<gtsam::Cal3_S2> cam_intrinsics;
  gtsam::SharedIsotropic cam_noise;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_LOC_FACTOR_ADDER_PARAMS_H_
