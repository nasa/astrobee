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
#ifndef GRAPH_VIO_SANITY_CHECKER_PARAMS_H_
#define GRAPH_VIO_SANITY_CHECKER_PARAMS_H_

namespace graph_vio {
struct SanityCheckerParams {
  // Check Pose
  bool check_pose_difference;
  int num_consecutive_pose_difference_failures_until_insane;
  double max_sane_position_difference;
  // Check Covariances
  bool check_position_covariance;
  bool check_orientation_covariance;
  double position_covariance_threshold;
  double orientation_covariance_threshold;
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_SANITY_CHECKER_PARAMS_H_
