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
#ifndef DEPTH_ODOMETRY_POINT_CLOUD_WITH_KNOWN_CORRESPONDENCES_ALIGNER_PARAMS_H_
#define DEPTH_ODOMETRY_POINT_CLOUD_WITH_KNOWN_CORRESPONDENCES_ALIGNER_PARAMS_H_

namespace depth_odometry {
struct PointCloudWithKnownCorrespondencesAlignerParams {
  int max_num_iterations;
  double function_tolerance;
  int max_num_matches;
  double normal_search_radius;
  bool use_umeyama_initial_guess;
  // TODO(rsoussan): these are exclusive, enforce this some how
  bool use_single_iteration_umeyama;
  bool use_point_to_plane_cost;
  bool use_symmetric_point_to_plane_cost;
  bool verbose_optimization;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_POINT_CLOUD_WITH_KNOWN_CORRESPONDENCES_ALIGNER_PARAMS_H_
