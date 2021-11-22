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
#ifndef POINT_CLOUD_COMMON_POINT_TO_PLANE_ICP_PARAMS_H_
#define POINT_CLOUD_COMMON_POINT_TO_PLANE_ICP_PARAMS_H_

namespace point_cloud_common {
struct PointToPlaneICPParams {
  double search_radius;
  double fitness_threshold;
  int max_iterations;
  bool symmetric_objective;
  bool enforce_same_direction_normals;
  bool correspondence_rejector_surface_normal;
  double correspondence_rejector_surface_normal_threshold;
  bool coarse_to_fine;
  int num_coarse_to_fine_levels;
  double coarse_to_fine_final_leaf_size;
  bool downsample_last_coarse_to_fine_iteration;
};
}  // namespace point_cloud_common

#endif  // POINT_CLOUD_COMMON_POINT_TO_PLANE_ICP_PARAMS_H_
