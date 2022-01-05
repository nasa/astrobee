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

#include <point_cloud_common/parameter_reader.h>
#include <localization_common/logger.h>
#include <msg_conversions/msg_conversions.h>

namespace point_cloud_common {
namespace mc = msg_conversions;

void LoadPointToPlaneICPParams(config_reader::ConfigReader& config, PointToPlaneICPParams& params) {
  params.fitness_threshold = mc::LoadDouble(config, "fitness_threshold");
  params.search_radius = mc::LoadDouble(config, "search_radius");
  params.max_iterations = mc::LoadInt(config, "max_iterations");
  params.symmetric_objective = mc::LoadBool(config, "symmetric_objective");
  params.enforce_same_direction_normals = mc::LoadBool(config, "enforce_same_direction_normals");
  params.correspondence_rejector_surface_normal = mc::LoadBool(config, "correspondence_rejector_surface_normal");
  params.correspondence_rejector_surface_normal_threshold =
    mc::LoadDouble(config, "correspondence_rejector_surface_normal_threshold");
  params.correspondence_rejector_median_distance = mc::LoadBool(config, "correspondence_rejector_median_distance");
  params.correspondence_rejector_median_distance_factor =
    mc::LoadDouble(config, "correspondence_rejector_median_distance_factor");
  params.coarse_to_fine = mc::LoadBool(config, "coarse_to_fine");
  params.num_coarse_to_fine_levels = mc::LoadInt(config, "num_coarse_to_fine_levels");
  params.coarse_to_fine_final_leaf_size = mc::LoadDouble(config, "coarse_to_fine_final_leaf_size");
  params.downsample_last_coarse_to_fine_iteration = mc::LoadBool(config, "downsample_last_coarse_to_fine_iteration");
}

void LoadPointCloudWithKnownCorrespondencesAlignerParams(config_reader::ConfigReader& config,
                                                         PointCloudWithKnownCorrespondencesAlignerParams& params) {
  params.max_num_iterations = mc::LoadInt(config, "pcwkca_max_num_iterations");
  params.function_tolerance = mc::LoadDouble(config, "pcwkca_function_tolerance");
  params.max_num_matches = mc::LoadInt(config, "pcwkca_max_num_match_sets");
  params.normal_search_radius = mc::LoadDouble(config, "pcwkca_normal_search_radius");
  params.use_umeyama_initial_guess = mc::LoadBool(config, "pcwkca_use_umeyama_initial_guess");
  params.use_single_iteration_umeyama = mc::LoadBool(config, "pcwkca_use_single_iteration_umeyama");
  params.use_point_to_plane_cost = mc::LoadBool(config, "pcwkca_use_point_to_plane_cost");
  params.use_symmetric_point_to_plane_cost = mc::LoadBool(config, "pcwkca_use_symmetric_point_to_plane_cost");
  if (!mc::SingleBoolTrue({params.use_single_iteration_umeyama, params.use_point_to_plane_cost,
                           params.use_symmetric_point_to_plane_cost}) &&
      (params.use_single_iteration_umeyama || params.use_point_to_plane_cost ||
       params.use_symmetric_point_to_plane_cost))
    LogFatal(
      "LoadPointCloudWithKnownCorrespondencesAlignerParams: Multiple solver methods enabled (only one of single "
      "iteration umeyama, point to plane cost, and symmetric point to plane cost can be enabled).");
  params.verbose_optimization = mc::LoadBool(config, "pcwkca_verbose_optimization");
}
}  // namespace point_cloud_common
