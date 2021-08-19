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

#include <depth_odometry/parameter_reader.h>
#include <msg_conversions/msg_conversions.h>

namespace depth_odometry {
namespace mc = msg_conversions;

void LoadDepthOdometryNodeletParams(config_reader::ConfigReader& config, DepthOdometryNodeletParams& params) {
  params.publish_point_clouds = mc::LoadBool(config, "publish_point_clouds");
}

void LoadDepthOdometryParams(config_reader::ConfigReader& config, DepthOdometryParams& params) {
  LoadDepthImageAlignerParams(config, params.depth_image_aligner);
  LoadICPParams(config, params.icp);
  params.max_time_diff = mc::LoadDouble(config, "max_time_diff");
  params.max_image_and_point_cloud_time_diff = mc::LoadDouble(config, "max_image_and_point_cloud_time_diff");
  params.depth_point_cloud_registration_enabled = mc::LoadBool(config, "depth_point_cloud_registration_enabled");
  params.depth_image_registration_enabled = mc::LoadBool(config, "depth_image_registration_enabled");
  params.position_covariance_threshold = mc::LoadDouble(config, "position_covariance_threshold");
  params.orientation_covariance_threshold = mc::LoadDouble(config, "orientation_covariance_threshold");
  params.inital_estimate_with_ransac_ia = mc::LoadBool(config, "inital_estimate_with_ransac_ia");
  params.frame_change_transform = mc::LoadBool(config, "frame_change_transform");
  params.body_T_haz_cam = msg_conversions::LoadEigenTransform(config, "haz_cam_transform");
}

void LoadDepthImageAlignerParams(config_reader::ConfigReader& config, DepthImageAlignerParams& params) {
  params.max_match_hamming_distance = mc::LoadInt(config, "max_match_hamming_distance");
  params.flann_table_number = mc::LoadInt(config, "flann_table_number");
  params.flann_key_size = mc::LoadInt(config, "flann_key_size");
  params.flann_multi_probe_level = mc::LoadInt(config, "flann_multi_probe_level");
  params.brisk_threshold = mc::LoadInt(config, "brisk_threshold");
  params.brisk_octaves = mc::LoadInt(config, "brisk_octaves");
  params.brisk_float_pattern_scale = mc::LoadFloat(config, "brisk_float_pattern_scale");
  params.use_clahe = mc::LoadBool(config, "use_clahe");
  params.clahe_grid_length = mc::LoadInt(config, "clahe_grid_length");
  params.clahe_clip_limit = mc::LoadDouble(config, "clahe_clip_limit");
}

void LoadICPParams(config_reader::ConfigReader& config, ICPParams& params) {
  params.fitness_threshold = mc::LoadDouble(config, "fitness_threshold");
  params.search_radius = mc::LoadDouble(config, "search_radius");
  params.max_iterations = mc::LoadInt(config, "max_iterations");
  params.symmetric_objective = mc::LoadBool(config, "symmetric_objective");
  params.enforce_same_direction_normals = mc::LoadBool(config, "enforce_same_direction_normals");
  params.correspondence_rejector_surface_normal = mc::LoadBool(config, "correspondence_rejector_surface_normal");
  params.correspondence_rejector_surface_normal_threshold =
    mc::LoadDouble(config, "correspondence_rejector_surface_normal_threshold");
  params.coarse_to_fine = mc::LoadBool(config, "coarse_to_fine");
  params.num_coarse_to_fine_levels = mc::LoadInt(config, "num_coarse_to_fine_levels");
  params.coarse_to_fine_final_leaf_size = mc::LoadDouble(config, "coarse_to_fine_final_leaf_size");
  params.downsample_last_coarse_to_fine_iteration = mc::LoadBool(config, "downsample_last_coarse_to_fine_iteration");
}
}  // namespace depth_odometry
