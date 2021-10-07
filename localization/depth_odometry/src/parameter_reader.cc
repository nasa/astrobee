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
#include <localization_common/logger.h>
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
  params.body_T_haz_cam = msg_conversions::LoadEigenTransform(config, "haz_cam_transform");
  const bool sim = mc::LoadBool(config, "sim");
  if (true) {  // sim) {
    params.haz_cam_A_haz_depth = Eigen::Affine3d::Identity();
  } else {
    Eigen::MatrixXd M(4, 4);
    config_reader::ConfigReader::Table mat(&config, "hazcam_depth_to_image_transform");
    int count = 0;
    for (int row = 0; row < M.rows(); row++) {
      for (int col = 0; col < M.cols(); col++) {
        count++;  // note that the count stats from 1
        if (!mat.GetReal(count, &M(row, col))) {
          LogFatal("Failed to get val for hazcam_depth_to_image_trafo!");
        }
      }
    }
    Eigen::Affine3d a;
    a.matrix() = M;
    /*Eigen::Matrix3d rot;
    Eigen::Matrix3d scale;
    a.computeScalingRotation(&scale, &rot);
    LogError("scale: " << scale.matrix());
    LogError("rot: " << rot.eulerAngles(2, 1, 0).matrix());
    params.haz_cam_A_haz_depth.matrix() = M;*/
    const double scale_factor = mc::LoadDouble(config, "scale_factor");
    // params.haz_cam_A_haz_depth = Eigen::Affine3d::Identity() * Eigen::Scaling(scale_factor);
    params.haz_cam_A_haz_depth.translation() = a.translation();
    params.haz_cam_A_haz_depth.linear() = a.linear();
    // params.haz_cam_A_haz_depth.linear()(0, 0) = 1.0;
    // params.haz_cam_A_haz_depth.linear()(1, 1) = 1.0;
    // params.haz_cam_A_haz_depth.linear()(2, 2) = 1.0;
  }
  LogError("hazcam_A_haz_depth: " << std::endl << params.haz_cam_A_haz_depth.matrix());
}

void LoadBriskFeatureDetectorAndMatcherParams(config_reader::ConfigReader& config,
                                              BriskFeatureDetectorAndMatcherParams& params) {
  params.brisk_threshold = mc::LoadInt(config, "brisk_threshold");
  params.brisk_octaves = mc::LoadInt(config, "brisk_octaves");
  params.brisk_float_pattern_scale = mc::LoadFloat(config, "brisk_float_pattern_scale");
  params.max_match_hamming_distance = mc::LoadInt(config, "brisk_max_match_hamming_distance");
  params.flann_table_number = mc::LoadInt(config, "brisk_flann_table_number");
  params.flann_key_size = mc::LoadInt(config, "brisk_flann_key_size");
  params.flann_multi_probe_level = mc::LoadInt(config, "brisk_flann_multi_probe_level");
}

void LoadLKOpticalFlowFeatureDetectorAndMatcherParams(config_reader::ConfigReader& config,
                                                      LKOpticalFlowFeatureDetectorAndMatcherParams& params) {
  params.max_iterations = mc::LoadInt(config, "lk_max_iterations");
  params.termination_epsilon = mc::LoadDouble(config, "lk_termination_epsilon");
  params.window_width = mc::LoadInt(config, "lk_window_width");
  params.window_height = mc::LoadInt(config, "lk_window_height");
  params.max_level = mc::LoadInt(config, "lk_max_level");
  params.min_eigen_threshold = mc::LoadDouble(config, "lk_min_eigen_threshold");
  params.max_flow_distance = mc::LoadDouble(config, "lk_max_flow_distance");
  params.max_backward_match_distance = mc::LoadDouble(config, "lk_max_backward_match_distance");
}

void LoadSurfFeatureDetectorAndMatcherParams(config_reader::ConfigReader& config,
                                             SurfFeatureDetectorAndMatcherParams& params) {
  params.surf_threshold = mc::LoadInt(config, "surf_threshold");
  params.max_match_distance = mc::LoadDouble(config, "surf_max_match_distance");
}

void LoadDepthImageAlignerParams(config_reader::ConfigReader& config, DepthImageAlignerParams& params) {
  LoadBriskFeatureDetectorAndMatcherParams(config, params.brisk_feature_detector_and_matcher);
  LoadLKOpticalFlowFeatureDetectorAndMatcherParams(config, params.lk_optical_flow_feature_detector_and_matcher);
  LoadSurfFeatureDetectorAndMatcherParams(config, params.surf_feature_detector_and_matcher);
  params.detector = mc::LoadString(config, "detector");
  params.use_clahe = mc::LoadBool(config, "use_clahe");
  params.clahe_grid_length = mc::LoadInt(config, "clahe_grid_length");
  params.clahe_clip_limit = mc::LoadDouble(config, "clahe_clip_limit");
  params.min_x_distance_to_border = mc::LoadDouble(config, "min_x_distance_to_border");
  params.min_y_distance_to_border = mc::LoadDouble(config, "min_y_distance_to_border");
  params.num_ransac_iterations = mc::LoadInt(config, "num_ransac_iterations");
  params.max_inlier_tolerance = mc::LoadInt(config, "max_inlier_tolerance");
  params.min_num_inliers = mc::LoadInt(config, "min_num_inliers");
  LoadPointCloudWithKnownCorrespondencesAlignerParams(config, params.point_cloud_with_known_correspondences_aligner);
  const bool sim = mc::LoadBool(config, "sim");
  if (sim) {
    const Eigen::Vector2i image_size(171, 224);
    const Eigen::Vector2d focal_length(186.40017522, 186.40017522);
    const Eigen::Vector2d optical_center(111.5, 85);
    params.camera_params.reset(new camera::CameraParameters(image_size, focal_length, optical_center));
    // params.camera_params->SetUndistortedSize(Eigen::Vector2i(600, 500));
    Eigen::VectorXd distortion(4);
    distortion[0] = 0;  //-0.050689743;
    distortion[1] = 0;  //-1.1461691;
    distortion[2] = 0;  //-0.001373226;
    distortion[3] = 0;  //-0.00056427513;
    params.camera_params->SetDistortion(distortion);
  } else {
    params.camera_params.reset(new camera::CameraParameters(&config, "haz_cam"));
  }
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

void LoadPointCloudWithKnownCorrespondencesAlignerParams(config_reader::ConfigReader& config,
                                                         PointCloudWithKnownCorrespondencesAlignerParams& params) {
  params.max_num_iterations = mc::LoadInt(config, "pcwkca_max_num_iterations");
  params.function_tolerance = mc::LoadDouble(config, "pcwkca_function_tolerance");
  params.max_num_matches = mc::LoadInt(config, "pcwkca_max_num_match_sets");
}
}  // namespace depth_odometry
