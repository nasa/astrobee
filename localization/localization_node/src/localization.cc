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

#include <localization_node/localization.h>

#include <sparse_mapping/sparse_map.h>
#include <ff_msgs/VisualLandmarks.h>
#include <msg_conversions/msg_conversions.h>
#include <ros/ros.h>

#include <camera/camera_params.h>

namespace localization_node {

Localizer::Localizer(sparse_mapping::SparseMap* map): map_(map) {}

void Localizer::ReadParams(config_reader::ConfigReader& config) {
  camera::CameraParameters cam_params(&config, "nav_cam");
  std::string prefix;
  const auto detector_name = map_->GetDetectorName();
  if (detector_name == "ORGBRISK") {
    prefix = "brisk_";
  } else if (detector_name == "TEBLID512") {
    prefix = "teblid512_";
  } else if (detector_name == "TEBLID256") {
    prefix = "teblid256_";
  } else {
    ROS_FATAL_STREAM("Invalid detector: " << detector_name);
  }

  // Loc params
  sparse_mapping::LocalizationParameters loc_params;
  LOAD_PARAM(loc_params.num_similar, config, prefix);
  LOAD_PARAM(loc_params.min_query_score_ratio, config, prefix);
  LOAD_PARAM(loc_params.ransac_inlier_tolerance, config, prefix);
  LOAD_PARAM(loc_params.num_ransac_iterations, config, prefix);
  LOAD_PARAM(loc_params.early_break_landmarks, config, prefix);
  LOAD_PARAM(loc_params.histogram_equalization, config, prefix);
  LOAD_PARAM(loc_params.check_essential_matrix, config, prefix);
  LOAD_PARAM(loc_params.essential_ransac_iterations, config, prefix);
  LOAD_PARAM(loc_params.add_similar_images, config, prefix);
  LOAD_PARAM(loc_params.add_best_previous_image, config, prefix);
  LOAD_PARAM(loc_params.hamming_distance, config, prefix);
  LOAD_PARAM(loc_params.goodness_ratio, config, prefix);
  LOAD_PARAM(loc_params.use_clahe, config, prefix);
  LOAD_PARAM(loc_params.num_extra_localization_db_images, config, prefix);
  LOAD_PARAM(loc_params.verbose_localization, config, "");
  LOAD_PARAM(loc_params.visualize_localization_matches, config, "");

  // Detector Params
  double min_threshold, default_threshold, max_threshold, goodness_ratio, too_many_ratio, too_few_ratio;
  int min_features, max_features, detection_retries;
  LOAD_PARAM(min_threshold, config, prefix);
  LOAD_PARAM(default_threshold, config, prefix);
  LOAD_PARAM(max_threshold, config, prefix);
  LOAD_PARAM(detection_retries, config, prefix);
  LOAD_PARAM(min_features, config, prefix);
  LOAD_PARAM(max_features, config, prefix);
  LOAD_PARAM(too_many_ratio, config, prefix);
  LOAD_PARAM(too_few_ratio, config, prefix);

  // Localizer threshold params
  LOAD_PARAM(params_.success_history_size, config, prefix);
  LOAD_PARAM(params_.min_success_rate, config, prefix);
  LOAD_PARAM(params_.max_success_rate, config, prefix);
  LOAD_PARAM(params_.min_features, config, prefix);
  LOAD_PARAM(params_.max_features, config, prefix);
  LOAD_PARAM(params_.adjust_num_similar, config, prefix);
  LOAD_PARAM(params_.min_num_similar, config, prefix);
  LOAD_PARAM(params_.max_num_similar, config, prefix);

  // This check must happen before the histogram_equalization flag is set into the map
  // to compare with what is there already.
  sparse_mapping::HistogramEqualizationCheck(map_->GetHistogramEqualization(),
                                             loc_params.histogram_equalization);
  // Check consistency between clahe params
  if (loc_params.use_clahe && (loc_params.histogram_equalization != 3 || map_->GetHistogramEqualization() != 3)) {
    ROS_FATAL("Invalid clahe and histogram equalization settings.");
  }

  map_->SetCameraParameters(cam_params);
  map_->SetLocParams(loc_params);
  map_->SetDetectorParams(min_features, max_features, detection_retries,
                          min_threshold, default_threshold, max_threshold, too_many_ratio, too_few_ratio);
}

bool Localizer::Localize(cv_bridge::CvImageConstPtr image_ptr, ff_msgs::VisualLandmarks* vl,
     Eigen::Matrix2Xd* image_keypoints) {
  bool multithreaded = false;
  cv::Mat image_descriptors;

  Eigen::Matrix2Xd keypoints;
  if (image_keypoints == NULL) {
    image_keypoints = &keypoints;
  }

  vl->header = std_msgs::Header();
  vl->header.stamp = image_ptr->header.stamp;
  vl->header.frame_id = "world";

  timer_.Start();
  map_->DetectFeatures(image_ptr->image, multithreaded, &image_descriptors, image_keypoints);
  camera::CameraModel camera(Eigen::Vector3d(),
                             Eigen::Matrix3d::Identity(),
                             map_->GetCameraParameters());
  std::vector<Eigen::Vector3d> landmarks;
  std::vector<Eigen::Vector2d> observations;
  if (!map_->Localize(image_descriptors, *image_keypoints,
                               &camera, &landmarks, &observations, nullptr, image_ptr->image)) {
    successes_.emplace_back(0);
    AdjustThresholds();
    // LOG(INFO) << "Failed to localize image.";
    timer_.Stop();
    return false;
  }
  successes_.emplace_back(1);
  AdjustThresholds();
  timer_.Stop();

  Eigen::Affine3d global_pose = camera.GetTransform().inverse();
  Eigen::Quaterniond quat(global_pose.rotation());

  vl->runtime = timer_.last_value();
  vl->pose.position = msg_conversions::eigen_to_ros_point(global_pose.translation());
  vl->pose.orientation = msg_conversions::eigen_to_ros_quat(quat);
  assert(landmarks.size() == observations.size());
  vl->landmarks.reserve(landmarks.size());

  for (size_t i = 0; i < landmarks.size(); i++) {
    ff_msgs::VisualLandmark l;
    l.x = landmarks[i].x();
    l.y = landmarks[i].y();
    l.z = landmarks[i].z();
    l.u = observations[i].x();
    l.v = observations[i].y();
    vl->landmarks.push_back(l);
  }

  return true;
}

void Localizer::AdjustThresholds() {
  if (successes_.size() == 0 || params_.success_history_size == 0) return;
  if (successes_.size() < params_.success_history_size) return;
  while (successes_.size() > params_.success_history_size) {
    successes_.pop_front();
  }
  const double average =
    std::accumulate(successes_.cbegin(), successes_.cend(), 0) / static_cast<double>(successes_.size());
  const int last_keypoint_count = map_->detector().dynamic_detector().last_keypoint_count();
  if (average < params_.min_success_rate) {
    if (last_keypoint_count < params_.max_features) {
      map_->detector().dynamic_detector().TooFew();
    }
    if (params_.adjust_num_similar) {
      const int current_num_similar = map_->loc_params().num_similar;
      const int new_num_similar = std::min(current_num_similar + 1, params_.max_num_similar);
      map_->loc_params().num_similar = new_num_similar;
    }
  }
  if (average > params_.max_success_rate) {
    if (last_keypoint_count > params_.min_features) {
      map_->detector().dynamic_detector().TooMany();
    }
    if (params_.adjust_num_similar) {
      const int current_num_similar = map_->loc_params().num_similar;
      const int new_num_similar = std::max(current_num_similar - 1, params_.min_num_similar);
      map_->loc_params().num_similar = new_num_similar;
    }
  }
}
}  // namespace localization_node
