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
  } else if (detector_name == "TEBLID") {
    prefix = "teblid_";
  } else {
    ROS_FATAL_STREAM("Invalid detector: " << detector_name);
  }

  // Loc params
  sparse_mapping::LocalizationParameters loc_params;
  LOAD_PARAM(loc_params.num_similar, config, prefix);
  LOAD_PARAM(loc_params.ransac_inlier_tolerance, config, prefix);
  LOAD_PARAM(loc_params.num_ransac_iterations, config, prefix);
  LOAD_PARAM(loc_params.early_break_landmarks, config, prefix);
  LOAD_PARAM(loc_params.histogram_equalization, config, prefix);
  LOAD_PARAM(loc_params.check_essential_matrix, config, prefix);
  LOAD_PARAM(loc_params.add_similar_images, config, prefix);
  LOAD_PARAM(loc_params.add_best_previous_image, config, prefix);
  LOAD_PARAM(loc_params.hamming_distance, config, prefix);
  LOAD_PARAM(loc_params.goodness_ratio, config, prefix);
  LOAD_PARAM(loc_params.use_clahe, config, prefix);
  LOAD_PARAM(loc_params.num_extra_localization_db_images, config, prefix);
  LOAD_PARAM(loc_params.verbose_localization, config, "");
  LOAD_PARAM(loc_params.visualize_localization_matches, config, "");

  // Detector Params
  double min_threshold, default_threshold, max_threshold, goodness_ratio;
  int min_features, max_features, detection_retries;
  LOAD_PARAM(min_threshold, config, prefix);
  LOAD_PARAM(default_threshold, config, prefix);
  LOAD_PARAM(max_threshold, config, prefix);
  LOAD_PARAM(detection_retries, config, prefix);
  LOAD_PARAM(min_features, config, prefix);
  LOAD_PARAM(max_features, config, prefix);

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
                          min_threshold, default_threshold, max_threshold);
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

  map_->DetectFeatures(image_ptr->image, multithreaded, &image_descriptors, image_keypoints);

  camera::CameraModel camera(Eigen::Vector3d(),
                             Eigen::Matrix3d::Identity(),
                             map_->GetCameraParameters());
  std::vector<Eigen::Vector3d> landmarks;
  std::vector<Eigen::Vector2d> observations;
  if (!map_->Localize(image_descriptors, *image_keypoints,
                               &camera, &landmarks, &observations, nullptr, image_ptr->image)) {
    // LOG(INFO) << "Failed to localize image.";
    return false;
  }

  Eigen::Affine3d global_pose = camera.GetTransform().inverse();
  Eigen::Quaterniond quat(global_pose.rotation());

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

};  // namespace localization_node
