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

namespace localization_node {

Localizer::Localizer(sparse_mapping::SparseMap* comp_map_ptr) :
      map_(comp_map_ptr) {
}

Localizer::~Localizer(void) {
}

void Localizer::ReadParams(config_reader::ConfigReader* config) {
  int num_similar, ransac_inlier_tolerance, ransac_iterations;
  int min_features, max_features, detection_retries;
  camera::CameraParameters cam_params(config, "nav_cam");
  if (!config->GetInt("num_similar", &num_similar))
    ROS_FATAL("num_similar not specified in localization.");
  if (!config->GetInt("ransac_inlier_tolerance", &ransac_inlier_tolerance))
    ROS_FATAL("ransac_inlier_tolerance not specified in localization.");
  if (!config->GetInt("ransac_iterations", &ransac_iterations))
    ROS_FATAL("ransac_iterations not specified in localization.");
  if (!config->GetInt("min_features", &min_features))
    ROS_FATAL("min_features not specified in localization.");
  if (!config->GetInt("max_features", &max_features))
    ROS_FATAL("max_features not specified in localization.");
  if (!config->GetInt("detection_retries", &detection_retries))
    ROS_FATAL("detection_retries not specified in localization.");
  map_->SetCameraParameters(cam_params);
  map_->SetNumSimilar(num_similar);
  map_->SetRansacInlierTolerance(ransac_inlier_tolerance);
  map_->SetRansacIterations(ransac_iterations);
  map_->SetDetectorParams(min_features, max_features, detection_retries);
}

bool Localizer::Localize(cv_bridge::CvImageConstPtr image_ptr, ff_msgs::VisualLandmarks* vl) {
  bool multithreaded = false;
  cv::Mat image_descriptors;
  Eigen::Matrix2Xd image_keypoints;

  vl->header = std_msgs::Header();
  vl->header.stamp = image_ptr->header.stamp;
  vl->header.frame_id = "world";

  map_->DetectFeatures(image_ptr->image, multithreaded, &image_descriptors, &image_keypoints);
  camera::CameraModel camera(Eigen::Vector3d(),
                             Eigen::Matrix3d::Identity(),
                             map_->GetCameraParameters());
  std::vector<Eigen::Vector3d> landmarks;
  std::vector<Eigen::Vector2d> observations;
  if (!map_->Localize(image_descriptors, image_keypoints,
                               &camera, &landmarks, &observations)) {
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
