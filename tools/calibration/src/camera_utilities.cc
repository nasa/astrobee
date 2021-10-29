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

#include <calibration/camera_utilities.h>

#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <random>
#include <unordered_map>

namespace calibration {
Eigen::Vector2d Project3dPointToImageSpace(const Eigen::Vector3d& cam_t_point, const Eigen::Matrix3d& intrinsics) {
  return (intrinsics * cam_t_point).hnormalized();
}

Eigen::Isometry3d Isometry3d(const cv::Mat& rodrigues_rotation_cv, const cv::Mat& translation_cv) {
  Eigen::Vector3d translation;
  cv::cv2eigen(translation_cv, translation);
  Eigen::Matrix3d rotation;
  cv::Mat rotation_cv;
  cv::Rodrigues(rodrigues_rotation_cv, rotation_cv);
  cv::cv2eigen(rotation_cv, rotation);
  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  pose.translation() = translation;
  pose.linear() = rotation;
  return pose;
}
}  // namespace calibration
