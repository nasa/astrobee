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

#ifndef SPARSE_MAPPING_VISUALIZATION_UTILITIES_H_
#define SPARSE_MAPPING_VISUALIZATION_UTILITIES_H_

#include <camera/camera_params.h>

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

namespace sparse_mapping {
// Convert keypoints from matrix to cv::KeyPoints
std::vector<cv::KeyPoint> Keypoints(const Eigen::Matrix2Xd& keypoints_mat);

// Convert keypoints from undistorted centered matrix to distorted cv::KeyPoints
std::vector<cv::KeyPoint> DistortedKeypoints(const Eigen::Matrix2Xd& keypoints_mat,
                                             const camera::CameraParameters& camera_params);

// View input keypoints, map keypoints, and matches
void ViewMatches(const Eigen::Matrix2Xd& input_keypoints_mat, const Eigen::Matrix2Xd& map_keypoints_mat,
                 const std::vector<cv::DMatch>& matches, const camera::CameraParameters& camera_params,
                 const cv::Mat& input_image, const cv::Mat& map_image);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_VISUALIZATION_UTILITIES_H_
