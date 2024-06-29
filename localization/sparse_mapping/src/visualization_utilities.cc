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

#include <sparse_mapping/visualization_utilities.h>

#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

namespace sparse_mapping {
std::vector<cv::KeyPoint> Keypoints(const Eigen::Matrix2Xd& keypoints_mat) {
  std::vector<cv::KeyPoint> keypoints;
  for (int i = 0; i < keypoints_mat.cols(); ++i) {
    keypoints.emplace_back(cv::KeyPoint(keypoints_mat.col(i).x(), keypoints_mat.col(i).y(), 1.0));
  }
  return keypoints;
}

std::vector<cv::KeyPoint> DistortedKeypoints(const Eigen::Matrix2Xd& keypoints_mat,
                                             const camera::CameraParameters& camera_params) {
  std::vector<cv::KeyPoint> keypoints;
  for (int i = 0; i < keypoints_mat.cols(); ++i) {
    Eigen::Vector2d distorted_point;
    camera_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(keypoints_mat.col(i), &distorted_point);
    keypoints.emplace_back(cv::KeyPoint(distorted_point.x(), distorted_point.y(), 1.0));
  }
  return keypoints;
}

void ViewMatches(const Eigen::Matrix2Xd& input_keypoints_mat, const Eigen::Matrix2Xd& map_keypoints_mat,
                 const std::vector<cv::DMatch>& matches, const camera::CameraParameters& camera_params,
                 const cv::Mat& input_image, const cv::Mat& map_image) {
  const auto input_keypoints = DistortedKeypoints(input_keypoints_mat, camera_params);
  const auto map_keypoints = DistortedKeypoints(map_keypoints_mat, camera_params);
  cv::Mat input_keypoints_image, map_keypoints_image;
  cv::drawKeypoints(input_image, input_keypoints, input_keypoints_image);
  cv::drawKeypoints(map_image, map_keypoints, map_keypoints_image);
  cv::Mat keypoints_image;
  cv::resize(input_keypoints_image, input_keypoints_image, cv::Size(1.1 * 960, 1.1 * 540));
  cv::resize(map_keypoints_image, map_keypoints_image, cv::Size(1.1 * 960, 1.1 * 540));
  cv::Mat combined_image;
  cv::hconcat(input_keypoints_image, map_keypoints_image, combined_image);
  cv::imshow("Input and Map Keypoints", combined_image);
  cv::waitKey(0);
  cv::destroyAllWindows();
  cv::Mat matches_image;
  cv::drawMatches(input_image, input_keypoints, map_image, map_keypoints, matches, matches_image, cv::Scalar::all(-1),
                  cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  cv::resize(matches_image, matches_image, cv::Size(1.7 * 960, 1.7 * 540));
  cv::imshow("Input to Map Matches", matches_image);
  cv::waitKey(0);
  cv::destroyAllWindows();
}
}  // namespace sparse_mapping
