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
#ifndef CALIBRATION_UTILITIES_H_
#define CALIBRATION_UTILITIES_H_

#include <calibration/camera_utilities.h>
#include <localization_common/image_correspondences.h>
#include <optimization_common/utilities.h>

#include <Eigen/Geometry>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <boost/optional.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>

namespace calibration {
template <typename DISTORTER>
void SaveReprojectionErrors(const std::vector<Eigen::Matrix<double, 6, 1>>& camera_T_targets,
                            const std::vector<localization_common::ImageCorrespondences>& valid_match_sets,
                            const Eigen::Matrix3d& intrinsics, const Eigen::VectorXd& distortion,
                            const Eigen::Vector2i& image_size, const double max_error_norm = 100) {
  cv::Mat reprojection_image_grayscale(image_size.y(), image_size.x(), CV_8UC1, cv::Scalar(0));
  std::ofstream errors_file;
  errors_file.open("errors_file.txt");
  for (int i = 0; i < static_cast<int>(valid_match_sets.size()); ++i) {
    const auto& match_set = valid_match_sets[i];
    const Eigen::Isometry3d camera_T_target = optimization_common::Isometry3(camera_T_targets[i].data());
    for (int j = 0; j < static_cast<int>(match_set.image_points.size()); ++j) {
      const auto& image_point = match_set.image_points[j];
      const auto& target_point = match_set.points_3d[j];
      const Eigen::Vector3d camera_t_target_point = camera_T_target * target_point;
      const Eigen::Vector2d projected_image_point =
        Project3dPointToImageSpaceWithDistortion<DISTORTER>(camera_t_target_point, intrinsics, distortion);
      const Eigen::Vector2d error = (image_point - projected_image_point);
      const double error_norm = error.norm();
      errors_file << error.x() << " " << error.y() << std::endl;
      const cv::Point2i rounded_image_point(std::round(image_point.x()), std::round(image_point.y()));
      // Add 1 to each value so background pixels stay white and we can map these back to white
      // after applying colormap.
      // Only map up to 235 since darker reds that occur from 235-255 are hard to differentiate from
      // darker blues from 0 to 20 or so.
      const int error_color = std::round(std::min(error_norm, max_error_norm) / max_error_norm * 235.0) + 1;
      cv::circle(reprojection_image_grayscale, rounded_image_point, 4, cv::Scalar(error_color), -1);
    }
  }
  // TODO(rsoussan): pass filepath!
  cv::Mat reprojection_image_color;
  cv::applyColorMap(reprojection_image_grayscale, reprojection_image_color, cv::COLORMAP_JET);
  // Map white pixels back from lowest JET value (128, 0, 0) to white
  cv::Mat base_mask;
  cv::inRange(reprojection_image_color, cv::Scalar(128, 0, 0), cv::Scalar(128, 0, 0), base_mask);
  reprojection_image_color.setTo(cv::Scalar(255, 255, 255), base_mask);
  cv::imwrite("reprojection_image.jpg", reprojection_image_color);
  errors_file.close();
}

}  // namespace calibration

#endif  // CALIBRATION_UTILITIES_H_
