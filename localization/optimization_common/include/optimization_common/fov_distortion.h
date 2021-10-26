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
#ifndef OPTIMIZATION_COMMON_FOV_DISTORTION_H_
#define OPTIMIZATION_COMMON_FOV_DISTORTION_H_

#include <optimization_common/utilities.h>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>

#include <ceres/ceres.h>

namespace optimization_common {
class FovDistortion {
 public:
  Eigen::Vector2d Distort(const Eigen::VectorXd& distortion, const Eigen::Matrix3d& intrinsics,
                          const Eigen::Vector2d& undistorted_point) const {
    return Distort(distortion.data(), intrinsics, undistorted_point);
  }

  template <typename T>
  Eigen::Matrix<T, 2, 1> Distort(const T* distortion, const Eigen::Matrix<T, 3, 3>& intrinsics,
                                 const Eigen::Matrix<T, 2, 1>& undistorted_point) const {
    // Distortion model expects image coordinates to be in relative coordinates
    const Eigen::Matrix<T, 2, 1> relative_coordinates = RelativeCoordinates(undistorted_point, intrinsics);
    const T& relative_x = relative_coordinates[0];
    const T& relative_y = relative_coordinates[1];

    // Squared norm
    const T r2 = relative_x * relative_x + relative_y * relative_y;
    const T& w = distortion[0];
    T warping;
    if (r2 > 1e-5) {
      const T a = 2.0 * ceres::tan(w / 2.0);
      const T b = ceres::atan(r2 * a) / w;
      warping = b / r2;
    } else {
      warping = T(1.0);
    }
    const T distorted_relative_x = warping * relative_x;
    const T distorted_relative_y = warping * relative_y;
    return AbsoluteCoordinates(Eigen::Matrix<T, 2, 1>(distorted_relative_x, distorted_relative_y), intrinsics);
  }

  cv::Mat Undistort(const cv::Mat& distorted_image, const Eigen::Matrix3d& intrinsics,
                    const Eigen::VectorXd& distortion) {
    cv::Mat undistorted_image(distorted_image.size(), CV_8UC1, 0);
    for (int i = 0; i < undistorted_image.rows; ++i) {
      for (int j = 0; j < undistorted_image.cols; ++j) {
        const std::uint8_t pixel_val = distorted_image.at<std::uint8_t>(i, j);
        const auto undistorted_point = Undistort(Eigen::Vector2d(i, j), distortion);
        if (!undistorted_point) continue;
        if (undistorted_point->x() >= undistorted_image.cols || undistorted_point->x() < 0) continue;
        if (undistorted_point->y() >= undistorted_image.rows || undistorted_point->y() < 0) continue;
        undistorted_image.at<std::uint8_t>(i, j) = pixel_val;
      }
    }
    return undistorted_image;
  }

  boost::optional<Eigen::Vector2i> Undistort(const Eigen::Vector2d& distorted_point,
                                             const Eigen::VectorXd& distortion) {
    const double w = distortion[0];
    // TODO(rsoussan): Clean this up
    // Adapted from Kalibr
    // TODO(rsoussan): Merge with camera params implementation?
    const double mul2tanwby2 = std::tan(w / 2.0) * 2.0;
    // Calculate distance from point to center.
    double r_d = distorted_point.norm();
    if (mul2tanwby2 == 0 || r_d == 0) {
      return boost::none;
    }

    // Calculate undistorted radius of point.
    double r_u;
    static constexpr double kMaxValidAngle = (89.0 * M_PI / 180.0);
    if (std::fabs(r_d * w) <= kMaxValidAngle) {
      r_u = std::tan(r_d * w) / (r_d * mul2tanwby2);
    } else {
      return boost::none;
    }

    const Eigen::Vector2d undistorted_point = distorted_point * r_u;
    const Eigen::Vector2i undistorted_rounded_point(std::round(undistorted_point[0]), std::round(undistorted_point[1]));
    return undistorted_rounded_point;
  }

  static constexpr int NUM_PARAMS = 1;
};
}  // namespace optimization_common

#endif  // OPTIMIZATION_COMMON_FOV_DISTORTION_H_
