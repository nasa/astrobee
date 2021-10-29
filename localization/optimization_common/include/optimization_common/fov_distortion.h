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

#include <optimization_common/distortion.h>
#include <optimization_common/utilities.h>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>

#include <ceres/ceres.h>

namespace optimization_common {
class FovDistortion : public Distortion<1, FovDistortion> {
 public:
  using Distortion<1, FovDistortion>::Distort;

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
                    const Eigen::VectorXd& distortion) const final {
    cv::Mat gray_distorted_image;
    cv::cvtColor(distorted_image, gray_distorted_image, CV_BGR2GRAY);
    cv::Mat undistorted_image(distorted_image.size(), CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < undistorted_image.rows; ++y) {
      for (int x = 0; x < undistorted_image.cols; ++x) {
        const uchar pixel_val = gray_distorted_image.at<uchar>(y, x);
        const Eigen::Vector2d undistorted_point = Undistort(Eigen::Vector2d(x, y), intrinsics, distortion);
        const Eigen::Vector2i undistorted_rounded_point(std::round(undistorted_point[0]),
                                                        std::round(undistorted_point[1]));
        if (undistorted_rounded_point.x() >= undistorted_image.cols || undistorted_rounded_point.x() < 0) continue;
        if (undistorted_rounded_point.y() >= undistorted_image.rows || undistorted_rounded_point.y() < 0) continue;
        undistorted_image.at<uchar>(undistorted_rounded_point.y(), undistorted_rounded_point.x()) = pixel_val;
      }
    }
    return undistorted_image;
  }

  Eigen::Vector2d Undistort(const Eigen::Vector2d& distorted_point, const Eigen::Matrix3d& intrinsics,
                            const Eigen::VectorXd& distortion) const final {
    const Eigen::Vector2d relative_distorted_point = RelativeCoordinates(distorted_point, intrinsics);
    const double rd = relative_distorted_point.norm();
    const double a = 2.0 * std::tan(distortion[0] / 2.0);
    const double ru = std::tan(rd * distortion[0]) / a;
    const double unwarping = rd > 1e-5 ? ru / rd : 1.0;
    const Eigen::Vector2d relative_undistorted_point = unwarping * relative_distorted_point;
    const Eigen::Vector2d undistorted_point = AbsoluteCoordinates(relative_undistorted_point, intrinsics);
    return undistorted_point;
  }
};
}  // namespace optimization_common

#endif  // OPTIMIZATION_COMMON_FOV_DISTORTION_H_
