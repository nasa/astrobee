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
#ifndef VISION_COMMON_DISTORTER_H_
#define VISION_COMMON_DISTORTER_H_

#include <Eigen/Core>

#include <opencv2/opencv.hpp>

#include <vector>

namespace vision_common {
template <int NUM_PARAMS, typename DISTORTER>
class Distorter {
 public:
  Eigen::Vector2d Distort(const Eigen::VectorXd& distortion, const Eigen::Matrix3d& intrinsics,
                          const Eigen::Vector2d& undistorted_point) const {
    return static_cast<DISTORTER const*>(this)->Distort(distortion.data(), intrinsics, undistorted_point);
  }

  virtual Eigen::Vector2d Undistort(const Eigen::Vector2d& distorted_point, const Eigen::Matrix3d& intrinsics,
                                    const Eigen::VectorXd& distortion) const = 0;

  std::vector<Eigen::Vector2d> Undistort(const std::vector<Eigen::Vector2d>& distorted_points,
                                         const Eigen::Matrix3d& intrinsics, const Eigen::VectorXd& distortion) const {
    std::vector<Eigen::Vector2d> undistorted_points;
    for (const auto& distorted_point : distorted_points) {
      undistorted_points.emplace_back(Undistort(distorted_point, intrinsics, distortion));
    }
    return undistorted_points;
  }

  virtual cv::Mat Undistort(const cv::Mat& distorted_image, const Eigen::Matrix3d& intrinsics,
                            const Eigen::VectorXd& distortion) const = 0;

  static constexpr int kNumParams = NUM_PARAMS;
};
}  // namespace vision_common

#endif  // VISION_COMMON_DISTORTER_H_
