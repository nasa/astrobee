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
#ifndef OPTIMIZATION_COMMON_IDENTITY_DISTORTER_H_
#define OPTIMIZATION_COMMON_IDENTITY_DISTORTER_H_

#include <optimization_common/distorter.h>

namespace optimization_common {
class IdentityDistorter : public Distorter<0, IdentityDistorter> {
 public:
  using Distorter<0, IdentityDistorter>::Distort;
  using Distorter<0, IdentityDistorter>::Undistort;

  template <typename T>
  Eigen::Matrix<T, 2, 1> Distort(const T* distortion, const Eigen::Matrix<T, 3, 3>& intrinsics,
                                 const Eigen::Matrix<T, 2, 1>& undistorted_point) const {
    return undistorted_point;
  }

  cv::Mat Undistort(const cv::Mat& distorted_image, const Eigen::Matrix3d& intrinsics,
                    const Eigen::VectorXd& distortion) const final {
    return distorted_image;
  }

  Eigen::Vector2d Undistort(const Eigen::Vector2d& distorted_point, const Eigen::Matrix3d& intrinsics,
                            const Eigen::VectorXd& distortion) const final {
    return distorted_point;
  }
};
}  // namespace optimization_common

#endif  // OPTIMIZATION_COMMON_IDENTITY_DISTORTER_H_
