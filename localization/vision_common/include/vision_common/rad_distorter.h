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
#ifndef VISION_COMMON_RAD_DISTORTER_H_
#define VISION_COMMON_RAD_DISTORTER_H_

#include <vision_common/distorter.h>
#include <vision_common/radtan_distorter.h>

#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>

namespace vision_common {
class RadDistorter : public Distorter<2, RadDistorter> {
 public:
  using Distorter<2, RadDistorter>::Distort;
  using Distorter<2, RadDistorter>::Undistort;

  template <typename T>
  Eigen::Matrix<T, 2, 1> Distort(const T* distortion, const Eigen::Matrix<T, 3, 3>& intrinsics,
                                 const Eigen::Matrix<T, 2, 1>& undistorted_point) const {
    T radtan_distortion[4];
    radtan_distortion[0] = distortion[0];
    radtan_distortion[1] = distortion[1];
    radtan_distortion[2] = T(0.0);
    radtan_distortion[3] = T(0.0);
    return radtan_distorter_.Distort(radtan_distortion, intrinsics, undistorted_point);
  }

  cv::Mat Undistort(const cv::Mat& distorted_image, const Eigen::Matrix3d& intrinsics,
                    const Eigen::VectorXd& distortion) const final {
    const Eigen::VectorXd radtan_distortion = RadTanDistortionVector(distortion);
    return radtan_distorter_.Undistort(distorted_image, intrinsics, radtan_distortion);
  }

  Eigen::Vector2d Undistort(const Eigen::Vector2d& distorted_point, const Eigen::Matrix3d& intrinsics,
                            const Eigen::VectorXd& distortion) const final {
    const Eigen::VectorXd radtan_distortion = RadTanDistortionVector(distortion);
    return radtan_distorter_.Undistort(distorted_point, intrinsics, radtan_distortion);
  }

 private:
  Eigen::VectorXd RadTanDistortionVector(const Eigen::VectorXd& rad_distortion) const {
    Eigen::VectorXd radtan_distortion(4);
    radtan_distortion[0] = rad_distortion[0];
    radtan_distortion[1] = rad_distortion[1];
    radtan_distortion[2] = 0;
    radtan_distortion[3] = 0;
    return radtan_distortion;
  }

  RadTanDistorter radtan_distorter_;
};
}  // namespace vision_common

#endif  // VISION_COMMON_RAD_DISTORTER_H_
