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
#ifndef VISION_COMMON_UTILITIES_H_
#define VISION_COMMON_UTILITIES_H_

#include <vision_common/feature_point.h>

#include <Eigen/Geometry>

#include <gtsam/base/Matrix.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <vector>

namespace vision_common {
template <typename T>
Eigen::Matrix<T, 2, 1> RelativeCoordinates(const Eigen::Matrix<T, 2, 1>& absolute_point,
                                           const Eigen::Matrix<T, 3, 3>& intrinsics);
template <typename T>
Eigen::Matrix<T, 2, 1> AbsoluteCoordinates(const Eigen::Matrix<T, 2, 1>& relative_point,
                                           const Eigen::Matrix<T, 3, 3>& intrinsics);

Eigen::Vector3d Backproject(const Eigen::Vector2d& measurement, const Eigen::Matrix3d& intrinsics, const double depth,
                            gtsam::OptionalJacobian<3, 1> d_backprojected_point_d_depth = boost::none);

Eigen::Vector2d FocalLengths(const Eigen::Matrix3d& intrinsics);

Eigen::Vector2d PrincipalPoints(const Eigen::Matrix3d& intrinsics);

Eigen::Vector2d Project(const Eigen::Vector3d& cam_t_point, const Eigen::Matrix3d& intrinsics,
                        gtsam::OptionalJacobian<2, 3> d_projected_point_d_cam_t_point = boost::none);

template <typename DISTORTER>
Eigen::Vector2d ProjectWithDistortion(const Eigen::Vector3d& cam_t_point, const Eigen::Matrix3d& intrinsics,
                                      const Eigen::VectorXd& distortion_params);

Eigen::Isometry3d Isometry3d(const cv::Mat& rodrigues_rotation_cv, const cv::Mat& translation_cv);

// TODO(rsoussan): Rename these
bool ValidPointSet(const int num_points, const double average_distance_from_mean,
                   const double min_avg_distance_from_mean, const int min_num_points);

double AverageDistanceFromMean(const std::vector<FeaturePoint>& points);

// Implementation
template <typename T>
Eigen::Matrix<T, 2, 1> RelativeCoordinates(const Eigen::Matrix<T, 2, 1>& absolute_point,
                                           const Eigen::Matrix<T, 3, 3>& intrinsics) {
  const T& f_x = intrinsics(0, 0);
  const T& f_y = intrinsics(1, 1);
  const T& p_x = intrinsics(0, 2);
  const T& p_y = intrinsics(1, 2);

  const T& x = absolute_point[0];
  const T& y = absolute_point[1];
  const T relative_x = (x - p_x) / f_x;
  const T relative_y = (y - p_y) / f_y;
  return Eigen::Matrix<T, 2, 1>(relative_x, relative_y);
}

template <typename T>
Eigen::Matrix<T, 2, 1> AbsoluteCoordinates(const Eigen::Matrix<T, 2, 1>& relative_point,
                                           const Eigen::Matrix<T, 3, 3>& intrinsics) {
  const T& f_x = intrinsics(0, 0);
  const T& f_y = intrinsics(1, 1);
  const T& p_x = intrinsics(0, 2);
  const T& p_y = intrinsics(1, 2);
  return Eigen::Matrix<T, 2, 1>(relative_point[0] * f_x + p_x, relative_point[1] * f_y + p_y);
}

template <typename DISTORTER>
Eigen::Vector2d ProjectWithDistortion(const Eigen::Vector3d& cam_t_point, const Eigen::Matrix3d& intrinsics,
                                      const Eigen::VectorXd& distortion_params) {
  const Eigen::Vector2d undistorted_image_point = Project(cam_t_point, intrinsics);
  const DISTORTER distorter;
  return distorter.Distort(distortion_params, intrinsics, undistorted_image_point);
}
}  // namespace vision_common

#endif  // VISION_COMMON_UTILITIES_H_
