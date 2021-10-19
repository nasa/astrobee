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
#ifndef DEPTH_ODOMETRY_CALIBRATOR_H_
#define DEPTH_ODOMETRY_CALIBRATOR_H_

#include <depth_odometry/calibrator_params.h>
#include <depth_odometry/depth_matches.h>
#include <depth_odometry/optimization_utilities.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/problem.h>
#include <ceres/jet.h>

namespace depth_odometry {
class Calibrator {
 public:
  Calibrator(const CalibratorParams& params) : params_(params) {}
  void Calibrate(const std::vector<DepthMatches>& match_sets, const Eigen::Affine3d& initial_depth_image_A_depth_cloud,
                 const Eigen::Matrix3d& initial_intrinsics, const Eigen::Matrix<double, 4, 1>& intial_distortion,
                 Eigen::Affine3d& calibrated_depth_image_A_depth_cloud, Eigen::Matrix3d& calibrated_intrinsics,
                 Eigen::Matrix<double, 4, 1>& calibrated_distortion);

  const CalibratorParams& params() { return params_; }

 private:
  void AddCostFunction(const Eigen::Vector2d& image_point, const Eigen::Vector3d& point_3d,
                       Eigen::Matrix<double, 7, 1>& depth_image_A_depth_cloud_vector,
                       Eigen::Matrix<double, 4, 1>& intrinsics_vector, Eigen::Matrix<double, 4, 1>& distortion,
                       ceres::Problem& problem);

  CalibratorParams params_;
};

class ReprojectionError {
 public:
  ReprojectionError(const Eigen::Vector2d& image_point, const Eigen::Vector3d& depth_cloud_F_point_3d)
      : image_point_(image_point), depth_cloud_F_point_3d_(depth_cloud_F_point_3d) {}

  template <typename T>
  bool operator()(const T* depth_image_A_depth_cloud_data, const T* intrinsics_data, const T* distortion_data,
                  T* reprojection_error) const {
    // Handle type conversions
    const auto intrinsics = Intrinsics<T>(intrinsics_data);
    const auto depth_image_A_depth_cloud = Affine3<T>(depth_image_A_depth_cloud_data);

    // Compute error
    const Eigen::Matrix<T, 3, 1> depth_image_F_point_3d = depth_image_A_depth_cloud * depth_cloud_F_point_3d_.cast<T>();
    const Eigen::Matrix<T, 2, 1> undistorted_reprojected_point_3d = (intrinsics * depth_image_F_point_3d).hnormalized();
    const Eigen::Matrix<T, 2, 1> distorted_reprojected_point_3d =
      Distort(distortion_data, intrinsics_data, undistorted_reprojected_point_3d);

    reprojection_error[0] = image_point_[0] - distorted_reprojected_point_3d[0];
    reprojection_error[1] = image_point_[1] - distorted_reprojected_point_3d[1];
    return true;
  }

 private:
  Eigen::Vector2d image_point_;
  Eigen::Vector3d depth_cloud_F_point_3d_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_CALIBRATOR_H_
