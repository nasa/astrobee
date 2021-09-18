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
#ifndef DEPTH_ODOMETRY_INTRINSICS_CALIBRATOR_H_
#define DEPTH_ODOMETRY_INTRINSICS_CALIBRATOR_H_

#include <camera/camera_params.h>
#include <depth_odometry/calibrator_params.h>
#include <depth_odometry/image_correspondences.h>
#include <localization_common/logger.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/problem.h>
#include <ceres/jet.h>

namespace depth_odometry {
class IntrinsicsCalibrator {
 public:
  IntrinsicsCalibrator(const CalibratorParams& params) : params_(params) {}
  void Calibrate(const std::vector<ImageCorrespondences>& match_sets, const camera::CameraParameters& camera_params,
                 const Eigen::Matrix3d& initial_intrinsics, const Eigen::Matrix<double, 4, 1>& initial_distortion,
                 Eigen::Matrix3d& calibrated_intrinsics, Eigen::Matrix<double, 4, 1>& calibrated_distortion);

  // Assumes compact quaternion parameterization for rotations
  // TODO(rsoussan): Use exponential map with local parameterization and compact axis angle parameterization
  template <typename T>
  static Eigen::Transform<T, 3, Eigen::Isometry> Isometry3(const T* isometry_data) {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> compact_quaternion(isometry_data);
    Eigen::Matrix<T, 3, 3> rotation;
    // For a quaternion, sqrt(x^2+y^2+z^2+w^2) = 1
    // Since a compact quaternion provides the x,y,z compenents, to recover w use:
    // w^2 = 1 - (x^2 + y^2 + z^2)
    const T w_squared = 1.0 - compact_quaternion.squaredNorm();
    // Catch invalid quaternion
    if (w_squared <= 0.0) {
      rotation = Eigen::Matrix<T, 3, 3>::Identity();
    } else {
      const Eigen::Quaternion<T> quaternion(ceres::sqrt(w_squared), compact_quaternion[0], compact_quaternion[1],
                                            compact_quaternion[2]);
      rotation = Eigen::Matrix<T, 3, 3>(quaternion);
    }
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(&isometry_data[3]);
    Eigen::Transform<T, 3, Eigen::Isometry> isometry_3;
    isometry_3.linear() = rotation;
    isometry_3.translation() = translation;
    return isometry_3;
  }

  // Assumes storage as focal lengths followed by principal points
  template <typename T>
  static Eigen::Matrix<T, 3, 3> Intrinsics(const T* intrinsics_data) {
    Eigen::Matrix<T, 3, 3> intrinsics(Eigen::Matrix<T, 3, 3>::Identity());
    intrinsics(0, 0) = intrinsics_data[0];
    intrinsics(1, 1) = intrinsics_data[1];
    intrinsics(0, 2) = intrinsics_data[2];
    intrinsics(1, 2) = intrinsics_data[3];
    return intrinsics;
  }

  template <typename T>
  static Eigen::Matrix<T, 2, 1> Distort(const T* distortion, const T* intrinsics,
                                        const Eigen::Matrix<T, 2, 1>& undistorted_point) {
    const T& k1 = distortion[0];
    const T& k2 = distortion[1];
    const T& p1 = distortion[2];
    const T& p2 = distortion[3];
    // TODO(rsoussan): Support 5 distortion params?
    const T k3(0.0);

    const T& f_x = intrinsics[0];
    const T& f_y = intrinsics[1];
    const T& p_x = intrinsics[2];
    const T& p_y = intrinsics[3];

    // Distortion model expects image coordinates to be in relative coordinates
    const T& x = undistorted_point[0];
    const T& y = undistorted_point[1];
    const T relative_x = (x - p_x) / f_x;
    const T relative_y = (y - p_y) / f_y;

    // Squared norm
    const T r2 = relative_x * relative_x + relative_y * relative_y;

    // Apply radial distortion
    const T radial_distortion_coeff = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    T distorted_relative_x = relative_x * radial_distortion_coeff;
    T distorted_relative_y = relative_y * radial_distortion_coeff;

    // Apply tangential distortion
    distorted_relative_x =
      distorted_relative_x + (2.0 * p1 * relative_x * relative_y + p2 * (r2 + 2.0 * relative_x * relative_x));
    distorted_relative_y =
      distorted_relative_y + (p1 * (r2 + 2.0 * relative_y * relative_y) + 2.0 * p2 * relative_x * relative_y);

    // Convert back to absolute coordinates
    const Eigen::Matrix<T, 2, 1> distorted_point(distorted_relative_x * f_x + p_x, distorted_relative_y * f_y + p_y);
    return distorted_point;
  }

  const CalibratorParams& params() { return params_; }

 private:
  void AddCostFunction(const Eigen::Vector2d& image_point, const Eigen::Vector3d& point_3d,
                       Eigen::Matrix<double, 6, 1>& camera_T_target, Eigen::Matrix<double, 4, 1>& intrinsics_vector,
                       Eigen::Matrix<double, 4, 1>& distortion, ceres::Problem& problem);

  static Eigen::Matrix<double, 6, 1> VectorFromIsometry3d(const Eigen::Isometry3d& isometry_3d);

  static Eigen::Matrix<double, 4, 1> VectorFromIntrinsicsMatrix(const Eigen::Matrix3d& intrinsics);

  CalibratorParams params_;
};

class ReprojectionError {
 public:
  ReprojectionError(const Eigen::Vector2d& image_point, const Eigen::Vector3d& target_t_point_3d)
      : image_point_(image_point), target_t_point_3d_(target_t_point_3d) {}

  template <typename T>
  bool operator()(const T* camera_T_target_data, const T* intrinsics_data, const T* distortion_data,
                  T* reprojection_error) const {
    // Handle type conversions
    const auto intrinsics = IntrinsicsCalibrator::Intrinsics<T>(intrinsics_data);
    const auto camera_T_target = IntrinsicsCalibrator::Isometry3<T>(camera_T_target_data);
    // Compute error
    const Eigen::Matrix<T, 3, 1> camera_t_point_3d = camera_T_target * target_t_point_3d_.cast<T>();
    const Eigen::Matrix<T, 2, 1> undistorted_reprojected_point_3d = (intrinsics * camera_t_point_3d).hnormalized();
    const Eigen::Matrix<T, 2, 1> distorted_reprojected_point_3d =
      IntrinsicsCalibrator::Distort(distortion_data, intrinsics_data, undistorted_reprojected_point_3d);

    reprojection_error[0] = image_point_[0] - distorted_reprojected_point_3d[0];
    reprojection_error[1] = image_point_[1] - distorted_reprojected_point_3d[1];
    return true;
  }

 private:
  Eigen::Vector2d image_point_;
  Eigen::Vector3d target_t_point_3d_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_INTRINSICS_CALIBRATOR_H_
