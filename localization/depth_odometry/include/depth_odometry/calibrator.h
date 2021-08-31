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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/problem.h>
#include <ceres/jet.h>

namespace depth_odometry {
class Calibrator {
 public:
  Calibrator(const CalibratorParams& params) : params_(params) {}
  Eigen::Affine3d Calibrate(const std::vector<DepthMatches>& match_sets,
                            const Eigen::Affine3d& initial_depth_image_A_depth_cloud);

  // Assumes compact quaternion parameterization for rotations
  // TODO(rsoussan): Use exponential map with local parameterization and compact axis angle parameterization
  template <typename T>
  static Eigen::Transform<T, 3, Eigen::Affine> Affine3(const T* affine_data) {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> compact_quaternion(affine_data);
    Eigen::Matrix<T, 3, 3> rotation;
    // For a quaternion, sqrt(x^2+y^2+z^2+w^2) = 1
    // Since a compact quaternion provides the x,y,z compenents, to recover w use:
    // w^2 = 1 - (x^2 + y^2 + z^2)
    const T w_squared = 1.0 - compact_quaternion.squaredNorm();
    // Catch invalid quaternion
    if (w_squared < 0.0) {
      rotation = Eigen::Matrix<T, 3, 3>::Identity();
    } else {
      const Eigen::Quaternion<T> quaternion(ceres::sqrt(w_squared), compact_quaternion[0], compact_quaternion[1],
                                            compact_quaternion[2]);
      rotation = Eigen::Matrix<T, 3, 3>(quaternion);
    }
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(&affine_data[3]);
    const T scale = affine_data[6];
    const Eigen::Matrix<T, 3, 3> scale_matrix(Eigen::Matrix<T, 3, 3>::Identity() * scale);
    Eigen::Transform<T, 3, Eigen::Affine> affine_3;
    affine_3.linear() = scale_matrix * rotation;
    affine_3.translation() = translation;
    // TODO(rsoussan): why doesnt't this work?
    // affine_3.fromPositionOrientationScale(translation, rotation, scale_matrix);
    {
      // Eigen::Map<const Eigen::Matrix<T, 7, 1>> vec(affine_data);
      // Eigen::Map<const Eigen::Matrix<T, 3, 1>> vec(affine_data);
      // LogError("affine data: " << vec.matrix());
      std::cout << "affine data: ";
      for (int i = 0; i < 7; ++i) {
        std::cout << affine_data[i] << ", ";
      }
      std::cout << std::endl;
      std::cout << "affine mat: ";
      for (int row = 0; row < 4; ++row) {
        std::cout << std::endl;
        for (int col = 0; col < 4; ++col) {
          int index = row + col * 4;
          std::cout << affine_3.data()[index] << ", ";
        }
      }
      std::cout << std::endl;
    }
    return affine_3;
  }

 private:
  void AddCostFunction(const Eigen::Vector2d& image_point, const Eigen::Vector3d& point_3d,
                       const Eigen::Matrix3d& intrinsics_matrix,
                       Eigen::Matrix<double, 7, 1>& depth_image_A_depth_cloud_vector, ceres::Problem& problem);

  static Eigen::Matrix<double, 7, 1> VectorFromAffine3d(const Eigen::Affine3d& affine_3d);

  CalibratorParams params_;
};

class ReprojectionError {
 public:
  ReprojectionError(const Eigen::Matrix3d& intrinsics_matrix, const Eigen::Vector2d& image_feature,
                    const Eigen::Vector3d& depth_cloud_F_point_3d_feature)
      : intrinsics_matrix_(intrinsics_matrix),
        image_feature_(image_feature),
        depth_cloud_F_point_3d_feature_(depth_cloud_F_point_3d_feature) {}

  template <typename T>
  bool operator()(const T* depth_image_A_depth_cloud_array, T* reprojection_error) const {
    const auto depth_image_A_depth_cloud = Calibrator::Affine3<T>(depth_image_A_depth_cloud_array);
    const Eigen::Matrix<T, 3, 1> depth_image_F_point_3d =
      depth_image_A_depth_cloud * depth_cloud_F_point_3d_feature_.cast<T>();
    const Eigen::Matrix<T, 2, 1> reprojected_pixel =
      (intrinsics_matrix_.cast<T>() * depth_image_F_point_3d).hnormalized();

    reprojection_error[0] = image_feature_[0] - reprojected_pixel[0];
    reprojection_error[1] = image_feature_[1] - reprojected_pixel[1];
    return true;
  }

 private:
  Eigen::Matrix3d intrinsics_matrix_;
  Eigen::Vector2d image_feature_;
  Eigen::Vector3d depth_cloud_F_point_3d_feature_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_CALIBRATOR_H_
