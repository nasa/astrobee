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

#ifndef LOCALIZATION_COMMON_TEST_UTILITIES_H_
#define LOCALIZATION_COMMON_TEST_UTILITIES_H_

#include <gtsam/geometry/Pose3.h>

#include <gtest/gtest.h>

#include <vector>

namespace localization_common {
class Sampler {
 public:
  Sampler(const double min, const double max, const double count);
  double Sample(const int increment) const;

 private:
  const double scale_;
  const double min_;
};

// Selected from [-100, 100]
double RandomDouble();

// Selected from [0, 100]
double RandomPositiveDouble();

// Selected from [min, max]
double RandomDouble(const double min, const double max);

double RandomGaussianDouble(const double mean, const double stddev);

bool RandomBool();

// Returns white noise with set stddev
double Noise(const double stddev);

// Each index ranges from [-100, 100]
template <int Dim>
Eigen::Matrix<double, Dim, 1> RandomVector();

Eigen::Vector3d RandomVector3d();

Eigen::Vector3d RandomPoint3d();

Eigen::Vector2d RandomVector2d();

Eigen::Vector2d RandomPoint2d();

// Translation ranges from [-100, 100]
// Rotation spans all possible rotations
gtsam::Pose3 RandomPose();

Eigen::Isometry3d RandomIsometry3d();

Eigen::Affine3d RandomAffine3d();

// Focal lengths and principal points selected from [0.1, 1000]
Eigen::Matrix3d RandomIntrinsics();

// Adds noise to identity Isometry3d
Eigen::Isometry3d RandomIdentityCenteredIsometry3d(const double translation_stddev, const double rotation_stddev);

Eigen::Isometry3d AddNoiseToIsometry3d(const Eigen::Isometry3d& pose, const double translation_stddev,
                                       const double rotation_stddev);

gtsam::Pose3 AddNoiseToPose(const gtsam::Pose3& pose, const double translation_stddev, const double rotation_stddev);

template <int N>
Eigen::Matrix<double, N, 1> AddNoiseToVector(const Eigen::Matrix<double, N, 1>& vector, const double noise_stddev);

// Samples in cylindrical coordinates for pose translation to keep pose in view frustrum.
// Samples z using scaled rho value to prevent large z vals with small rho values
// that may move the pose out of the view frustrum.
Eigen::Isometry3d RandomFrontFacingPose(const double rho_min, const double rho_max, const double phi_min,
                                        const double phi_max, const double z_rho_scale, const double yaw_min,
                                        const double yaw_max, const double pitch_min, const double pitch_max,
                                        const double roll_min, const double roll_max);

Eigen::Isometry3d RandomFrontFacingPose();

std::vector<Eigen::Vector3d> RandomFrontFacingPoints(const int num_points);

Eigen::Vector3d RandomFrontFacingPoint();

Eigen::Vector3d RandomFrontFacingPoint(const double rho_min, const double rho_max, const double phi_min,
                                       const double phi_max, const double z_rho_scale);

bool MatrixEquality(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs, const double tolerance);

// Implementation
template <int Dim>
Eigen::Matrix<double, Dim, 1> RandomVector() {
  // Eigen::Matrix::Random() is constrained to [-1, 1]
  return RandomDouble() * Eigen::Matrix<double, Dim, 1>::Random();
}

template <int N>
Eigen::Matrix<double, N, 1> AddNoiseToVector(const Eigen::Matrix<double, N, 1>& vector, const double noise_stddev) {
  Eigen::Matrix<double, N, 1> noisy_vector = vector;
  for (int i = 0; i < vector.rows(); ++i) {
    noisy_vector(i, 0) += Noise(noise_stddev);
  }
  return noisy_vector;
}
}  // namespace localization_common

// Add GTEST like MACROS with no namespace so these mirror GTEST calls
// clang-format off
#define EXPECT_MATRIX_NEAR(lhs, rhs, tol) \
  do { \
    EXPECT_PRED3(localization_common::MatrixEquality, lhs.matrix(), rhs.matrix(), tol); \
  } while (0)

#endif  // LOCALIZATION_COMMON_TEST_UTILITIES_H_
