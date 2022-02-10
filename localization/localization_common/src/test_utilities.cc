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

#include <localization_common/utilities.h>
#include <localization_common/test_utilities.h>

#include <random>

namespace localization_common {
Sampler::Sampler(const double min, const double max, const double count)
    : min_(min), scale_((max - min) / static_cast<double>(count - 1)) {}

double Sampler::Sample(const int increment) const { return min_ + increment * scale_; }

double RandomDouble() { return RandomDouble(-100, 100); }

double RandomPositiveDouble() { return RandomDouble(0, 100); }

double RandomDouble(const double min, const double max) {
  std::random_device dev;
  std::mt19937 rng(dev());
  return std::uniform_real_distribution<double>(min, max)(rng);
}

double RandomGaussianDouble(const double mean, const double stddev) {
  std::random_device dev;
  std::mt19937 rng(dev());
  std::normal_distribution<double> distribution(mean, stddev);
  return distribution(rng);
}

bool RandomBool() { return RandomDouble(0, 1) < 0.5; }

double Noise(const double stddev) { return RandomGaussianDouble(0.0, stddev); }

Eigen::Vector3d RandomVector3d() { return RandomVector<3>(); }

Eigen::Vector3d RandomPoint3d() { return RandomVector<3>(); }

Eigen::Vector2d RandomVector2d() { return RandomVector<2>(); }

Eigen::Vector2d RandomPoint2d() { return RandomVector<2>(); }

gtsam::Pose3 RandomPose() {
  std::random_device dev;
  std::mt19937 rng(dev());
  gtsam::Rot3 rot = gtsam::Rot3::Random(rng);
  gtsam::Point3 trans = RandomVector<3>();
  return gtsam::Pose3(rot, trans);
}

Eigen::Isometry3d RandomIsometry3d() {
  const gtsam::Pose3 random_pose = RandomPose();
  return EigenPose(random_pose);
}

Eigen::Affine3d RandomAffine3d() {
  const Eigen::Isometry3d random_pose = RandomIsometry3d();
  const double scale = RandomPositiveDouble();
  Eigen::Affine3d random_affine3d = Eigen::Affine3d::Identity();
  random_affine3d.translation() = random_pose.translation();
  random_affine3d.linear() = scale * random_pose.linear();
  return random_affine3d;
}

Eigen::Matrix3d RandomIntrinsics() {
  Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
  const double f_x = RandomGaussianDouble(500, 20);
  // Ensure that focal lengths are quite similar
  const double f_y = RandomGaussianDouble(f_x, 5);
  const double p_x = RandomGaussianDouble(640, 50);
  const double p_y = RandomGaussianDouble(480, 50);

  intrinsics(0, 0) = f_x;
  intrinsics(1, 1) = f_y;
  intrinsics(0, 2) = p_x;
  intrinsics(1, 2) = p_y;
  return intrinsics;
}

Eigen::Isometry3d RandomIdentityCenteredIsometry3d(const double translation_stddev, const double rotation_stddev) {
  return AddNoiseToIsometry3d(Eigen::Isometry3d::Identity(), translation_stddev, rotation_stddev);
}

Eigen::Isometry3d AddNoiseToIsometry3d(const Eigen::Isometry3d& pose, const double translation_stddev,
                                       const double rotation_stddev) {
  const double mean = 0.0;
  std::random_device dev;
  std::mt19937 rng(dev());
  std::normal_distribution<double> translation_distribution(mean, translation_stddev);
  std::normal_distribution<double> rotation_distribution(mean, rotation_stddev);

  const Eigen::Vector3d translation_noise(translation_distribution(rng), translation_distribution(rng),
                                          translation_distribution(rng));
  const Eigen::Matrix3d rotation_noise(
    RotationFromEulerAngles(rotation_distribution(rng), rotation_distribution(rng), rotation_distribution(rng)));

  Eigen::Isometry3d pose_noise = Isometry3d(translation_noise, rotation_noise);
  return pose * pose_noise;
}

gtsam::Pose3 AddNoiseToPose(const gtsam::Pose3& pose, const double translation_stddev, const double rotation_stddev) {
  return GtPose(AddNoiseToIsometry3d(EigenPose(pose), translation_stddev, rotation_stddev));
}

Eigen::Isometry3d RandomFrontFacingPose() {
  static constexpr double rho_min = 1.0;
  static constexpr double rho_max = 3.0;
  static constexpr double phi_min = -25.0;
  static constexpr double phi_max = 25.0;
  static constexpr double z_rho_scale = 0.5;

  // Pitch acts like yaw since z axis points outwards in camera frame
  static constexpr double yaw_min = -10.0;
  static constexpr double yaw_max = 10.0;
  static constexpr double pitch_min = -45;
  static constexpr double pitch_max = 45;
  static constexpr double roll_min = -10;
  static constexpr double roll_max = 10;

  return RandomFrontFacingPose(rho_min, rho_max, phi_min, phi_max, z_rho_scale, yaw_min, yaw_max, pitch_min, pitch_max,
                               roll_min, roll_max);
}

Eigen::Isometry3d RandomFrontFacingPose(const double rho_min, const double rho_max, const double phi_min,
                                        const double phi_max, const double z_rho_scale, const double yaw_min,
                                        const double yaw_max, const double pitch_min, const double pitch_max,
                                        const double roll_min, const double roll_max) {
  const Eigen::Vector3d translation = RandomFrontFacingPoint(rho_min, rho_max, phi_min, phi_max, z_rho_scale);
  const double yaw = RandomDouble(yaw_min, yaw_max);
  const double pitch = RandomDouble(pitch_min, pitch_max);
  const double roll = RandomDouble(roll_min, roll_max);
  const Eigen::Matrix3d rotation = RotationFromEulerAngles(yaw, pitch, roll);
  return Isometry3d(translation, rotation);
}

std::vector<Eigen::Vector3d> RandomFrontFacingPoints(const int num_points) {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < num_points; ++i) {
    points.emplace_back(RandomFrontFacingPoint());
  }
  return points;
}

Eigen::Vector3d RandomFrontFacingPoint() {
  static constexpr double rho_min = 1.0;
  static constexpr double rho_max = 3.0;
  static constexpr double phi_min = -25.0;
  static constexpr double phi_max = 25.0;
  static constexpr double z_rho_scale = 0.5;
  return RandomFrontFacingPoint(rho_min, rho_max, phi_min, phi_max, z_rho_scale);
}

Eigen::Vector3d RandomFrontFacingPoint(const double rho_min, const double rho_max, const double phi_min,
                                       const double phi_max, const double z_rho_scale) {
  const double rho = RandomDouble(rho_min, rho_max);
  const double phi = RandomDouble(phi_min, phi_max);
  const double z = RandomDouble(-1.0 * z_rho_scale * rho, z_rho_scale * rho);
  // Z and x are swapped so z defines distance from camera rather than height
  const Eigen::Vector3d tmp = CylindricalToCartesian(Eigen::Vector3d(rho, phi, z));
  const Eigen::Vector3d random_point(tmp.z(), tmp.y(), tmp.x());
  return random_point;
}

bool MatrixEquality(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs, const double tolerance) {
  // Seperately check for zero matrices since isApprox fails for these
  if (lhs.isZero(tolerance) || rhs.isZero(tolerance)) {
    return lhs.isZero(tolerance) && rhs.isZero(tolerance);
  }
  return lhs.isApprox(rhs, tolerance);
}
}  // namespace localization_common
