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

Eigen::Vector3d RandomVector() {
  // Eigen::Vector3 is constrained to [-1, 1]
  return RandomDouble() * Eigen::Vector3d::Random();
}

gtsam::Pose3 RandomPose() {
  std::random_device dev;
  std::mt19937 rng(dev());
  gtsam::Rot3 rot = gtsam::Rot3::Random(rng);
  gtsam::Point3 trans = RandomVector();
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
  const double f_x = RandomGaussianDouble(500, 100);
  // Ensure that focal lengths are quite similar
  const double f_y = RandomGaussianDouble(f_x, 5);
  const double p_x = RandomGaussianDouble(500, 100);
  const double p_y = RandomGaussianDouble(500, 100);

  intrinsics(0, 0) = f_x;
  intrinsics(1, 1) = f_y;
  intrinsics(0, 2) = p_x;
  intrinsics(1, 2) = p_y;
  return intrinsics;
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
}  // namespace localization_common
