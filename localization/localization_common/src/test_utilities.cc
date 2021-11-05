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
double RandomDouble() {
  std::random_device dev;
  std::mt19937 rng(dev());
  return std::uniform_real_distribution<double>(-100, 100)(rng);
}

double RandomPositiveDouble() {
  std::random_device dev;
  std::mt19937 rng(dev());
  return std::uniform_real_distribution<double>(0, 100)(rng);
}

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
  const double scale = RandomDouble();
  Eigen::Affine3d random_affine3d = Eigen::Affine3d::Identity();
  random_affine3d.translation() = random_pose.translation();
  const Eigen::Matrix3d scale_matrix(Eigen::Matrix3d::Identity() * scale);
  random_affine3d.linear() = scale_matrix * random_pose.rotation();
  return random_affine3d;
}
}  // namespace localization_common
