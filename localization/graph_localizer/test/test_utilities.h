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
#include <localization_measurements/plane.h>

#include <gtsam/geometry/Pose3.h>

#include <random>

namespace graph_localizer {
double RandomDouble() {
  std::random_device dev;
  std::mt19937 rng(dev());
  return std::uniform_real_distribution<double>(-100, 100)(rng);
}

gtsam::Vector3 RandomVector() {
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

localization_measurements::Plane RandomPlane() {
  gtsam::Point3 point = RandomVector();
  gtsam::Vector3 normal = RandomVector().normalized();
  return localization_measurements::Plane(point, normal);
}
}  // namespace graph_localizer
