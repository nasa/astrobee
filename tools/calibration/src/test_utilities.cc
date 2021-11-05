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

#include <calibration/camera_utilities.h>
#include <calibration/test_utilities.h>
#include <localization_common/test_utilities.h>

namespace calibration {
namespace lc = localization_common;

RandomRegistrationCorrespondences::RandomRegistrationCorrespondences() {
  static constexpr double x_min = 0.1;
  static constexpr double x_max = 30;
  static constexpr double y_min = -10.0;
  static constexpr double y_max = 10.0;
  static constexpr double z_min = -10.0;
  static constexpr double z_max = 10.0;

  // Sample points in front of camera
  const double x = lc::RandomDouble(x_min, x_max);
  const double y = lc::RandomDouble(y_min, y_max);
  const double z = lc::RandomDouble(z_min, z_max);
  // TODO(rsoussan): Sample orientation such that it is facing camera to some extent
  camera_T_target_ = lc::RandomIsometry3d();
  camera_T_target_.translation() = Eigen::Vector3d(x, y, z);
  intrinsics_ = lc::RandomIntrinsics();
  const std::vector<Eigen::Vector3d> target_t_target_points = TargetPoints();
  for (const auto& target_t_target_point : target_t_target_points) {
    const Eigen::Vector3d camera_t_target_point = camera_T_target_ * target_t_target_point;
    if (camera_t_target_point.z() <= 0) continue;
    const Eigen::Vector2d image_point = Project3dPointToImageSpace(camera_t_target_point, intrinsics_);
    correspondences_.AddCorrespondence(image_point, target_t_target_point);
  }
}

std::vector<Eigen::Vector3d> RandomRegistrationCorrespondences::TargetPoints() {
  static constexpr double kRowSpacing = 0.1;
  static constexpr double kColSpacing = 0.1;
  static constexpr int kNumPointsPerRow = 10;
  static constexpr int kNumPointsPerCol = 10;

  std::vector<Eigen::Vector3d> target_points;
  for (int i = 0; i < kNumPointsPerCol; ++i) {
    for (int j = 0; j < kNumPointsPerRow; ++j) {
      target_points.emplace_back(Eigen::Vector3d(i * kColSpacing, j * kRowSpacing, 0));
    }
  }
  return target_points;
}
}  // namespace calibration
