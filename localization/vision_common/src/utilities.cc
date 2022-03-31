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
#include <vision_common/utilities.h>

namespace vision_common {
namespace lc = localization_common;

Eigen::Vector3d Backproject(const Eigen::Vector2d& measurement, const Eigen::Matrix3d& intrinsics, const double depth,
                            gtsam::OptionalJacobian<3, 1> d_backprojected_point_d_depth) {
  if (d_backprojected_point_d_depth) {
    *d_backprojected_point_d_depth << (measurement.x() - intrinsics(0, 2)) / intrinsics(0, 0),
      (measurement.y() - intrinsics(1, 2)) / intrinsics(1, 1), 1;
  }
  Eigen::Vector3d point_3d(0, 0, depth);
  point_3d.head<2>() = depth * RelativeCoordinates(measurement, intrinsics);
  return point_3d;
}

Eigen::Vector2d FocalLengths(const Eigen::Matrix3d& intrinsics) {
  return Eigen::Vector2d(intrinsics(0, 0), intrinsics(1, 1));
}

Eigen::Vector2d PrincipalPoints(const Eigen::Matrix3d& intrinsics) {
  return Eigen::Vector2d(intrinsics(0, 2), intrinsics(1, 2));
}

Eigen::Vector2d Project(const Eigen::Vector3d& cam_t_point, const Eigen::Matrix3d& intrinsics,
                        gtsam::OptionalJacobian<2, 3> d_projected_point_d_cam_t_point) {
  if (d_projected_point_d_cam_t_point) {
    const Eigen::Vector3d p = intrinsics * cam_t_point;
    const double pz_2 = cam_t_point.z() * cam_t_point.z();
    const double z_inv = 1.0 / cam_t_point.z();
    const double z_inv_2 = z_inv * z_inv;
    *d_projected_point_d_cam_t_point << intrinsics(0, 0) * z_inv, intrinsics(0, 1) * z_inv,
      -intrinsics(0, 0) * cam_t_point.x() * z_inv_2 - intrinsics(0, 1) * cam_t_point.y() * z_inv_2, 0,
      intrinsics(1, 1) * z_inv, -intrinsics(1, 1) * cam_t_point.y() * z_inv_2;
  }
  return (intrinsics * cam_t_point).hnormalized();
}

Eigen::Isometry3d Isometry3d(const cv::Mat& rodrigues_rotation_cv, const cv::Mat& translation_cv) {
  Eigen::Vector3d translation;
  cv::cv2eigen(translation_cv, translation);
  Eigen::Matrix3d rotation;
  cv::Mat rotation_cv;
  cv::Rodrigues(rodrigues_rotation_cv, rotation_cv);
  cv::cv2eigen(rotation_cv, rotation);
  return lc::Isometry3d(translation, rotation);
}
}  // namespace vision_common
