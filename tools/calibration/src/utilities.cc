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

#include <camera/camera_model.h>
#include <calibration/utilities.h>
#include <calibration/camera_utilities.h>

namespace calibration {
namespace lc = localization_common;

boost::optional<Eigen::Isometry3d> CameraTTarget(const camera::CameraParameters& camera,
                                                 const lc::ImageCorrespondences& matches, const int min_num_inliers) {
  Eigen::Isometry3d camera_T_target(Eigen::Isometry3d::Identity());
  constexpr int num_ransac_iterations = 100;
  constexpr int ransac_inlier_tolerance = 3;
  camera::CameraModel cam_model(camera_T_target, camera);
  if (!RansacEstimateCameraWithDistortion(matches.points_3d, matches.image_points, num_ransac_iterations,
                                          ransac_inlier_tolerance, min_num_inliers, &cam_model))
    return boost::none;
  return Eigen::Isometry3d(cam_model.GetTransform().matrix());
}
}  // namespace calibration
