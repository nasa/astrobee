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

#ifndef LOCALIZATION_COMMON_POSE_WITH_COVARIANCE_H_
#define LOCALIZATION_COMMON_POSE_WITH_COVARIANCE_H_

#include <Eigen/Geometry>

namespace localization_common {
struct PoseWithCovariance {
  PoseWithCovariance(const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 6>& covariance)
      : pose(pose), covariance(covariance) {}
  Eigen::Isometry3d pose;
  Eigen::Matrix<double, 6, 6> covariance;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_POSE_WITH_COVARIANCE_H_
