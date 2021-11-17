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
#ifndef CALIBRATION_MATCH_SET_H_
#define CALIBRATION_MATCH_SET_H_

#include <localization_common/image_correspondences.h>

#include <Eigen/Geometry>

#include <vector>

namespace calibration {
struct MatchSet {
  MatchSet(const localization_common::ImageCorrespondences& correspondences, const Eigen::Isometry3d& pose_estimate,
           const std::vector<int>& inliers)
      : correspondences(correspondences), pose_estimate(pose_estimate), inliers(inliers) {}
  localization_common::ImageCorrespondences correspondences;
  Eigen::Isometry3d pose_estimate;
  std::vector<int> inliers;
};
}  // namespace calibration

#endif  // CALIBRATION_MATCH_SET_H_
