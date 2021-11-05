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

#include <calibration/test_utilities.h>
#include <localization_common/test_utilities.h>

namespace calibration {
namespace lc = localization_common;

RegistrationCorrespondences::RegistrationCorrespondences() {
  pose_ = lc::RandomIsometry3d();
  // generate random pose in front of camera!(how to do this???)
  // add function to sample double within user defined bounds!!! (A)
  // sample position in some bounds in front of camera (min 0.1 or someting, always positive z) (B)
  // sample randon orientation
  // throw out any points that don't project onto image, add todo to sample orientation better to prevent this
  // make sure camera is oriented correctly!!!!
  // sample 3d points (on target grid? how to define grid size?), save them
  // project w/o distortion into image space, save image pts
  // add identity distorter, use this!
}

std::vector<Eigen::Vector3d> RegistrationCorrespondences::TargetPoints() {
  constexpr double kRowSpacing = 0.1;
  constexpr double kColSpacing = 0.1;
  constexpr int kNumPointsPerRow = 10;
  constexpr int kNumPointsPerCol = 10;

  std::vector<Eigen::Vector3d> target_points;
  for (int i = 0; i < kNumPointsPerCol; ++i) {
    for (int j = 0; j < kNumPointsPerRow; ++j) {
      target_points.emplace_back(Eigen::Vector3d(i * kColSpacing, j * kRowSpacing, 0));
    }
  }
  return target_points;
}
}  // namespace calibration
