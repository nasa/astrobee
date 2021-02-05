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
#ifndef GROUND_TRUTH_LOCALIZER_TWIST_H_
#define GROUND_TRUTH_LOCALIZER_TWIST_H_

#include <Eigen/Core>

namespace ground_truth_localizer {
struct Twist {
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;
};
}  // namespace ground_truth_localizer

#endif  // GROUND_TRUTH_LOCALIZER_TWIST_H_
