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

#include <graph_localizer/sanity_checker.h>

namespace graph_localizer {
SanityChecker::SanityChecker(const SanityCheckerParams& params) : params_(params), num_consecutive_failures_(0) {}

bool SanityChecker::CheckSanity(const gtsam::Pose3& sparse_mapping_pose, const gtsam::Pose3& localizer_pose) {
  const gtsam::Vector3 difference_vector = sparse_mapping_pose.translation() - localizer_pose.translation();
  const double euclidean_distance = difference_vector.norm();
  if (euclidean_distance > params_.max_sane_position_difference)
    ++num_consecutive_failures_;
  else
    num_consecutive_failures_ = 0;
  return (num_consecutive_failures_ < params_.num_consecutive_failures_until_insane);
}
}  // namespace graph_localizer
