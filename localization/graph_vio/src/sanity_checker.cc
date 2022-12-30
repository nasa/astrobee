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

#include <graph_vio/sanity_checker.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <localization_common/utilities.h>

namespace graph_vio {
namespace lc = localization_common;
SanityChecker::SanityChecker(const SanityCheckerParams& params) : params_(params), num_consecutive_failures_(0) {}

bool SanityChecker::CheckPoseSanity(const gtsam::Pose3& sparse_mapping_pose, const gtsam::Pose3& localizer_pose) {
  if (!params_.check_pose_difference) return true;

  const gtsam::Vector3 difference_vector = sparse_mapping_pose.translation() - localizer_pose.translation();
  const double euclidean_distance = difference_vector.norm();
  if (euclidean_distance > params_.max_sane_position_difference)
    ++num_consecutive_failures_;
  else
    num_consecutive_failures_ = 0;
  return (num_consecutive_failures_ < params_.num_consecutive_pose_difference_failures_until_insane);
}

bool SanityChecker::CheckCovarianceSanity(const lc::CombinedNavStateCovariances& covariances) const {
  return lc::PoseCovarianceSane(covariances.pose_covariance(), params_.position_covariance_threshold,
                                params_.orientation_covariance_threshold, params_.check_position_covariance,
                                params_.check_orientation_covariance);
}

void SanityChecker::Reset() { num_consecutive_failures_ = 0; }

}  // namespace graph_vio
