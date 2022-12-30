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

#ifndef GRAPH_VIO_SANITY_CHECKER_H_
#define GRAPH_VIO_SANITY_CHECKER_H_

#include <graph_vio/sanity_checker_params.h>
#include <localization_common/combined_nav_state_covariances.h>

#include <gtsam/geometry/Pose3.h>

namespace graph_vio {
class SanityChecker {
 public:
  explicit SanityChecker(const SanityCheckerParams& params);
  bool CheckPoseSanity(const gtsam::Pose3& sparse_mapping_pose, const gtsam::Pose3& localizer_pose);
  bool CheckCovarianceSanity(const localization_common::CombinedNavStateCovariances& covariances) const;
  void Reset();

 private:
  SanityCheckerParams params_;
  int num_consecutive_failures_;
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_SANITY_CHECKER_H_
