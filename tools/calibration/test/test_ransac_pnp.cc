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
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <optimization_common/identity_distorter.h>
#include <optimization_common/utilities.h>

#include <opencv2/calib3d/calib3d.hpp>

#include <gtest/gtest.h>

namespace ca = calibration;
namespace lc = localization_common;
namespace oc = optimization_common;

// TODO(rsoussan): Put back RansacPnP tests once pnp issues are resolved
/*TEST(RansacPnPTester, EvenlySpacedTargetsIdentityDistortionWithNoise) {
  const auto params = ca::DefaultRansacPnPParams();
  const int num_rows = 5;
  const int num_cols = 5;
  const int num_y_levels = 5;
  const auto target_poses = ca::EvenlySpacedTargetPoses(num_rows, num_cols, num_y_levels);
  const auto target_points = ca::TargetPoints(10, 10);
  std::vector<int> initial_inliers(target_points.size());
  // Fill inliers with all indices
  std::iota(initial_inliers.begin(), initial_inliers.end(), 0);
  for (int i = 0; i < target_poses.size(); ++i) {
    const auto correspondences =
      ca::RegistrationCorrespondences<oc::IdentityDistorter>(target_poses[i], lc::RandomIntrinsics(), target_points);
    const auto pose_estimate = ca::RansacPnP<oc::IdentityDistorter>(
      correspondences.correspondences().image_points, correspondences.correspondences().points_3d,
      correspondences.intrinsics(), Eigen::VectorXd(), params);
    ASSERT_TRUE(pose_estimate != boost::none);
    ASSERT_PRED2(lc::MatrixEquality<2>, pose_estimate->pose.matrix(), correspondences.camera_T_target().matrix());
    ASSERT_TRUE(pose_estimate->inliers.size() == correspondences.correspondences().size());
  }
}*/
