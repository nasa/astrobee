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
TEST(RansacPnPTester, RansacPnP) {
  auto params = ca::DefaultRansacPnPParams();
  params.pnp_method = cv::SOLVEPNP_ITERATIVE;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().z() = 3;
  Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
  intrinsics(0, 0) = 500;
  intrinsics(1, 1) = 500;
  intrinsics(0, 2) = 500;
  intrinsics(1, 2) = 500;
  for (int i = 0; i < 500; ++i) {
    const auto correspondences = ca::RegistrationCorrespondences(pose, intrinsics, ca::RandomFrontFacingPoints(100));
    const auto pose_estimate = ca::RansacPnP<oc::IdentityDistorter>(
      correspondences.correspondences().image_points, correspondences.correspondences().points_3d,
      correspondences.intrinsics(), Eigen::VectorXd(), params);
    ASSERT_TRUE(pose_estimate != boost::none);
    ASSERT_TRUE(pose_estimate->second.size() == correspondences.correspondences().image_points.size());
    LogError("true pose: " << std::endl << correspondences.camera_T_target().matrix());
    LogError("pnp pose: " << std::endl << pose_estimate->first.matrix());
    // TODO(rsoussan): Decrease tolerance once cv::solvePnP issues are resolved
    constexpr double tolerance = 1e-2;
    ASSERT_TRUE(pose_estimate->first.matrix().isApprox(correspondences.camera_T_target().matrix(), tolerance));
  }
}

/*TEST(RansacPnPTester, Inliers) {
const int num_inliers = Inliers<optimization_common::Identity>(const std::vector<Eigen::Vector2d>& image_points, const
std::vector<Eigen::Vector3d>& points_3d, const Eigen::Matrix3d& intrinsics, const Eigen::VectorXd& distortion, const
Eigen::Isometry3d& pose_estimate, const double max_inlier_threshold, boost::optional<std::vector<int>&> inliers =
boost::none);


}*/
