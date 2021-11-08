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
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().z() = 3;
  Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
  intrinsics(0, 0) = 500;
  intrinsics(1, 1) = 500;
  intrinsics(0, 2) = 500;
  intrinsics(1, 2) = 500;
  for (int i = 0; i < 500; ++i) {
    const auto correspondences = ca::RegistrationCorrespondences(pose, intrinsics);
    std::vector<cv::Point2d> image_points_cv;
    int count = 0;
    for (const auto& image_point : correspondences.correspondences().image_points) {
      if (count++ >= 4) break;
      image_points_cv.emplace_back(cv::Point2d(image_point.x(), image_point.y()));
    }

    count = 0;
    std::vector<cv::Point3d> points_3d_cv;
    for (const auto& point_3d : correspondences.correspondences().points_3d) {
      if (count++ >= 4) break;
      points_3d_cv.emplace_back(point_3d.x(), point_3d.y(), point_3d.z());
    }

    cv::Mat intrinsics_cv;
    cv::eigen2cv(correspondences.intrinsics(), intrinsics_cv);
    cv::Mat rodrigues_rotation_cv(3, 1, cv::DataType<double>::type, cv::Scalar(0));
    cv::Mat translation_cv(3, 1, cv::DataType<double>::type, cv::Scalar(0));
    ca::UndistortedPnP(image_points_cv, points_3d_cv, intrinsics_cv, cv::SOLVEPNP_P3P, rodrigues_rotation_cv,
                       translation_cv);
    const Eigen::Isometry3d pose_estimate = ca::Isometry3d(rodrigues_rotation_cv, translation_cv);
    LogError("true pose: " << std::endl << correspondences.camera_T_target().matrix());
    LogError("pnp pose: " << std::endl << pose_estimate.matrix());
    // TODO(rsoussan): Decrease tolerance once cv::solvePnP issues are resolved
    constexpr double tolerance = 1e-1;
    ASSERT_TRUE(pose_estimate.matrix().isApprox(correspondences.camera_T_target().matrix(), tolerance));
  }
}
