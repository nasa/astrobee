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

#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <point_cloud_common/utilities.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace pc = point_cloud_common;
namespace sym = gtsam::symbol_shorthand;

double PointToPlaneError(const gtsam::Point3& point_1, const gtsam::Point3& point_2, const gtsam::Vector3& normal_2,
                         const gtsam::Pose3& relative_transform) {
  return (relative_transform * point_1 - point_2).dot(normal_2);
}

TEST(UtilitiesTester, PointToPlaneJacobian) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 point_1 = lc::RandomVector();
    const gtsam::Point3 point_2 = lc::RandomVector();
    const gtsam::Vector3 normal_2 = lc::RandomVector();
    const gtsam::Pose3 relative_transform = lc::RandomPose();
    const gtsam::Matrix H = pc::PointToPlaneJacobian(point_1, normal_2, relative_transform);
    const auto numerical_H = gtsam::numericalDerivative11<double, gtsam::Pose3>(
      boost::function<double(const gtsam::Pose3&)>(boost::bind(&PointToPlaneError, point_1, point_2, normal_2, _1)),
      relative_transform, 1e-5);
    ASSERT_TRUE(numerical_H.isApprox(H.matrix(), 1e-6));
  }
}

TEST(UtilitiesTester, A) {}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
