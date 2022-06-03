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

#include <localization_common/test_utilities.h>
#include <optimization_common/se3_local_parameterization.h>
#include <optimization_common/utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace oc = optimization_common;
TEST(SE3LocalParameterizationTester, SE3Plus) {
  oc::SE3Plus se3_plus;
  for (int i = 0; i < 500; ++i) {
    const Eigen::Isometry3d pose = lc::RandomIsometry3d();
    const Eigen::Isometry3d delta = lc::RandomIsometry3d();
    const Eigen::Isometry3d updated_pose = pose * delta;
    const Eigen::Matrix<double, 6, 1> pose_vector = oc::VectorFromIsometry3d(pose);
    const Eigen::Matrix<double, 6, 1> delta_vector = oc::VectorFromIsometry3d(delta);
    Eigen::Matrix<double, 6, 1> updated_pose_vector;
    se3_plus(pose_vector.data(), delta_vector.data(), updated_pose_vector.data());
    const Eigen::Isometry3d updated_pose_again = oc::Isometry3d(updated_pose_vector);
    EXPECT_MATRIX_NEAR(updated_pose, updated_pose_again, 1e-6);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
