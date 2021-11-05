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
#include <optimization_common/utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace oc = optimization_common;
TEST(UtilitiesTester, Isometry3dToVectorToIsometry3d) {
  for (int i = 0; i < 500; ++i) {
    const Eigen::Isometry3d pose = lc::RandomIsometry3d();
    const Eigen::Matrix<double, 6, 1> pose_vector = oc::VectorFromIsometry3d(pose);
    const Eigen::Isometry3d pose_again = oc::Isometry3d(pose_vector);
    ASSERT_TRUE(pose_again.matrix().isApprox(pose.matrix(), 1e-6));
  }
}

TEST(UtilitiesTester, VectorToAffine3dToVector) {
  for (int i = 0; i < 500; ++i) {
    const Eigen::Affine3d affine_3d = lc::RandomAffine3d();
    const Eigen::Matrix<double, 7, 1> affine_3d_vector = oc::VectorFromAffine3d(affine_3d);
    const Eigen::Affine3d affine_3d_again = oc::Affine3d(affine_3d_vector);
    ASSERT_TRUE(affine_3d_again.matrix().isApprox(affine_3d.matrix(), 1e-6));
  }
}
