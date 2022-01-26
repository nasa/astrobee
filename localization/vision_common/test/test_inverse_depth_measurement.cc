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
#include <vision_common/inverse_depth_measurement.h>
#include <vision_common/test_utilities.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace vc = vision_common;

TEST(InverseDepthMeasurementTester, Project) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 world_t_measurement = lc::RandomPoint3d();
    const gtsam::Pose3 world_T_source_body = lc::RandomPose();
    const gtsam::Pose3 world_T_target_body = lc::RandomPose();
    const gtsam::Pose3 body_T_camera = lc::RandomPose();
    const gtsam::Pose3 world_T_source = world_T_source_body * body_T_camera;
    const gtsam::Pose3 world_T_target = world_T_target_body * body_T_camera;
    const Eigen::Matrix3d intrinsics = lc::RandomIntrinsics();
    const gtsam::Point3 source_t_measurement = world_T_source.inverse() * world_t_measurement;
    const auto source_measurement = vc::Project(source_t_measurement, intrinsics);
    vc::InverseDepthMeasurement inverse_depth_measurement(source_t_measurement.z(), source_measurement, intrinsics,
                                                          body_T_camera);
    const auto projected_target_measurement =
      inverse_depth_measurement.Project(world_T_source_body, world_T_target_body);
    const gtsam::Point3 target_t_measurement = world_T_target.inverse() * world_t_measurement;
    const auto target_measurement = vc::Project(target_t_measurement, intrinsics);
    EXPECT_TRUE(lc::MatrixEquality<6>(target_measurement.matrix(), projected_target_measurement->matrix()));
  }
}
/*TEST(InverseDepthMeasurementTester, ProjectJacobian) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 cam_t_point = lc::RandomPoint3d();
    const Eigen::Matrix3d intrinsics = lc::RandomIntrinsics();
    const auto noise = gtsam::noiseModel::Unit::Create(2);
    gtsam::Matrix H;
    const auto projection = vc::Project(cam_t_point, intrinsics, H);
    const auto numerical_H = gtsam::numericalDerivative11<Eigen::Vector2d, Eigen::Vector3d>(
      boost::function<Eigen::Vector2d(const Eigen::Vector3d&)>(boost::bind(&vc::Project, _1, intrinsics, boost::none)),
      cam_t_point, 1e-5);
    EXPECT_TRUE(lc::MatrixEquality<6>(numerical_H.matrix(), H.matrix()));
  }
}*/

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
