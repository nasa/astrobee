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

TEST(InverseDepthMeasurementTester, Backproject) {
  for (int i = 0; i < 50; ++i) {
    const gtsam::Point3 cam_t_measurement = lc::RandomFrontFacingPoint();
    const gtsam::Pose3 body_T_cam = lc::RandomPose();
    const Eigen::Matrix3d intrinsics = lc::RandomIntrinsics();
    const auto source_measurement = vc::Project(cam_t_measurement, intrinsics);
    vc::InverseDepthMeasurement inverse_depth_measurement(1.0 / cam_t_measurement.z(), source_measurement, intrinsics,
                                                          body_T_cam);
    const auto backprojected_measurement = inverse_depth_measurement.Backproject();
    EXPECT_TRUE(lc::MatrixEquality<6>(backprojected_measurement.matrix(), cam_t_measurement.matrix()));
  }
}

TEST(InverseDepthMeasurementTester, Project) {
  constexpr double translation_stddev = 0.05;
  constexpr double rotation_stddev = 1;
  for (int i = 0; i < 50; ++i) {
    const gtsam::Point3 source_cam_t_measurement = lc::RandomFrontFacingPoint();
    const gtsam::Pose3 body_T_cam = lc::RandomPose();
    const Eigen::Matrix3d intrinsics = lc::RandomIntrinsics();
    const auto source_measurement = vc::Project(source_cam_t_measurement, intrinsics);
    vc::InverseDepthMeasurement inverse_depth_measurement(1.0 / source_cam_t_measurement.z(), source_measurement,
                                                          intrinsics, body_T_cam);
    // Make sure point is still likely to project into target camera frame
    const gtsam::Pose3 source_cam_T_target_cam =
      lc::GtPose(lc::RandomIdentityCenteredIsometry3d(translation_stddev, rotation_stddev));
    const gtsam::Pose3 world_T_source_body = lc::RandomPose();
    const gtsam::Pose3 world_T_target_body =
      world_T_source_body * body_T_cam * source_cam_T_target_cam * body_T_cam.inverse();
    const auto projected_target_measurement =
      inverse_depth_measurement.Project(world_T_source_body, world_T_target_body);
    // Shouldn't occur very often it at all since there is small difference between the source and target cam frames
    if (!projected_target_measurement) continue;
    const gtsam::Point3 target_cam_t_measurement = source_cam_T_target_cam.inverse() * source_cam_t_measurement;
    const auto target_measurement = vc::Project(target_cam_t_measurement, intrinsics);
    EXPECT_TRUE(lc::MatrixEquality<6>(target_measurement.matrix(), projected_target_measurement->matrix()));
  }
}

TEST(InverseDepthMeasurementTester, InvalidProject) {
  const gtsam::Point3 source_cam_t_measurement = lc::RandomFrontFacingPoint();
  const gtsam::Pose3 body_T_cam = lc::RandomPose();
  const Eigen::Matrix3d intrinsics = lc::RandomIntrinsics();
  const auto source_measurement = vc::Project(source_cam_t_measurement, intrinsics);
  vc::InverseDepthMeasurement inverse_depth_measurement(1.0 / source_cam_t_measurement.z(), source_measurement,
                                                        intrinsics, body_T_cam);
  // Move target camera frame in front of measurement point, ensures that point is behind target frame and projection
  // should fail
  const gtsam::Pose3 source_cam_T_target_cam =
    lc::GtPose(lc::Isometry3d(Eigen::Vector3d(0, 0, source_cam_t_measurement.z() + 1), Eigen::Matrix3d::Identity()));
  const gtsam::Pose3 world_T_source_body = lc::RandomPose();
  const gtsam::Pose3 world_T_target_body =
    world_T_source_body * body_T_cam * source_cam_T_target_cam * body_T_cam.inverse();
  const auto projected_target_measurement = inverse_depth_measurement.Project(world_T_source_body, world_T_target_body);
  EXPECT_TRUE(projected_target_measurement == boost::none);
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
