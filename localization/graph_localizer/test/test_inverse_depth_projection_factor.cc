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

#include <graph_localizer/inverse_depth_projection_factor.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace vc = vision_common;
/*Eigen::Vector2d ProjectPoseJacobianHelper(const vc::InverseDepthMeasurement& inverse_depth_measurement,
                                          const gtsam::Pose3& world_T_source_body,
                                          const gtsam::Pose3& world_T_target_body) {
  const auto projected_target_measurement = inverse_depth_measurement.Project(world_T_source_body, world_T_target_body);
  // Assumes no cheirality errors occur
  return *projected_target_measurement;
}

Eigen::Vector2d ProjectInverseDepthJacobianHelper(double inverse_depth, const Eigen::Vector2d& source_measurement,
                                                  const gtsam::Pose3& body_T_cam,
                                                  const gtsam::Pose3& world_T_source_body,
                                                  const gtsam::Pose3& world_T_target_body,
                                                  const Eigen::Matrix3d& intrinsics) {
  const vc::InverseDepthMeasurement inverse_depth_measurement(inverse_depth, source_measurement, intrinsics,
                                                              body_T_cam);
  const auto projected_target_measurement = inverse_depth_measurement.Project(world_T_source_body, world_T_target_body);
  return ProjectPoseJacobianHelper(inverse_depth_measurement, world_T_source_body, world_T_target_body);
}*/

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
    const auto new_target_measurement = lc::RandomPoint2d();
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1, 1));
    const gtsam::InverseDepthProjectionFactor factor(new_target_measurement, noise, 1, 2, 3);
    const auto error = factor.evaluateError(world_T_source_body, inverse_depth_measurement, world_T_target_body);
    const Eigen::Vector2d expected_error = target_measurement - new_target_measurement;
    EXPECT_MATRIX_TYPE_NEAR<6>(error, expected_error);
  }
}

/*TEST(InverseDepthMeasurementTester, InvalidProject) {
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

TEST(InverseDepthMeasurementTester, ProjectInverseDepthJacobian) {
  constexpr double translation_stddev = 0.05;
  constexpr double rotation_stddev = 1;
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 source_cam_t_measurement = lc::RandomFrontFacingPoint();
    const gtsam::Pose3 body_T_cam = lc::RandomPose();
    const Eigen::Matrix3d intrinsics = lc::RandomIntrinsics();
    const auto source_measurement = vc::Project(source_cam_t_measurement, intrinsics);
    const double inverse_depth = 1.0 / source_cam_t_measurement.z();
    vc::InverseDepthMeasurement inverse_depth_measurement(inverse_depth, source_measurement, intrinsics, body_T_cam);
    // Make sure point is still likely to project into target camera frame
    const gtsam::Pose3 source_cam_T_target_cam =
      lc::GtPose(lc::RandomIdentityCenteredIsometry3d(translation_stddev, rotation_stddev));
    const gtsam::Pose3 world_T_source_body = lc::RandomPose();
    const gtsam::Pose3 world_T_target_body =
      world_T_source_body * body_T_cam * source_cam_T_target_cam * body_T_cam.inverse();
    gtsam::Matrix d_projected_point_d_inverse_depth;
    gtsam::Matrix d_projected_point_d_world_T_source_body;
    gtsam::Matrix d_projected_point_d_world_T_target_body;
    const auto projected_target_measurement = inverse_depth_measurement.Project(
      world_T_source_body, world_T_target_body, d_projected_point_d_world_T_source_body,
      d_projected_point_d_world_T_target_body, d_projected_point_d_inverse_depth);
    // Shouldn't occur very often it at all since there is small difference between the source and target cam frames
    ASSERT_TRUE(projected_target_measurement != boost::none);
    // Inverse depth Jacobian
    {
      const auto numerical_d_projected_point_d_inverse_depth = gtsam::numericalDerivative11<Eigen::Vector2d, double>(
        boost::function<Eigen::Vector2d(const double)>(boost::bind(&ProjectInverseDepthJacobianHelper, _1,
                                                                   source_measurement, body_T_cam, world_T_source_body,
                                                                   world_T_target_body, intrinsics)),
        inverse_depth);
      EXPECT_MATRIX_TYPE_NEAR<6>(numerical_d_projected_point_d_inverse_depth, d_projected_point_d_inverse_depth);
    }
    // world_T_source_body Jacobian
    {
      const auto numerical_d_projected_point_d_world_T_source_body =
        gtsam::numericalDerivative11<Eigen::Vector2d, gtsam::Pose3>(
          boost::function<Eigen::Vector2d(const gtsam::Pose3&)>(
            boost::bind(&ProjectPoseJacobianHelper, inverse_depth_measurement, _1, world_T_target_body)),
          world_T_source_body);
      EXPECT_MATRIX_TYPE_NEAR<6>(numerical_d_projected_point_d_world_T_source_body,
                                 d_projected_point_d_world_T_source_body);
    }
    // world_T_target_body Jacobian
    {
      const auto numerical_d_projected_point_d_world_T_target_body =
        gtsam::numericalDerivative11<Eigen::Vector2d, gtsam::Pose3>(
          boost::function<Eigen::Vector2d(const gtsam::Pose3&)>(
            boost::bind(&ProjectPoseJacobianHelper, inverse_depth_measurement, world_T_source_body, _1)),
          world_T_target_body);
      EXPECT_MATRIX_TYPE_NEAR<6>(numerical_d_projected_point_d_world_T_target_body,
                                 d_projected_point_d_world_T_target_body);
    }
  }
}

TEST(InverseDepthMeasurementTester, ManifoldOperations) {
  for (int i = 0; i < 50; ++i) {
    const double starting_inverse_depth = lc::RandomDouble();
    vc::InverseDepthMeasurement inverse_depth_measurement(starting_inverse_depth, Eigen::Vector2d(), Eigen::Matrix3d(),
                                                          gtsam::Pose3());
    const double true_inverse_depth = lc::RandomDouble();
    const gtsam::Key key(1);
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(0.0001));
    gtsam::SimpleInverseDepthFactor factor(true_inverse_depth, noise, key);
    gtsam::NonlinearFactorGraph graph;
    graph.add(factor);
    gtsam::Values values;
    values.insert(key, inverse_depth_measurement);
    const auto result = gtsam::LevenbergMarquardtOptimizer(graph, values).optimize();
    EXPECT_NEAR(true_inverse_depth, (result.at<vc::InverseDepthMeasurement>(key)).inverse_depth(), 1e-6);
  }
}*/

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
