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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace vc = vision_common;

TEST(InverseDepthProjectionFactorTester, EvaluateError) {
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
    const gtsam::Point3 target_cam_t_measurement = source_cam_T_target_cam.inverse() * source_cam_t_measurement;
    const auto target_measurement = vc::Project(target_cam_t_measurement, intrinsics);
    const auto new_target_measurement = lc::RandomPoint2d();
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1, 1));
    const gtsam::InverseDepthProjectionFactor factor(new_target_measurement, noise, 1, 2, 3);
    const auto error = factor.evaluateError(world_T_source_body, inverse_depth_measurement, world_T_target_body);
    const Eigen::Vector2d expected_error = target_measurement - new_target_measurement;
    EXPECT_MATRIX_NEAR(error, expected_error, 1e-6);
  }
}

TEST(InverseDepthProjectionFactorTester, InvalidError) {
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
  const auto new_target_measurement = lc::RandomPoint2d();
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1, 1));
  const gtsam::InverseDepthProjectionFactor factor(new_target_measurement, noise, 1, 2, 3);
  gtsam::Matrix d_error_d_inverse_depth;
  gtsam::Matrix d_error_d_world_T_source_body;
  gtsam::Matrix d_error_d_world_T_target_body;
  const auto error =
    factor.evaluateError(world_T_source_body, inverse_depth_measurement, world_T_target_body,
                         d_error_d_world_T_source_body, d_error_d_inverse_depth, d_error_d_world_T_target_body);
  EXPECT_MATRIX_NEAR(error, Eigen::Vector2d::Zero(), 1e-6);
  EXPECT_MATRIX_NEAR(d_error_d_inverse_depth, (Eigen::Matrix<double, 2, 1>::Zero()), 1e-6);
  EXPECT_MATRIX_NEAR(d_error_d_world_T_source_body, (Eigen::Matrix<double, 2, 6>::Zero()), 1e-6);
  EXPECT_MATRIX_NEAR(d_error_d_world_T_target_body, (Eigen::Matrix<double, 2, 6>::Zero()), 1e-6);
}

TEST(InverseDepthProjectionFactorTester, Jacobians) {
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
    const auto new_target_measurement = lc::RandomPoint2d();
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1, 1));
    const gtsam::InverseDepthProjectionFactor factor(new_target_measurement, noise, 1, 2, 3);
    gtsam::Matrix d_error_d_inverse_depth;
    gtsam::Matrix d_error_d_world_T_source_body;
    gtsam::Matrix d_error_d_world_T_target_body;
    const auto error =
      factor.evaluateError(world_T_source_body, inverse_depth_measurement, world_T_target_body,
                           d_error_d_world_T_source_body, d_error_d_inverse_depth, d_error_d_world_T_target_body);
    const auto numerical_d_error_d_world_T_source_body =
      gtsam::numericalDerivative31<Eigen::Vector2d, gtsam::Pose3, vc::InverseDepthMeasurement, gtsam::Pose3>(
        boost::function<Eigen::Vector2d(const gtsam::Pose3&, const vc::InverseDepthMeasurement&, const gtsam::Pose3&)>(
          boost::bind(&gtsam::InverseDepthProjectionFactor::evaluateError, factor, _1, _2, _3, boost::none, boost::none,
                      boost::none)),
        world_T_source_body, inverse_depth_measurement, world_T_target_body);
    EXPECT_MATRIX_NEAR(numerical_d_error_d_world_T_source_body, d_error_d_world_T_source_body, 1e-6);
    const auto numerical_d_error_d_inverse_depth =
      gtsam::numericalDerivative32<Eigen::Vector2d, gtsam::Pose3, vc::InverseDepthMeasurement, gtsam::Pose3>(
        boost::function<Eigen::Vector2d(const gtsam::Pose3&, const vc::InverseDepthMeasurement&, const gtsam::Pose3&)>(
          boost::bind(&gtsam::InverseDepthProjectionFactor::evaluateError, factor, _1, _2, _3, boost::none, boost::none,
                      boost::none)),
        world_T_source_body, inverse_depth_measurement, world_T_target_body);
    EXPECT_MATRIX_NEAR(numerical_d_error_d_inverse_depth, d_error_d_inverse_depth, 1e-6);
    const auto numerical_d_error_d_world_T_target_body =
      gtsam::numericalDerivative33<Eigen::Vector2d, gtsam::Pose3, vc::InverseDepthMeasurement, gtsam::Pose3>(
        boost::function<Eigen::Vector2d(const gtsam::Pose3&, const vc::InverseDepthMeasurement&, const gtsam::Pose3&)>(
          boost::bind(&gtsam::InverseDepthProjectionFactor::evaluateError, factor, _1, _2, _3, boost::none, boost::none,
                      boost::none)),
        world_T_source_body, inverse_depth_measurement, world_T_target_body);
    EXPECT_MATRIX_NEAR(numerical_d_error_d_world_T_target_body, d_error_d_world_T_target_body, 1e-6);
  }
}

TEST(InverseDepthProjectionFactorTester, Optimization) {
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
    const auto perfect_target_measurement = inverse_depth_measurement.Project(world_T_source_body, world_T_target_body);
    ASSERT_TRUE(perfect_target_measurement != boost::none);
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1e-6, 1e-6));
    const gtsam::Key world_T_source_body_key = 1;
    const gtsam::Key inverse_depth_measurement_key = 2;
    const gtsam::Key world_T_target_body_key = 3;
    const gtsam::InverseDepthProjectionFactor factor(*perfect_target_measurement, noise, world_T_source_body_key,
                                                     inverse_depth_measurement_key, world_T_target_body_key);
    // No noise
    {
      gtsam::NonlinearFactorGraph graph;
      graph.add(factor);
      gtsam::Values values;
      values.insert(world_T_source_body_key, world_T_source_body);
      values.insert(inverse_depth_measurement_key, inverse_depth_measurement);
      values.insert(world_T_target_body_key, world_T_target_body);
      const auto result = gtsam::LevenbergMarquardtOptimizer(graph, values).optimize();
      EXPECT_MATRIX_NEAR(world_T_source_body, result.at<gtsam::Pose3>(world_T_source_body_key), 1e-6);
      EXPECT_NEAR(inverse_depth,
                  (result.at<vc::InverseDepthMeasurement>(inverse_depth_measurement_key)).inverse_depth(), 1e-6);
      EXPECT_MATRIX_NEAR(world_T_target_body, result.at<gtsam::Pose3>(world_T_target_body_key), 1e-6);
    }
    // Noisy inverse depth
    {
      gtsam::NonlinearFactorGraph graph;
      graph.add(factor);
      gtsam::Values values;
      values.insert(world_T_source_body_key, world_T_source_body);
      // Make sure inverse depth is still positive but starts with a lot of noise
      const double noisy_inverse_depth =
        inverse_depth + lc::RandomDouble(-1.0 * inverse_depth + 1e-6, 2.0 * inverse_depth);
      const auto noisy_inverse_depth_measurement =
        vc::InverseDepthMeasurement(noisy_inverse_depth, source_measurement, intrinsics, body_T_cam);
      values.insert(inverse_depth_measurement_key, noisy_inverse_depth_measurement);
      values.insert(world_T_target_body_key, world_T_target_body);
      // Set non noisy values constant
      gtsam::NonlinearEquality1<gtsam::Pose3> world_T_source_body_equality_factor(world_T_source_body,
                                                                                  world_T_source_body_key);
      graph.add(world_T_source_body_equality_factor);
      gtsam::NonlinearEquality1<gtsam::Pose3> world_T_target_body_equality_factor(world_T_target_body,
                                                                                  world_T_target_body_key);
      graph.add(world_T_target_body_equality_factor);
      const auto result = gtsam::LevenbergMarquardtOptimizer(graph, values).optimize();
      EXPECT_MATRIX_NEAR(world_T_source_body, result.at<gtsam::Pose3>(world_T_source_body_key), 1e-6);
      EXPECT_NEAR(inverse_depth,
                  (result.at<vc::InverseDepthMeasurement>(inverse_depth_measurement_key)).inverse_depth(), 1e-6);
      EXPECT_MATRIX_NEAR(world_T_target_body, result.at<gtsam::Pose3>(world_T_target_body_key), 1e-6);
    }
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
