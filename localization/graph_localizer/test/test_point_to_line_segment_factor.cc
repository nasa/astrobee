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

#include "test_utilities.h"  // NOLINT
#include <graph_localizer/point_to_line_segment_factor.h>
#include <localization_common/logger.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtest/gtest.h>

namespace gl = graph_localizer;
namespace sym = gtsam::symbol_shorthand;
TEST(PointToLineSegmentFactorTester, Jacobian) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 sensor_t_point = gl::RandomVector();
    const gtsam::Pose3 world_T_line = gl::RandomPose();
    const gtsam::Pose3 body_T_sensor = gl::RandomPose();
    const gtsam::Pose3 world_T_body = gl::RandomPose();
    const double line_length = gl::RandomPositiveDouble();
    const auto noise = gtsam::noiseModel::Unit::Create(2);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0));
    gtsam::Matrix H;
    const auto factor_error = factor.evaluateError(world_T_body, H);
    const auto numerical_H = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
      boost::function<gtsam::Vector(const gtsam::Pose3&)>(
        boost::bind(&gtsam::PointToLineSegmentFactor::evaluateError, factor, _1, boost::none)),
      world_T_body, 1e-5);
    ASSERT_TRUE(numerical_H.isApprox(H.matrix(), 1e-6));
  }
}

TEST(PointToLineSegmentFactorTester, JacobianWithSilu) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 sensor_t_point = gl::RandomVector();
    const gtsam::Pose3 world_T_line = gl::RandomPose();
    const gtsam::Pose3 body_T_sensor = gl::RandomPose();
    const gtsam::Pose3 world_T_body = gl::RandomPose();
    const double line_length = gl::RandomPositiveDouble();
    const auto noise = gtsam::noiseModel::Unit::Create(2);
    const bool use_silu = true;
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0), use_silu);
    gtsam::Matrix H;
    const auto factor_error = factor.evaluateError(world_T_body, H);
    const auto numerical_H = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
      boost::function<gtsam::Vector(const gtsam::Pose3&)>(
        boost::bind(&gtsam::PointToLineSegmentFactor::evaluateError, factor, _1, boost::none)),
      world_T_body, 1e-5);
    ASSERT_TRUE(numerical_H.isApprox(H.matrix(), 1e-6));
  }
}

TEST(PointToLineSegmentFactorTester, ZeroZErrorInBetweenSegmentEndpoints) {
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const double line_length = 0.3;
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 0.1);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0));
    const auto error = factor.evaluateError(world_T_body);
    EXPECT_DOUBLE_EQ(error.z(), 0);
  }
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, -0.1);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0));
    const auto error = factor.evaluateError(world_T_body);
    EXPECT_DOUBLE_EQ(error.z(), 0);
  }
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 0);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0));
    const auto error = factor.evaluateError(world_T_body);
    EXPECT_DOUBLE_EQ(error.z(), 0);
  }
}

TEST(PointToLineSegmentFactorTester, NonZeroZErrorOutisdeOfSegmentEndpoints) {
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const double line_length = 0.3;
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 1);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0));
    const auto error = factor.evaluateError(world_T_body);
    EXPECT_DOUBLE_EQ(std::abs(error.z()), 0.85);
  }
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, -1);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0));
    const auto error = factor.evaluateError(world_T_body);
    EXPECT_DOUBLE_EQ(std::abs(error.z()), 0.85);
  }
}

TEST(PointToLineSegmentFactorTester, InvariantToRotationAboutZAxis) {
  double error_1_norm;
  double error_2_norm;
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const double line_length = 0.3;
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  const gtsam::Point3 sensor_t_point(1.0, 2.0, 3.0);
  {
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0));
    error_1_norm = (factor.evaluateError(world_T_body)).norm();
  }
  {
    const gtsam::Rot3 z_axis_rotation = gtsam::Rot3::Rz(2.13);
    const gtsam::Point3 sensor_t_rotated_point = z_axis_rotation * sensor_t_point;
    const gtsam::PointToLineSegmentFactor factor(sensor_t_rotated_point, world_T_line, body_T_sensor, line_length,
                                                 noise, sym::P(0));
    error_2_norm = (factor.evaluateError(world_T_body)).norm();
  }
  EXPECT_DOUBLE_EQ(error_1_norm, error_2_norm);
}

TEST(PointToLineSegmentFactorTester, IncreaseErrorWithIncreasedXDistance) {
  double error_1_norm;
  double error_2_norm;
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const double line_length = 0.3;
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 3.0);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0));
    error_1_norm = (factor.evaluateError(world_T_body)).norm();
  }
  {
    const gtsam::Point3 sensor_t_point(30.3, 2.0, 3.0);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0));
    error_2_norm = (factor.evaluateError(world_T_body)).norm();
  }
  EXPECT_LT(error_1_norm, error_2_norm);
}

TEST(PointToLineSegmentFactorTester, IncreaseErrorWithIncreasedYDistance) {
  double error_1_norm;
  double error_2_norm;
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const double line_length = 0.3;
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 3.0);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0));
    error_1_norm = (factor.evaluateError(world_T_body)).norm();
  }
  {
    const gtsam::Point3 sensor_t_point(1.0, -17.2, 3.0);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0));
    error_2_norm = (factor.evaluateError(world_T_body)).norm();
  }
  EXPECT_LT(error_1_norm, error_2_norm);
}

// Tests with Silu
TEST(PointToLineSegmentFactorTester, ZeroZErrorInBetweenSegmentEndpointsWithSilu) {
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const double line_length = 0.3;
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 0.1);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0), true);
    const auto error = factor.evaluateError(world_T_body);
    EXPECT_NEAR(error.z(), 0, 1e-1);
  }
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, -0.1);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0), true);
    const auto error = factor.evaluateError(world_T_body);
    EXPECT_NEAR(error.z(), 0, 1e-1);
  }
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 0);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0), true);
    const auto error = factor.evaluateError(world_T_body);
    EXPECT_NEAR(error.z(), 0, 1e-1);
  }
}

TEST(PointToLineSegmentFactorTester, NonZeroZErrorOutisdeOfSegmentEndpointsWithSilu) {
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const double line_length = 0.3;
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 1);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0), true);
    const auto error = factor.evaluateError(world_T_body);
    EXPECT_NEAR(std::abs(error.z()), 0.85, 3e-1);
  }
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, -1);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0), true);
    const auto error = factor.evaluateError(world_T_body);
    EXPECT_NEAR(std::abs(error.z()), 0.85, 3e-1);
  }
}

TEST(PointToLineSegmentFactorTester, InvariantToRotationAboutZAxisWithSilu) {
  double error_1_norm;
  double error_2_norm;
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const double line_length = 0.3;
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  const gtsam::Point3 sensor_t_point(1.0, 2.0, 3.0);
  {
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0), true);
    error_1_norm = (factor.evaluateError(world_T_body)).norm();
  }
  {
    const gtsam::Rot3 z_axis_rotation = gtsam::Rot3::Rz(2.13);
    const gtsam::Point3 sensor_t_rotated_point = z_axis_rotation * sensor_t_point;
    const gtsam::PointToLineSegmentFactor factor(sensor_t_rotated_point, world_T_line, body_T_sensor, line_length,
                                                 noise, sym::P(0), true);
    error_2_norm = (factor.evaluateError(world_T_body)).norm();
  }
  EXPECT_DOUBLE_EQ(error_1_norm, error_2_norm);
}

TEST(PointToLineSegmentFactorTester, IncreaseErrorWithIncreasedXDistanceWithSilu) {
  double error_1_norm;
  double error_2_norm;
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const double line_length = 0.3;
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 3.0);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0), true);
    error_1_norm = (factor.evaluateError(world_T_body)).norm();
  }
  {
    const gtsam::Point3 sensor_t_point(30.3, 2.0, 3.0);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0), true);
    error_2_norm = (factor.evaluateError(world_T_body)).norm();
  }
  EXPECT_LT(error_1_norm, error_2_norm);
}

TEST(PointToLineSegmentFactorTester, IncreaseErrorWithIncreasedYDistanceWithSilu) {
  double error_1_norm;
  double error_2_norm;
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const double line_length = 0.3;
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 3.0);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0), true);
    error_1_norm = (factor.evaluateError(world_T_body)).norm();
  }
  {
    const gtsam::Point3 sensor_t_point(1.0, -17.2, 3.0);
    const gtsam::PointToLineSegmentFactor factor(sensor_t_point, world_T_line, body_T_sensor, line_length, noise,
                                                 sym::P(0), true);
    error_2_norm = (factor.evaluateError(world_T_body)).norm();
  }
  EXPECT_LT(error_1_norm, error_2_norm);
}
