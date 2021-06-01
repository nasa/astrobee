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

#include <graph_localizer/point_to_line_factor.h>
#include <localization_common/logger.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtest/gtest.h>

namespace {
// TODO(rsoussan): move this to a utils fcn
gtsam::Pose3 RandomPose() {
  std::random_device dev;
  std::mt19937 rng(dev());
  gtsam::Rot3 rot = gtsam::Rot3::Random(rng);
  gtsam::Point3 trans = Eigen::Vector3d::Random();
  return gtsam::Pose3(rot, trans);
}
}  // namespace

namespace sym = gtsam::symbol_shorthand;
TEST(PointToLineFactorTester, Jacobian) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 sensor_t_point = Eigen::Vector3d::Random();
    const gtsam::Pose3 world_T_line = RandomPose();
    const gtsam::Pose3 body_T_sensor = RandomPose();
    const gtsam::Pose3 world_T_body = RandomPose();
    const auto noise = gtsam::noiseModel::Unit::Create(2);
    const gtsam::PointToLineFactor factor(sensor_t_point, world_T_line, body_T_sensor, noise, sym::P(0));
    gtsam::Matrix H;
    const auto factor_error = factor.evaluateError(world_T_body, H);
    const auto numerical_H = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
      boost::function<gtsam::Vector(const gtsam::Pose3&)>(
        boost::bind(&gtsam::PointToLineFactor::evaluateError, factor, _1, boost::none)),
      world_T_body, 1e-5);
    ASSERT_TRUE(numerical_H.isApprox(H.matrix(), 1e-6));
  }
}

TEST(PointToLineFactorTester, InvariantToZTranslation) {
  double error_1_norm;
  double error_2_norm;
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 3.0);
    const gtsam::PointToLineFactor factor(sensor_t_point, world_T_line, body_T_sensor, noise, sym::P(0));
    error_1_norm = (factor.evaluateError(world_T_body)).norm();
  }
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 5.0);
    const gtsam::PointToLineFactor factor(sensor_t_point, world_T_line, body_T_sensor, noise, sym::P(0));
    error_2_norm = (factor.evaluateError(world_T_body)).norm();
  }
  EXPECT_DOUBLE_EQ(error_1_norm, error_2_norm);
}

TEST(PointToLineFactorTester, InvariantToRotationAboutZAxis) {
  double error_1_norm;
  double error_2_norm;
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  const gtsam::Point3 sensor_t_point(1.0, 2.0, 3.0);
  {
    const gtsam::PointToLineFactor factor(sensor_t_point, world_T_line, body_T_sensor, noise, sym::P(0));
    error_1_norm = (factor.evaluateError(world_T_body)).norm();
  }
  {
    const gtsam::Rot3 z_axis_rotation = gtsam::Rot3::Rz(2.13);
    const gtsam::Point3 sensor_t_rotated_point = z_axis_rotation * sensor_t_point;
    const gtsam::PointToLineFactor factor(sensor_t_rotated_point, world_T_line, body_T_sensor, noise, sym::P(0));
    error_2_norm = (factor.evaluateError(world_T_body)).norm();
  }
  EXPECT_DOUBLE_EQ(error_1_norm, error_2_norm);
}

TEST(PointToLineFactorTester, IncreaseErrorWithIncreasedXDistance) {
  double error_1_norm;
  double error_2_norm;
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 3.0);
    const gtsam::PointToLineFactor factor(sensor_t_point, world_T_line, body_T_sensor, noise, sym::P(0));
    error_1_norm = (factor.evaluateError(world_T_body)).norm();
  }
  {
    const gtsam::Point3 sensor_t_point(30.3, 2.0, 3.0);
    const gtsam::PointToLineFactor factor(sensor_t_point, world_T_line, body_T_sensor, noise, sym::P(0));
    error_2_norm = (factor.evaluateError(world_T_body)).norm();
  }
  EXPECT_LT(error_1_norm, error_2_norm);
}

TEST(PointToLineFactorTester, IncreaseErrorWithIncreasedYDistance) {
  double error_1_norm;
  double error_2_norm;
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  {
    const gtsam::Point3 sensor_t_point(1.0, 2.0, 3.0);
    const gtsam::PointToLineFactor factor(sensor_t_point, world_T_line, body_T_sensor, noise, sym::P(0));
    error_1_norm = (factor.evaluateError(world_T_body)).norm();
  }
  {
    const gtsam::Point3 sensor_t_point(1.0, -17.2, 3.0);
    const gtsam::PointToLineFactor factor(sensor_t_point, world_T_line, body_T_sensor, noise, sym::P(0));
    error_2_norm = (factor.evaluateError(world_T_body)).norm();
  }
  EXPECT_LT(error_1_norm, error_2_norm);
}
TEST(PointToLineFactorTester, ZeroErrorForZeroOffest) {
  double error_norm;
  const gtsam::Pose3 world_T_line = gtsam::Pose3::identity();
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const auto noise = gtsam::noiseModel::Unit::Create(2);
  const gtsam::Point3 sensor_t_point(0, 0, 0);
  const gtsam::PointToLineFactor factor(sensor_t_point, world_T_line, body_T_sensor, noise, sym::P(0));
  error_norm = (factor.evaluateError(world_T_body)).norm();
  EXPECT_DOUBLE_EQ(error_norm, 0);
}
