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

#include <graph_localizer/point_to_point_between_factor.h>
#include <graph_localizer/test_utilities.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtest/gtest.h>

namespace gl = graph_localizer;
namespace lc = localization_common;
namespace sym = gtsam::symbol_shorthand;

TEST(PointToPointBetweenFactorTester, Jacobian) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 sensor_t_point_source = lc::RandomPoint3d();
    const gtsam::Point3 sensor_t_point_target = lc::RandomPoint3d();
    const gtsam::Pose3 body_T_sensor = lc::RandomPose();
    const gtsam::Pose3 world_T_body_source = lc::RandomPose();
    const gtsam::Pose3 world_T_body_target = lc::RandomPose();
    const auto noise = gtsam::noiseModel::Unit::Create(3);
    const gtsam::PointToPointBetweenFactor factor(sensor_t_point_source, sensor_t_point_target, body_T_sensor, noise,
                                                  sym::P(0), sym::P(1));
    gtsam::Matrix H1;
    gtsam::Matrix H2;
    const auto factor_error = factor.evaluateError(world_T_body_source, world_T_body_target, H1, H2);
    const auto numerical_H1 = gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3, gtsam::Pose3>(
      boost::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Pose3&)>(
        boost::bind(&gtsam::PointToPointBetweenFactor::evaluateError, factor, _1, _2, boost::none, boost::none)),
      world_T_body_source, world_T_body_target, 1e-5);
    EXPECT_MATRIX_NEAR(numerical_H1, H1, 1e-6);
    const auto numerical_H2 = gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3, gtsam::Pose3>(
      boost::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Pose3&)>(
        boost::bind(&gtsam::PointToPointBetweenFactor::evaluateError, factor, _1, _2, boost::none, boost::none)),
      world_T_body_source, world_T_body_target, 1e-5);
    EXPECT_MATRIX_NEAR(numerical_H2, H2, 1e-6);
  }
}

TEST(PointToPointBetweenFactorTester, SamePointAndPoseError) {
  for (int i = 0; i < 50; ++i) {
    const gtsam::Point3 sensor_t_point_source = lc::RandomPoint3d();
    const gtsam::Point3 sensor_t_point_target = sensor_t_point_source;
    const gtsam::Pose3 body_T_sensor = lc::RandomPose();
    const gtsam::Pose3 world_T_body_source = lc::RandomPose();
    const gtsam::Pose3 world_T_body_target = world_T_body_source;
    const auto noise = gtsam::noiseModel::Unit::Create(3);
    const gtsam::PointToPointBetweenFactor factor(sensor_t_point_source, sensor_t_point_target, body_T_sensor, noise,
                                                  sym::P(0), sym::P(1));
    const auto factor_error = factor.evaluateError(world_T_body_source, world_T_body_target);
    EXPECT_MATRIX_NEAR(factor_error, Eigen::Vector3d::Zero(), 1e-6);
  }
}

TEST(PointToPointBetweenFactorTester, DifferentPointIdentityPosesError) {
  for (int i = 0; i < 50; ++i) {
    const gtsam::Point3 sensor_t_point_source = lc::RandomPoint3d();
    const gtsam::Point3 sensor_t_point_target = lc::RandomPoint3d();
    const gtsam::Pose3 body_T_sensor = lc::GtPose(Eigen::Isometry3d::Identity());
    const gtsam::Pose3 world_T_body_source = lc::GtPose(Eigen::Isometry3d::Identity());
    const gtsam::Pose3 world_T_body_target = world_T_body_source;
    const auto noise = gtsam::noiseModel::Unit::Create(3);
    const gtsam::PointToPointBetweenFactor factor(sensor_t_point_source, sensor_t_point_target, body_T_sensor, noise,
                                                  sym::P(0), sym::P(1));
    const auto factor_error = factor.evaluateError(world_T_body_source, world_T_body_target);
    EXPECT_MATRIX_NEAR(factor_error, (sensor_t_point_source - sensor_t_point_target), 1e-6);
  }
}

TEST(PointToPointBetweenFactorTester, DifferentPointSamePoseError) {
  for (int i = 0; i < 50; ++i) {
    const gtsam::Point3 sensor_t_point_source = lc::RandomPoint3d();
    const gtsam::Point3 sensor_t_point_target = lc::RandomPoint3d();
    const gtsam::Pose3 body_T_sensor = lc::RandomPose();
    const gtsam::Pose3 world_T_body_source = lc::RandomPose();
    const gtsam::Pose3 world_T_body_target = world_T_body_source;
    const auto noise = gtsam::noiseModel::Unit::Create(3);
    const gtsam::PointToPointBetweenFactor factor(sensor_t_point_source, sensor_t_point_target, body_T_sensor, noise,
                                                  sym::P(0), sym::P(1));
    const auto factor_error = factor.evaluateError(world_T_body_source, world_T_body_target);
    EXPECT_MATRIX_NEAR(
      factor_error,
      (world_T_body_source.rotation() * body_T_sensor.rotation() * (sensor_t_point_source - sensor_t_point_target)),
      1e-6);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
