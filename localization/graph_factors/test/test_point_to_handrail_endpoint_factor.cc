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

#include <graph_factors/point_to_handrail_endpoint_factor.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace sym = gtsam::symbol_shorthand;
TEST(PointToHandrailEndpointFactorTester, Jacobian) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 sensor_t_point = lc::RandomPoint3d();
    const gtsam::Point3 world_t_handrail_endpoint_a = lc::RandomPoint3d();
    const gtsam::Point3 world_t_handrail_endpoint_b = lc::RandomPoint3d();
    const gtsam::Pose3 body_T_sensor = lc::RandomPose();
    const gtsam::Pose3 world_T_body = lc::RandomPose();
    // Ignore case where sensor point is directly between two endpoints as this leads to a known
    // discontinuity in the Jacobian.
    {
      const gtsam::Point3 world_t_point = world_T_body * body_T_sensor * sensor_t_point;
      const double distance_to_endpoint_a = (world_t_point - world_t_handrail_endpoint_a).norm();
      const double distance_to_endpoint_b = (world_t_point - world_t_handrail_endpoint_b).norm();
      if (std::abs(distance_to_endpoint_a - distance_to_endpoint_b) < 1e-4) continue;
    }
    const auto noise = gtsam::noiseModel::Unit::Create(3);
    const gtsam::PointToHandrailEndpointFactor factor(sensor_t_point, world_t_handrail_endpoint_a,
                                                      world_t_handrail_endpoint_b, body_T_sensor, noise, sym::P(0));
    gtsam::Matrix H;
    const auto factor_error = factor.evaluateError(world_T_body, H);
    const auto numerical_H = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
      boost::function<gtsam::Vector(const gtsam::Pose3&)>(
        boost::bind(&gtsam::PointToHandrailEndpointFactor::evaluateError, factor, _1, boost::none)),
      world_T_body);
    EXPECT_MATRIX_NEAR(numerical_H, H, 1e-6);
  }
}

TEST(PointToHandrailEndpointFactorTester, SelectingCorrectEndpoint) {
  const gtsam::Point3 world_t_handrail_endpoint_a(1, 0, 0);
  const gtsam::Point3 world_t_handrail_endpoint_b(2, 0, 0);
  const gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  const gtsam::Pose3 world_T_body = gtsam::Pose3::identity();
  const auto noise = gtsam::noiseModel::Unit::Create(3);
  // Closer to endpoint a
  {
    const gtsam::Point3 sensor_t_point(1.2, 0, 0);
    const gtsam::PointToHandrailEndpointFactor factor(sensor_t_point, world_t_handrail_endpoint_a,
                                                      world_t_handrail_endpoint_b, body_T_sensor, noise, sym::P(0));
    const auto error = factor.evaluateError(world_T_body);
    EXPECT_MATRIX_NEAR(error, gtsam::Vector3(0.2, 0, 0), 1e-6);
  }
  // Closer to endpoint b
  {
    const gtsam::Point3 sensor_t_point(1.6, 0, 0);
    const gtsam::PointToHandrailEndpointFactor factor(sensor_t_point, world_t_handrail_endpoint_a,
                                                      world_t_handrail_endpoint_b, body_T_sensor, noise, sym::P(0));
    const auto error = factor.evaluateError(world_T_body);
    EXPECT_MATRIX_NEAR(error, gtsam::Vector3(-0.4, 0, 0), 1e-6);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
