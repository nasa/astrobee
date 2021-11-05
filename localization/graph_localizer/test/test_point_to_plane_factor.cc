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

#include <graph_localizer/point_to_plane_factor.h>
#include <graph_localizer/test_utilities.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_measurements/plane.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtest/gtest.h>

namespace gl = graph_localizer;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
TEST(PointToPlaneFactorTester, Jacobian) {
  for (int i = 0; i < 500; ++i) {
    const gtsam::Point3 sensor_t_point = lc::RandomVector();
    const lm::Plane world_T_handrail_plane = gl::RandomPlane();
    const gtsam::Pose3 body_T_sensor = lc::RandomPose();
    const gtsam::Pose3 world_T_body = lc::RandomPose();
    const auto noise = gtsam::noiseModel::Unit::Create(1);
    const gtsam::PointToPlaneFactor factor(sensor_t_point, world_T_handrail_plane, body_T_sensor, noise, sym::P(0));
    gtsam::Matrix H;
    const auto factor_error = factor.evaluateError(world_T_body, H);
    const auto numerical_H = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
      boost::function<gtsam::Vector(const gtsam::Pose3&)>(
        boost::bind(&gtsam::PointToPlaneFactor::evaluateError, factor, _1, boost::none)),
      world_T_body, 1e-5);
    ASSERT_TRUE(numerical_H.isApprox(H.matrix(), 1e-6));
  }
}
