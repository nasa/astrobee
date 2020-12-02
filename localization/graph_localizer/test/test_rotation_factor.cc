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

#include <graph_localizer/pose_rotation_factor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PoseTranslationPrior.h>

#include <glog/logging.h>
#include <gtest/gtest.h>

namespace sym = gtsam::symbol_shorthand;
TEST(RotationFactorTester, Jacobian) {
  const gtsam::Pose3 world_T_body_1;
  const gtsam::Point3 body_1_t_body_2(0.1, 0.1, 0.1);
  const gtsam::Rot3 body_1_R_body_2(gtsam::Rot3::RzRyRx(3.0 * M_PI / 180.0, 2.0 * M_PI / 180.0, 1.0 * M_PI / 180.0));
  const gtsam::Pose3 body_1_T_body_2(body_1_R_body_2, body_1_t_body_2);
  const gtsam::Pose3 world_T_body_2 = world_T_body_1 * body_1_T_body_2;
  // Use body_1_T_body_2 as a pertubation
  const gtsam::Pose3 world_T_perturbed_body_2 = world_T_body_2 * body_1_T_body_2;
  const auto noise = gtsam::noiseModel::Unit::Create(3);
  const gtsam::PoseRotationFactor rotation_factor(body_1_R_body_2, noise, sym::P(0), sym::P(1));
  gtsam::Matrix factor_H1, factor_H2;
  const auto factor_error =
    rotation_factor.evaluateError(world_T_body_1, world_T_perturbed_body_2, factor_H1, factor_H2);
  const auto numerical_H1 = gtsam::numericalDerivative21<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(
    boost::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Pose3&)>(
      boost::bind(&gtsam::PoseRotationFactor::evaluateError, rotation_factor, _1, _2, boost::none, boost::none)),
    world_T_body_1, world_T_perturbed_body_2, 1e-5);
  ASSERT_TRUE(numerical_H1.isApprox(factor_H1.matrix(), 1e-6));
  const auto numerical_H2 = gtsam::numericalDerivative22<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(
    boost::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Pose3&)>(
      boost::bind(&gtsam::PoseRotationFactor::evaluateError, rotation_factor, _1, _2, boost::none, boost::none)),
    world_T_body_1, world_T_perturbed_body_2, 1e-5);
  ASSERT_TRUE(numerical_H2.isApprox(factor_H2.matrix(), 1e-6));
}
