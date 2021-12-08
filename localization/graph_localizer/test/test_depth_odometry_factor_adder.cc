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

#include <graph_localizer/depth_odometry_factor_adder.h>
#include <graph_localizer/point_to_point_between_factor.h>
#include <graph_localizer/test_utilities.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/depth_odometry_measurement.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtest/gtest.h>

namespace gl = graph_localizer;
namespace go = graph_optimizer;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;

gl::DepthOdometryFactorAdderParams DefaultParams() {
  gl::DepthOdometryFactorAdderParams params;
  params.enabled = true;
  params.huber_k = 1.345;
  params.noise_scale = 1.0;
  params.use_points_between_factor = false;
  params.point_to_point_error_threshold = 10.0;
  params.pose_translation_norm_threshold = 3.0;
  params.body_T_sensor = lc::GtPose(Eigen::Isometry3d::Identity());
  return params;
}

lm::DepthOdometryMeasurement MeasurementFromPose(const Eigen::Isometry3d& pose, const lc::Time source_time,
                                                 const lc::Time target_time) {
  lm::Odometry odometry;
  odometry.source_time = source_time;
  odometry.target_time = target_time;
  odometry.sensor_F_source_T_target.pose = pose;
  odometry.body_F_source_T_target.pose = pose;
  std::vector<Eigen::Vector3d> empty_points;
  lm::DepthCorrespondences correspondences(empty_points, empty_points);
  return lm::DepthOdometryMeasurement(odometry, correspondences, target_time);
}

TEST(DepthOdometryFactorAdderTester, ValidPose) {
  const auto params = DefaultParams();
  gl::DepthOdometryFactorAdder factor_adder(params);
  const lc::Time source_timestamp = 0;
  const lc::Time target_timestamp = 0.1;
  const Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
  const auto measurement = MeasurementFromPose(relative_pose, source_timestamp, target_timestamp);
  const auto factors_to_add_vec = factor_adder.AddFactors(measurement);
  ASSERT_EQ(factors_to_add_vec.size(), 1);
  EXPECT_EQ(factors_to_add_vec[0].timestamp(), target_timestamp);
  ASSERT_EQ(factors_to_add_vec[0].Get().size(), 1);
  const auto& factor_to_add = factors_to_add_vec[0].Get()[0];
  // Check Key Info
  {
    const auto& key_infos = factor_to_add.key_infos;
    ASSERT_EQ(key_infos.size(), 2);
    // Key Info Source
    {
      const auto& key_info = key_infos[0];
      EXPECT_FALSE(key_info.is_static());
      EXPECT_NEAR(key_info.timestamp(), source_timestamp, 1e-6);
      EXPECT_EQ(key_info.UninitializedKey(), sym::P(0));
      EXPECT_EQ(key_info.node_updater_type(), go::NodeUpdaterType::CombinedNavState);
    }
    // Key Info Target
    {
      const auto& key_info = key_infos[1];
      EXPECT_FALSE(key_info.is_static());
      EXPECT_NEAR(key_info.timestamp(), target_timestamp, 1e-6);
      EXPECT_EQ(key_info.UninitializedKey(), sym::P(0));
      EXPECT_EQ(key_info.node_updater_type(), go::NodeUpdaterType::CombinedNavState);
    }
  }
  // Check Factors
  {
    const auto factor = dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(factor_to_add.factor.get());
    ASSERT_TRUE(factor);
    const auto& pose = factor->measured();
    EXPECT_PRED2(lc::MatrixEquality<6>, pose.matrix(), relative_pose.matrix());
    const auto& keys = factor->keys();
    const auto& key_info = factor_to_add.key_infos[0];
    EXPECT_EQ(keys[0], key_info.UninitializedKey());
    EXPECT_EQ(keys[1], key_info.UninitializedKey());
  }
}

TEST(DepthOdometryFactorAdderTester, InvalidPose) {
  const auto params = DefaultParams();
  gl::DepthOdometryFactorAdder factor_adder(params);
  const lc::Time source_timestamp = 0;
  const lc::Time target_timestamp = 0.1;
  Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
  // Ensure translation is above threshold
  relative_pose.translation() += Eigen::Vector3d(1.1, 0, 0) * params.pose_translation_norm_threshold;
  const auto measurement = MeasurementFromPose(relative_pose, source_timestamp, target_timestamp);
  const auto factors_to_add_vec = factor_adder.AddFactors(measurement);
  ASSERT_EQ(factors_to_add_vec.size(), 0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
