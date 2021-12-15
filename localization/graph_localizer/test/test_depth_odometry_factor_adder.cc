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
  params.position_covariance_threshold = 100;
  params.orientation_covariance_threshold = 100;
  params.point_to_point_error_threshold = 10.0;
  params.pose_translation_norm_threshold = 3.0;
  params.max_num_points_between_factors = 100;
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

void AddInlierAndOutlierPoints(const int num_inliers, const int num_outliers,
                               const double point_to_point_error_threshold, lm::DepthOdometryMeasurement& measurement) {
  for (int i = 0; i < num_inliers; ++i) {
    const Eigen::Vector3d source_point = lc::RandomVector();
    const Eigen::Vector3d target_point = measurement.odometry.sensor_F_source_T_target.pose.inverse() * source_point;
    measurement.correspondences.source_3d_points.emplace_back(source_point);
    measurement.correspondences.target_3d_points.emplace_back(target_point);
  }
  for (int i = 0; i < num_outliers; ++i) {
    const Eigen::Vector3d source_point = lc::RandomVector();
    const Eigen::Vector3d noise(1.1 * point_to_point_error_threshold, 0, 0);
    const Eigen::Vector3d target_point =
      measurement.odometry.sensor_F_source_T_target.pose.inverse() * (source_point + noise);
    measurement.correspondences.source_3d_points.emplace_back(source_point);
    measurement.correspondences.target_3d_points.emplace_back(target_point);
  }
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

TEST(DepthOdometryFactorAdderTester, Points) {
  auto params = DefaultParams();
  params.use_points_between_factor = true;
  gl::DepthOdometryFactorAdder factor_adder(params);
  const lc::Time source_timestamp = 0;
  const lc::Time target_timestamp = 0.1;
  const Eigen::Isometry3d relative_pose = lc::RandomIsometry3d();
  auto measurement = MeasurementFromPose(relative_pose, source_timestamp, target_timestamp);
  const int num_inliers = 2;
  const int num_outliers = 2;
  AddInlierAndOutlierPoints(num_inliers, num_outliers, params.point_to_point_error_threshold, measurement);
  ASSERT_EQ(measurement.correspondences.source_3d_points.size(), num_inliers + num_outliers);
  const auto factors_to_add_vec = factor_adder.AddFactors(measurement);
  ASSERT_EQ(factors_to_add_vec.size(), 1);
  EXPECT_EQ(factors_to_add_vec[0].timestamp(), target_timestamp);
  ASSERT_EQ(factors_to_add_vec[0].Get().size(), num_inliers);
  // Factor 0
  {
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
    // Check Factor
    {
      const auto factor = dynamic_cast<const gtsam::PointToPointBetweenFactor*>(factor_to_add.factor.get());
      ASSERT_TRUE(factor);
      const auto& source_point = factor->sensor_t_point_source();
      EXPECT_PRED2(lc::MatrixEquality<6>, source_point.matrix(),
                   measurement.correspondences.source_3d_points[0].matrix());
      const auto& target_point = factor->sensor_t_point_target();
      EXPECT_PRED2(lc::MatrixEquality<6>, target_point.matrix(),
                   measurement.correspondences.target_3d_points[0].matrix());
      const auto& body_T_sensor = factor->body_T_sensor();
      EXPECT_PRED2(lc::MatrixEquality<6>, body_T_sensor.matrix(), params.body_T_sensor.matrix());
      const auto& keys = factor->keys();
      const auto& key_info = factor_to_add.key_infos[0];
      EXPECT_EQ(keys[0], key_info.UninitializedKey());
      EXPECT_EQ(keys[1], key_info.UninitializedKey());
    }
  }
  // Factor 1
  {
    const auto& factor_to_add = factors_to_add_vec[0].Get()[1];
    const auto factor = dynamic_cast<const gtsam::PointToPointBetweenFactor*>(factor_to_add.factor.get());
    ASSERT_TRUE(factor);
    const auto& source_point = factor->sensor_t_point_source();
    EXPECT_PRED2(lc::MatrixEquality<6>, source_point.matrix(),
                 measurement.correspondences.source_3d_points[1].matrix());
    const auto& target_point = factor->sensor_t_point_target();
    EXPECT_PRED2(lc::MatrixEquality<6>, target_point.matrix(),
                 measurement.correspondences.target_3d_points[1].matrix());
    const auto& body_T_sensor = factor->body_T_sensor();
    EXPECT_PRED2(lc::MatrixEquality<6>, body_T_sensor.matrix(), params.body_T_sensor.matrix());
    const auto& keys = factor->keys();
    const auto& key_info = factor_to_add.key_infos[0];
    EXPECT_EQ(keys[0], key_info.UninitializedKey());
    EXPECT_EQ(keys[1], key_info.UninitializedKey());
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
