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

#include <parameter_reader/graph_vio.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <node_adders/utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace gv = graph_vio;
namespace na = node_adders;
namespace pr = parameter_reader;

class GraphVIOParameterReaderTest : public ::testing::Test {
 public:
  GraphVIOParameterReaderTest() {}

  void SetUp() final {
    lc::SetEnvironmentConfigs();
    config_reader::ConfigReader config;
    lc::LoadGraphVIOConfig(config);
    pr::LoadGraphVIOParams(config, params_);
  }

  gv::GraphVIOParams params_;
};

TEST_F(GraphVIOParameterReaderTest, StandstillFactorAdderParams) {
  const auto& params = params_.standstill_factor_adder;
  EXPECT_EQ(params.enabled, true);
  EXPECT_NEAR(params.huber_k, 1.345, 1e-6);
  EXPECT_EQ(params.add_velocity_prior, true);
  EXPECT_EQ(params.add_pose_between_factor, true);
  EXPECT_NEAR(params.prior_velocity_stddev, 0.01, 1e-6);
  EXPECT_NEAR(params.pose_between_factor_translation_stddev, 0.01, 1e-6);
  EXPECT_NEAR(params.pose_between_factor_rotation_stddev, 0.01, 1e-6);
}

TEST_F(GraphVIOParameterReaderTest, VOFactorAdderParams) {
  const auto& params = params_.vo_smart_projection_factor_adder;
  EXPECT_EQ(params.enabled, true);
  EXPECT_NEAR(params.huber_k, 1.345, 1e-6);
  // Spaced Feature Tracker Params
  EXPECT_EQ(params.spaced_feature_tracker.remove_undetected_feature_tracks, true);
  EXPECT_EQ(params.spaced_feature_tracker.measurement_spacing, 2);
  EXPECT_EQ(params.max_num_factors, 8);
  EXPECT_EQ(params.min_num_points_per_factor, 2);
  EXPECT_EQ(params.max_num_points_per_factor, 6);
  EXPECT_NEAR(params.min_avg_distance_from_mean, 0.075, 1e-6);
  EXPECT_EQ(params.robust, true);
  EXPECT_EQ(params.rotation_only_fallback, true);
  EXPECT_EQ(params.fix_invalid_factors, true);
  EXPECT_EQ(params.scale_noise_with_num_points, true);
  EXPECT_NEAR(params.noise_scale, 2.0, 1e-6);
  // Taken using current nav cam extrinsics
  const gtsam::Pose3 expected_body_T_cam(gtsam::Rot3(0.500, 0.500, 0.500, 0.500),
                                         gtsam::Point3(0.1177, -0.0422, -0.0826));
  EXPECT_MATRIX_NEAR(params.body_T_cam, expected_body_T_cam, 1e-6);
  // Taken using current nav cam intrinsics, undistorted with no skew so only expected
  // non-zero focal lengths
  gtsam::Vector5 expected_intrinsics;
  expected_intrinsics = (gtsam::Vector(5) << 608.8073, 607.61439, 0, 0, 0).finished();
  EXPECT_MATRIX_NEAR(params.cam_intrinsics->vector(), expected_intrinsics, 1e-6);
  const double expected_sigma = dynamic_cast<gtsam::noiseModel::Isotropic*>(params.cam_noise.get())->sigma();
  EXPECT_NEAR(0.1, expected_sigma, 1e-6);
  // Smart Factor
  EXPECT_NEAR(params.smart_factor.triangulation.rankTolerance, 1e-9, 1e-6);
  EXPECT_NEAR(params.smart_factor.triangulation.landmarkDistanceThreshold, 500, 1e-6);
  EXPECT_NEAR(params.smart_factor.triangulation.dynamicOutlierRejectionThreshold, 50, 1e-6);
  EXPECT_EQ(params.smart_factor.triangulation.enableEPI, false);
  EXPECT_EQ(params.smart_factor.verboseCheirality, false);
  EXPECT_NEAR(params.smart_factor.retriangulationThreshold, 1e-5, 1e-6);
  EXPECT_EQ(params.smart_factor.degeneracyMode, gtsam::DegeneracyMode::HANDLE_INFINITY);
}

TEST_F(GraphVIOParameterReaderTest, CombinedNavStateNodeAdderParams) {
  const auto& params = params_.combined_nav_state_node_adder;
  EXPECT_NEAR(params.huber_k, 1.345, 1e-6);
  EXPECT_EQ(params.add_priors, true);
  EXPECT_NEAR(params.ideal_duration, 3.25, 1e-6);
  EXPECT_EQ(params.min_num_states, 3);
  EXPECT_EQ(params.max_num_states, 8);
}

TEST_F(GraphVIOParameterReaderTest, CombinedNavStateNodeAdderModelParams) {
  const auto& params = params_.combined_nav_state_node_adder_model;
  EXPECT_NEAR(params.huber_k, 1.345, 1e-6);
  // IMU Integrator
  EXPECT_MATRIX_NEAR(params.imu_integrator.gravity, Eigen::Vector3d::Zero(), 1e-6);
  // Taken using current nav cam extrinsics
  const gtsam::Pose3 expected_body_T_imu(gtsam::Rot3(0.70710678118, 0, 0, 0.70710678118),
                                         gtsam::Point3(0.0386, 0.0247, -0.01016));
  EXPECT_MATRIX_NEAR(params.imu_integrator.body_T_imu, expected_body_T_imu, 1e-6);
  EXPECT_NEAR(params.imu_integrator.gyro_sigma, 0.00001, 1e-6);
  EXPECT_NEAR(params.imu_integrator.accel_sigma, 0.0005, 1e-6);
  EXPECT_NEAR(params.imu_integrator.accel_bias_sigma, 0.0005, 1e-6);
  EXPECT_NEAR(params.imu_integrator.gyro_bias_sigma, 0.0000035, 1e-6);
  EXPECT_NEAR(params.imu_integrator.integration_variance, 0.0001, 1e-6);
  EXPECT_NEAR(params.imu_integrator.bias_acc_omega_int, 0.00015, 1e-6);
  // IMU filter
  EXPECT_EQ(params.imu_integrator.filter.quiet_accel, "ButterO3S125Lp3N33_33");
  EXPECT_EQ(params.imu_integrator.filter.quiet_ang_vel, "ButterO1S125Lp3N33_33");
  EXPECT_EQ(params.imu_integrator.filter.nominal_accel, "ButterO3S125Lp3N41_66");
  EXPECT_EQ(params.imu_integrator.filter.nominal_ang_vel, "ButterO1S125Lp3N41_66");
  EXPECT_EQ(params.imu_integrator.filter.aggressive_accel, "ButterO3S125Lp3N46_66");
  EXPECT_EQ(params.imu_integrator.filter.aggressive_ang_vel, "ButterO1S125Lp3N46_66");
}

TEST_F(GraphVIOParameterReaderTest, NonlinearOptimizerParams) {
  const auto& params = params_.nonlinear_optimizer;
  EXPECT_EQ(params.marginals_factorization, "qr");
  EXPECT_EQ(params.max_iterations, 6);
  EXPECT_EQ(params.verbose, false);
  EXPECT_EQ(params.use_ceres_params, false);
}

TEST_F(GraphVIOParameterReaderTest, SlidingWindowGraphOptimizerParams) {
  const auto& params = params_.sliding_window_graph_optimizer;
  EXPECT_NEAR(params.huber_k, 1.345, 1e-6);
  EXPECT_EQ(params.log_stats_on_destruction, false);
  EXPECT_EQ(params.print_after_optimization, false);
  EXPECT_EQ(params.add_marginal_factors, false);
  EXPECT_EQ(params.slide_window_before_optimization, true);
}

TEST_F(GraphVIOParameterReaderTest, StandstillParams) {
  const auto& params = params_.standstill;
  EXPECT_EQ(params.min_num_points_per_track, 4);
  EXPECT_NEAR(params.duration, 1, 1e-6);
  EXPECT_NEAR(params.max_avg_distance_from_mean, 0.075, 1e-6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
