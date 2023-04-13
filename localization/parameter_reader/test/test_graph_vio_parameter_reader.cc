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

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace gv = graph_vio;
namespace pr = parameter_reader;

class GraphVIOParameterReaderTest : public ::testing::Test {
 public:
  GraphVIOParameterReaderTest() {}

  void SetUp() final {
    config_reader::ConfigReader config;
    lc::LoadGraphVIOConfig(config);
      config.AddFile("transforms.config");
        config.AddFile("cameras.config");
        config.AddFile("geometry.config");
        if (!config.ReadFiles()) {
          LogFatal("Failed to read config files.");
        }
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
  EXPECT_EQ(params.max_num_factors, 13);
  EXPECT_EQ(params.min_num_points_per_factor, 2);
  EXPECT_EQ(params.max_num_points_per_factor, 7);
  EXPECT_NEAR(params.min_avg_distance_from_mean, 0.075, 1e-6);
  EXPECT_EQ(params.robust, true);
  EXPECT_EQ(params.rotation_only_fallback, true);
  EXPECT_EQ(params.fix_invalid_factors, true);
  EXPECT_EQ(params.scale_noise_with_num_points, true);
  EXPECT_NEAR(params.noise_scale, 2.0, 1e-6);
  // Taken using current nav cam extrinsics
  const gtsam::Pose3 expected_body_T_cam
  EXPECT_MATRIX_NEAR(body_T_cam, , 1e-6);
  /*  boost::shared_ptr<gtsam::Cal3_S2> cam_intrinsics;
    gtsam::SharedIsotropic cam_noise;*/
  // Smart Factor
  EXPECT_NEAR(params.smart_factor.triangulation.rankTolerance, 1e-9, 1e-6);
  EXPECT_NEAR(params.smart_factor.triangulation.landmarkDistanceThreshold, 500, 1e-6);
  EXPECT_NEAR(params.smart_factor.triangulation.dynamicOutlierRejectionThreshold, 50, 1e-6);
  EXPECT_EQ(params.smart_factor.triangulation.enableEPI, false);
  EXPECT_EQ(params.smart_factor.verboseCheirality, false);
  EXPECT_NEAR(params.smart_factor.retriangulationThreshold, 1e-5, 1e-6);
  EXPECT_EQ(params.smart_factor.degeneracyMode, gtsam::DegeneracyMode::HANDLE_INFINITY);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
