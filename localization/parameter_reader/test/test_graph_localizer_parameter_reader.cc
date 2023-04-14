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

#include <parameter_reader/graph_localizer.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <node_adders/utilities.h>

#include <gtest/gtest.h>

namespace gl = graph_localizer;
namespace lc = localization_common;
namespace na = node_adders;
namespace pr = parameter_reader;

class GraphLocalizerParameterReaderTest : public ::testing::Test {
 public:
  GraphLocalizerParameterReaderTest() {}

  void SetUp() final {
    lc::SetEnvironmentConfigs();
    config_reader::ConfigReader config;
    lc::LoadGraphLocalizerConfig(config);
    pr::LoadGraphLocalizerParams(config, params_);
  }

  gl::GraphLocalizerParams params_;
};

TEST_F(GraphLocalizerParameterReaderTest, SparseMapLocFactorAdderParams) {
  const auto& params = params_.sparse_map_loc_factor_adder;
  EXPECT_EQ(params.enabled, true);
  EXPECT_NEAR(params.huber_k, 1.345, 1e-6);
  EXPECT_EQ(params.add_pose_priors, false);
  EXPECT_EQ(params.add_projection_factors, true);
  EXPECT_EQ(params.add_prior_if_projection_factors_fail, true);
  EXPECT_NEAR(params.prior_translation_stddev, 0.06, 1e-6);
  EXPECT_NEAR(params.prior_quaternion_stddev, 0.06, 1e-6);
  EXPECT_EQ(params.scale_pose_noise_with_num_landmarks, true);
  EXPECT_EQ(params.scale_projection_noise_with_num_landmarks, false);
  EXPECT_EQ(params.scale_projection_noise_with_landmark_distance, false);
  EXPECT_NEAR(params.pose_noise_scale, 7, 1e-6);
  EXPECT_NEAR(params.projection_noise_scale, 60, 1e-6);
  EXPECT_EQ(params.max_num_projection_factors, 50);
  EXPECT_EQ(params.min_num_matches_per_measurement, 5);
  EXPECT_NEAR(params.max_valid_projection_error, 30, 1e-6);
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
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
