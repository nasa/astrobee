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

#include <factor_adders/loc_factor_adder.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/matched_projections_measurement.h>
#include <node_adders/node_adder.h>
#include <node_adders/utilities.h>
#include <nodes/nodes.h>

#include <gtsam/geometry/Pose3.h>

#include <gtest/gtest.h>

namespace fa = factor_adders;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace na = node_adders;
namespace no = nodes;

// Test node adder that just returns keys that should be used.
// Key values are calculated using the integer timestamps passed.
class SimplePoseNodeAdder : public na::NodeAdder {
 public:
  void AddInitialNodesAndPriors(gtsam::NonlinearFactorGraph& graph) final{};

  bool AddNode(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final {
    nodes_.Add(gtsam::Pose3::identity());
    return true;
  }

  bool CanAddNode(const localization_common::Time timestamp) const final { return true; }

  // Assumes integer timestamps that perfectly cast to ints.
  // First key is pose key.
  gtsam::KeyVector Keys(const localization_common::Time timestamp) const final {
    gtsam::KeyVector keys;
    // Offset by 1 since node keys start at 1
    keys.emplace_back(gtsam::Key(static_cast<int>(timestamp + 1)));
    return keys;
  }

  const no::Nodes& nodes() const { return nodes_; }

  no::Nodes& nodes() { return nodes_; }

  std::string type() const final { return "simple_pose_node_adder"; }

 private:
  no::Nodes nodes_;
};

class LocFactorAdderTest : public ::testing::Test {
 public:
  LocFactorAdderTest() { node_adder_.reset(new SimplePoseNodeAdder()); }

  void SetUp() final { AddMeasurements(); }

  void AddMeasurements() {
    for (int time = 0; time < num_times_; ++time) {
      lm::MatchedProjectionsMeasurement measurement;
      measurement.global_T_cam = lc::RandomPose();
      measurement.timestamp = time;
      for (int i = 0; i < num_projections_per_measurement_; ++i) {
        const lm::ImagePoint image_point(i, i + 1);
        const lm::MapPoint map_point(i, i + 1, i + 2);
        const lm::MatchedProjection matched_projection(image_point, map_point, time);
        measurement.matched_projections.emplace_back(matched_projection);
      }
      measurements_.emplace_back(measurement);
    }
  }

  void Initialize(const fa::LocFactorAdderParams& params) {
    factor_adder_.reset(new fa::LocFactorAdder<SimplePoseNodeAdder>(params, node_adder_));
    params_ = params;
  }

  fa::LocFactorAdderParams DefaultParams() {
    fa::LocFactorAdderParams params;
    params.add_pose_priors = false;
    params.add_projection_factors = false;
    params.add_prior_if_projection_factors_fail = false;
    params.prior_translation_stddev = 0.1;
    params.prior_quaternion_stddev = 0.2;
    params.scale_pose_noise_with_num_landmarks = false;
    params.scale_projection_noise_with_num_landmarks = false;
    params.scale_projection_noise_with_landmark_distance = false;
    params.pose_noise_scale = 2;
    params.projection_noise_scale = 2;
    params.max_num_projection_factors = 3;
    params.min_num_matches_per_measurement = 1;
    params.max_valid_projection_error = 1e6;
    params.body_T_cam = gtsam::Pose3::identity();
    params.cam_intrinsics = boost::make_shared<gtsam::Cal3_S2>();
    params.cam_noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);
    params.huber_k = 1.345;
    params.enabled = true;
    return params;
  }

  void EXPECT_SAME_PROJECTION_FACTOR(const int factor_index, const int measurement_time, const int measurement_index) {
    const auto projection_factor = dynamic_cast<gtsam::LocProjectionFactor<>*>(factors_[factor_index].get());
    ASSERT_TRUE(projection_factor);
    const auto& measurement = measurements_[measurement_index].matched_projections[measurement_index];
    EXPECT_MATRIX_NEAR(projection_factor->measured(), measurement.image_point, 1e-6);
    EXPECT_MATRIX_NEAR(projection_factor->landmark_point(), measurement.map_point, 1e-6);
    EXPECT_MATRIX_NEAR(projection_factor->body_P_sensor()->matrix(), params_.body_T_cam, 1e-6);
    EXPECT_MATRIX_NEAR(projection_factor->calibration()->matrix(), params_.cam_intrinsics->matrix(), 1e-6);
    // TODO(rsoussan): check noise!
  }

  std::unique_ptr<fa::LocFactorAdder<SimplePoseNodeAdder>> factor_adder_;
  std::shared_ptr<SimplePoseNodeAdder> node_adder_;
  gtsam::NonlinearFactorGraph factors_;
  const int num_times_ = 10;
  const int num_projections_per_measurement_ = 3;
  std::vector<lm::MatchedProjectionsMeasurement> measurements_;
  fa::LocFactorAdderParams params_;
};

TEST_F(LocFactorAdderTest, ProjectionFactors) {
  auto params = DefaultParams();
  params.add_projection_factors = true;
  Initialize(params);
  factor_adder_->AddMeasurement(measurements_[0]);
  // Add first factors
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), num_projections_per_measurement_);
  EXPECT_EQ(factors_.size(), 3);
  EXPECT_SAME_PROJECTION_FACTOR(0, 0, 0);
  EXPECT_SAME_PROJECTION_FACTOR(1, 0, 1);
  EXPECT_SAME_PROJECTION_FACTOR(2, 0, 2);

  // Add second factors
  factor_adder_->AddMeasurement(measurements_[1]);
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), num_projections_per_measurement_);
  EXPECT_EQ(factors_.size(), 6);
  // t0 factors
  EXPECT_SAME_PROJECTION_FACTOR(0, 0, 0);
  EXPECT_SAME_PROJECTION_FACTOR(1, 0, 1);
  EXPECT_SAME_PROJECTION_FACTOR(2, 0, 2);
  // t1 factors
  EXPECT_SAME_PROJECTION_FACTOR(3, 1, 0);
  EXPECT_SAME_PROJECTION_FACTOR(4, 1, 1);
  EXPECT_SAME_PROJECTION_FACTOR(5, 1, 2);

  // Repeat add factors with no new measurements, nothing should change
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 0);
  EXPECT_EQ(factors_.size(), 6);
  /* // t0 factors
   EXPECT_SAME_PROJECTION_FACTOR(0, 0, 0);
   EXPECT_SAME_PROJECTION_FACTOR(1, 0, 1);
   EXPECT_SAME_PROJECTION_FACTOR(2, 0, 2);
   // t1 factors
   EXPECT_SAME_PROJECTION_FACTOR(3, 1, 0);
   EXPECT_SAME_PROJECTION_FACTOR(4, 1, 1);
   EXPECT_SAME_PROJECTION_FACTOR(5, 1, 2);*/
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
