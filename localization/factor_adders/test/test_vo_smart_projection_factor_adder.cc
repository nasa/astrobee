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

#include <factor_adders/vo_smart_projection_factor_adder.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/feature_points_measurement.h>
#include <node_adders/node_adder.h>
#include <node_adders/utilities.h>
#include <nodes/nodes.h>
#include <vision_common/feature_point.h>

#include <gtest/gtest.h>

namespace fa = factor_adders;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace na = node_adders;
namespace no = nodes;
namespace vc = vision_common;

// Test node adder that just returns keys that should be used.
// Key values are calculated using the integer timestamps passed.
class SimplePoseNodeAdder : public na::NodeAdder {
 public:
  void AddInitialNodesAndPriors(gtsam::NonlinearFactorGraph& graph) final{};

  bool AddNode(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final { return true; }

  bool CanAddNode(const localization_common::Time timestamp) const final { return true; }

  // Assumes integer timestamps that perfectly cast to ints.
  // First key is pose key.
  gtsam::KeyVector Keys(const localization_common::Time timestamp) const final {
    gtsam::KeyVector keys;
    keys.emplace_back(gtsam::Key(static_cast<int>(timestamp)));
    return keys;
  }

  const no::Nodes& nodes() { return nodes_; }

  std::string type() const final { return "simple_pose_node_adder"; }

 private:
  no::Nodes nodes_;
};

class VoSmartProjectionFactorAdderTest : public ::testing::Test {
 public:
  VoSmartProjectionFactorAdderTest() { node_adder_.reset(new SimplePoseNodeAdder()); }

  void SetUp() final { CreateMeasurements(); }

  // Add measurements for each track at multiple timestamps
  void CreateMeasurements() {
    for (int time = 0; time < num_measurements_; ++time) {
      lm::FeaturePointsMeasurement measurement;
      measurement.timestamp = time;
      for (int track_id = 0; track_id < num_tracks_; ++track_id) {
        // Make image points different for different measurements
        const vc::FeaturePoint p(track_id * 10 + time, track_id * 10 + time + 1, time + 1, track_id, time);
        measurement.feature_points.emplace_back(p);
      }
      measurements_.emplace_back(measurement);
    }
  }

  void EXPECT_SAME_FACTOR(const int factor_index, const std::vector<int>& timestamps) {
    const auto factor = factors_[factor_index];
    const auto smart_factor = dynamic_cast<const fa::RobustSmartFactor*>(factor.get());
    ASSERT_TRUE(smart_factor);
    const auto& keys = factor->keys();
    EXPECT_EQ(keys.size(), timestamps.size());
    const auto& measurements = smart_factor->measured();
    EXPECT_EQ(measurements.size(), timestamps.size());
    for (int i = 0; i < keys.size(); ++i) {
      EXPECT_EQ(keys[i], gtsam::Key(timestamps[i]));
      EXPECT_MATRIX_NEAR(measurements[i], measurements_[timestamps[i]].feature_points[factor_index].image_point, 1e-6);
    }

    // Check factor noise
    // Uses inv sigma scaling, applies optional robust loss to whitened noise after
    const double noise_scale =
      params_.scale_noise_with_num_points ? params_.noise_scale * measurements.size() : params_.noise_scale;
    const double expected_sigma = noise_scale * params_.cam_noise->sigma();
    EXPECT_NEAR(smart_factor->noise_inv_sigma(), 1.0 / expected_sigma, 1e-6);
    EXPECT_EQ(smart_factor->robust(), params_.robust);
  }

  lc::Time timestamp(const int measurement_index) { return measurements_[measurement_index].timestamp; }

  void Initialize(const fa::VoSmartProjectionFactorAdderParams& params) {
    factor_adder_.reset(new fa::VoSmartProjectionFactorAdder<SimplePoseNodeAdder>(params, node_adder_));
    params_ = params;
  }

  fa::VoSmartProjectionFactorAdderParams DefaultParams() {
    fa::VoSmartProjectionFactorAdderParams params;
    // Feature Tracker Params
    params.spaced_feature_tracker.measurement_spacing = 0;
    params.spaced_feature_tracker.remove_undetected_feature_tracks = true;
    params.max_num_factors = 2;
    params.min_num_points_per_factor = 2;
    params.max_num_points_per_factor = 3;
    params.min_avg_distance_from_mean = 1e-9;
    params.robust = true;
    params.rotation_only_fallback = false;
    params.splitting = false;
    params.scale_noise_with_num_points = false;
    params.noise_scale = 1;
    params.body_T_cam = gtsam::Pose3::identity();
    params.cam_intrinsics = boost::make_shared<gtsam::Cal3_S2>();
    params.cam_noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);
    // Smart Factor Params
    params.smart_factor.verboseCheirality = false;
    params.smart_factor.setRankTolerance(1e-9);
    params.smart_factor.setLandmarkDistanceThreshold(3);
    params.smart_factor.setDynamicOutlierRejectionThreshold(1);
    params.smart_factor.setRetriangulationThreshold(0.1);
    params.smart_factor.setDegeneracyMode(gtsam::DegeneracyMode::HANDLE_INFINITY);
    params.smart_factor.setEnableEPI(false);
    params.enabled = true;
    params.huber_k = 1.345;
    return params;
  }

  std::unique_ptr<fa::VoSmartProjectionFactorAdder<SimplePoseNodeAdder>> factor_adder_;
  std::shared_ptr<SimplePoseNodeAdder> node_adder_;
  gtsam::NonlinearFactorGraph factors_;
  std::vector<lm::FeaturePointsMeasurement> measurements_;
  fa::VoSmartProjectionFactorAdderParams params_;
  int num_measurements_ = 10;
  int num_tracks_ = 3;
};

TEST_F(VoSmartProjectionFactorAdderTest, AddFactors) {
  auto params = DefaultParams();
  Initialize(params);
  const int max_factors = std::min(params.max_num_factors, num_tracks_);
  // Add first measurement
  // No factors should be added since there are too few measurements for each factor
  factor_adder_->AddMeasurement(measurements_[0]);
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(0), timestamp(0), factors_), 0);
  // Track: Measurement Timestamps
  // 0: 0
  // 1: 0
  // 2: 0
  EXPECT_EQ(factors_.size(), 0);
  // Add second measurement
  factor_adder_->AddMeasurement(measurements_[1]);
  // No factors added if don't include second measurement timestamp
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(0), (timestamp(0) + timestamp(1)) / 2.0, factors_), 0);
  // Track: Measurement Timestamps
  // 0: 0
  // 1: 0
  // 2: 0
  EXPECT_EQ(factors_.size(), 0);
  // All factors added if include second measurement timestamp
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(0), timestamp(1), factors_), max_factors);
  // Track: Measurement Timestamps
  // 0: 0, 1
  // 1: 0, 1
  // 2: 0, 1
  EXPECT_SAME_FACTOR(0, {0, 1});
  EXPECT_SAME_FACTOR(0, {0, 1});
  EXPECT_EQ(factors_.size(), max_factors);
  // Try to add factors from t: 1->2, but no measurement added yet for t2, so not enough
  // measurements to add factors
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(1), timestamp(2), factors_), 0);
  // Track: Measurement Timestamps
  // 0: 1
  // 1: 1
  // 2: 1
  EXPECT_EQ(factors_.size(), 0);
  // Add 3rd measurement
  factor_adder_->AddMeasurement(measurements_[2]);
  // Add factors from t: 1->2
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(1), timestamp(2), factors_), 2);
  // Track: Measurement Timestamps
  // 0: 1, 2
  // 1: 1, 2
  // 2: 1, 2
  EXPECT_SAME_FACTOR(0, {1, 2});
  EXPECT_SAME_FACTOR(1, {1, 2});
  EXPECT_EQ(factors_.size(), 2);
  // Add 4th and 5th Measurement
  factor_adder_->AddMeasurement(measurements_[3]);
  factor_adder_->AddMeasurement(measurements_[4]);
  // Add factors from t: 5->6
  // Should erase all measurements and add no factors
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(5), timestamp(6), factors_), 0);
  // Track: Measurement Timestamps
  // 0:
  // 1:
  // 2:
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(0), timestamp(5), factors_), 0);
  factor_adder_->AddMeasurement(measurements_[5]);
  factor_adder_->AddMeasurement(measurements_[6]);
  factor_adder_->AddMeasurement(measurements_[7]);
  factor_adder_->AddMeasurement(measurements_[8]);
  factor_adder_->AddMeasurement(measurements_[9]);
  // Add factors from t: 5->8
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(5), timestamp(8), factors_), 2);
  // Track: Measurement Timestamps
  // 0: 5, 6, 7, 8
  // 1: 5, 6, 7, 8
  // 2: 5, 6, 7, 8
  EXPECT_EQ(factors_.size(), 2);
  // Since max points per factor is 3, expect only first three measurements to be used.
  EXPECT_SAME_FACTOR(0, {5, 6, 7});
  EXPECT_SAME_FACTOR(1, {5, 6, 7});
}

TEST_F(VoSmartProjectionFactorAdderTest, AddSpacedFactors) {
  auto params = DefaultParams();
  params.spaced_feature_tracker.measurement_spacing = 1;
  Initialize(params);
  const int max_factors = std::min(params.max_num_factors, num_tracks_);
  // Add first measurement
  // No factors should be added since there are too few measurements for each factor
  factor_adder_->AddMeasurement(measurements_[0]);
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(0), timestamp(0), factors_), 0);
  // Track: Measurement Timestamps
  // 0: 0
  // 1: 0
  // 2: 0
  EXPECT_EQ(factors_.size(), 0);
  // Add second measurement
  // No factors added since skip second measurement due to measurement spacing
  factor_adder_->AddMeasurement(measurements_[1]);
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(0), timestamp(1), factors_), 0);
  // Track: Measurement Timestamps (skipped measurements in parenthesis)
  // 0: 0, (1)
  // 1: 0, (1)
  // 2: 0, (1)
  EXPECT_EQ(factors_.size(), 0);
  // Add 3rd measurement
  // Add factors from t: 0->3
  factor_adder_->AddMeasurement(measurements_[2]);
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(0), timestamp(2), factors_), 2);
  // Track: Measurement Timestamps
  // 0: 0, (1), 2
  // 1: 0, (1), 2
  // 2: 0, (1), 2
  EXPECT_SAME_FACTOR(0, {0, 2});
  EXPECT_SAME_FACTOR(1, {0, 2});
  // Adding repeat factors shouldn't change graph
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(0), timestamp(2), factors_), 2);
  // Track: Measurement Timestamps
  // 0: 0, (1), 2
  // 1: 0, (1), 2
  // 2: 0, (1), 2
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_FACTOR(0, {0, 2});
  EXPECT_SAME_FACTOR(1, {0, 2});
  // Add measurements from 3->7
  factor_adder_->AddMeasurement(measurements_[3]);
  factor_adder_->AddMeasurement(measurements_[4]);
  factor_adder_->AddMeasurement(measurements_[5]);
  factor_adder_->AddMeasurement(measurements_[6]);
  factor_adder_->AddMeasurement(measurements_[7]);
  // Add factors from 3 -> 7
  // Only timestamps 4 and 6 should be used
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(3), timestamp(7), factors_), 2);
  // Track: Measurement Timestamps
  // 0: (3), 4, (5), 6, (7)
  // 1: (3), 4, (5), 6, (7)
  // 2: (3), 4, (5), 6, (7)
  EXPECT_SAME_FACTOR(0, {4, 6});
  EXPECT_SAME_FACTOR(1, {4, 6});
}

TEST_F(VoSmartProjectionFactorAdderTest, AvgDistFromMean) {
  auto params = DefaultParams();
  // Each point pair is separated by (1, 1), so for 3 pairs:
  // (0, 1), (1, 2), (2, 3) -> mean: (1, 2)
  // avg dist from mean: |((1,1)*2)/3| = (sqrt(2*(2/3)^2))/3 = 2sqr(2)/9 ~= 0.3142
  // Use a slightly larger min so all tracks are invalid
  params.min_avg_distance_from_mean = 0.3142 + 0.1;
  Initialize(params);
  const int max_factors = std::min(params.max_num_factors, num_tracks_);
  factor_adder_->AddMeasurement(measurements_[0]);
  factor_adder_->AddMeasurement(measurements_[1]);
  factor_adder_->AddMeasurement(measurements_[2]);
  // A factors should fail to be added due to avg distance from mean failure.
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(0), timestamp(0), factors_), 0);

  // Repeat with slightly smaller distance, all factors should be added
  params.min_avg_distance_from_mean = 0.3142 - 0.1;
  Initialize(params);
  factor_adder_->AddMeasurement(measurements_[0]);
  factor_adder_->AddMeasurement(measurements_[1]);
  factor_adder_->AddMeasurement(measurements_[2]);
  EXPECT_EQ(factor_adder_->AddFactors(timestamp(0), timestamp(0), factors_), 2);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
