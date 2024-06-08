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
#include <nodes/timestamped_nodes.h>
#include <nodes/values.h>

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
  SimplePoseNodeAdder() : values_(std::make_shared<no::Values>()), nodes_(values_) {}

  void AddInitialNodesAndPriors(gtsam::NonlinearFactorGraph& graph) final{};

  bool AddNode(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final {
    if (!CanAddNode(timestamp)) return false;
    if (!values_) std::cout << "no values!" << std::endl;
    values().Add(gtsam::Pose3::identity());
    return true;
  }

  bool CanAddNode(const localization_common::Time timestamp) const final {
    return timestamp <= latest_measurement_time_;
  }

  // Assumes integer timestamps that perfectly cast to ints.
  // First key is pose key.
  gtsam::KeyVector Keys(const localization_common::Time timestamp) const final {
    gtsam::KeyVector keys;
    // Offset by 1 since node keys start at 1
    keys.emplace_back(gtsam::Key(static_cast<int>(timestamp + 1)));
    return keys;
  }

  const no::Values& values() const { return *values_; }

  no::Values& values() { return *values_; }

  no::TimestampedNodes<gtsam::Pose3>& nodes() { return nodes_; }

  // Simulate measurement delay for node adder and control end of measurements time.
  int latest_measurement_time_ = 10;

 private:
  std::shared_ptr<no::Values> values_;
  no::TimestampedNodes<gtsam::Pose3> nodes_;
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
    pose_prior_noise_sigmas_ = (gtsam::Vector(6) << params_.prior_translation_stddev, params_.prior_translation_stddev,
                                params_.prior_translation_stddev, params_.prior_quaternion_stddev,
                                params_.prior_quaternion_stddev, params_.prior_quaternion_stddev)
                                 .finished();
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

  void EXPECT_SAME_POSE_FACTOR(const int factor_index, const int measurement_index) {
    const auto pose_factor = dynamic_cast<gtsam::LocPoseFactor*>(factors_[factor_index].get());
    ASSERT_TRUE(pose_factor);
    const gtsam::Pose3 factor_pose = pose_factor->prior();
    const gtsam::Pose3 measurement_pose = measurements_[measurement_index].global_T_cam * params_.body_T_cam.inverse();
    EXPECT_MATRIX_NEAR(factor_pose, measurement_pose, 1e-6);
  }

  void EXPECT_SAME_PROJECTION_FACTOR(const int factor_index, const int measurement_time, const int measurement_index) {
    const auto projection_factor = dynamic_cast<gtsam::LocProjectionFactor<>*>(factors_[factor_index].get());
    ASSERT_TRUE(projection_factor);
    const auto& measurement = measurements_[measurement_index].matched_projections[measurement_index];
    EXPECT_MATRIX_NEAR(projection_factor->measured(), measurement.image_point, 1e-6);
    EXPECT_MATRIX_NEAR(projection_factor->landmark_point(), measurement.map_point, 1e-6);
    EXPECT_MATRIX_NEAR(projection_factor->body_P_sensor()->matrix(), params_.body_T_cam, 1e-6);
    EXPECT_MATRIX_NEAR(projection_factor->calibration()->matrix(), params_.cam_intrinsics->matrix(), 1e-6);
  }

  void EXPECT_SAME_NOISE(const int factor_index, const gtsam::SharedNoiseModel& noise) {
    const auto projection_factor = dynamic_cast<gtsam::LocProjectionFactor<>*>(factors_[factor_index].get());
    ASSERT_TRUE(projection_factor);
    EXPECT_MATRIX_NEAR(na::Covariance(projection_factor->noiseModel()), na::Covariance(noise), 1e-6);
  }

  void EXPECT_SAME_NOISE_NON_ROBUST(const int factor_index, const gtsam::SharedNoiseModel& noise) {
    const auto projection_factor = dynamic_cast<gtsam::LocProjectionFactor<>*>(factors_[factor_index].get());
    ASSERT_TRUE(projection_factor);
    const auto factor_noise =
      dynamic_cast<gtsam::noiseModel::Gaussian*>(projection_factor->noiseModel().get())->covariance();
    const auto expected_noise = dynamic_cast<gtsam::noiseModel::Gaussian*>(noise.get())->covariance();
    EXPECT_MATRIX_NEAR(factor_noise, expected_noise, 1e-6);
  }

  void EXPECT_SAME_POSE_NOISE(const int factor_index, const gtsam::SharedNoiseModel& noise) {
    const auto pose_factor = dynamic_cast<gtsam::LocPoseFactor*>(factors_[factor_index].get());
    ASSERT_TRUE(pose_factor);
    EXPECT_MATRIX_NEAR(na::Covariance(pose_factor->noiseModel()), na::Covariance(noise), 1e-6);
  }

  std::unique_ptr<fa::LocFactorAdder<SimplePoseNodeAdder>> factor_adder_;
  std::shared_ptr<SimplePoseNodeAdder> node_adder_;
  gtsam::NonlinearFactorGraph factors_;
  const int num_times_ = 10;
  const int num_projections_per_measurement_ = 3;
  std::vector<lm::MatchedProjectionsMeasurement> measurements_;
  fa::LocFactorAdderParams params_;
  gtsam::Vector6 pose_prior_noise_sigmas_;
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
}

TEST_F(LocFactorAdderTest, MaxProjectionFactors) {
  auto params = DefaultParams();
  params.add_projection_factors = true;
  params.max_num_projection_factors = 2;
  Initialize(params);
  factor_adder_->AddMeasurement(measurements_[0]);
  // Add first factors, only max_num_projection_factors should be added
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), params.max_num_projection_factors);
  EXPECT_EQ(factors_.size(), params.max_num_projection_factors);
  EXPECT_SAME_PROJECTION_FACTOR(0, 0, 0);
  EXPECT_SAME_PROJECTION_FACTOR(1, 0, 1);
}

TEST_F(LocFactorAdderTest, MinNumMeasurementsPerFactor) {
  auto params = DefaultParams();
  params.add_projection_factors = true;
  params.add_pose_priors = true;
  params.min_num_matches_per_measurement = 10;
  Initialize(params);
  factor_adder_->AddMeasurement(measurements_[0]);
  // Add first factors, none should be added
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 0);
  EXPECT_EQ(factors_.size(), 0);
}

TEST_F(LocFactorAdderTest, ProjectionNoise) {
  auto params = DefaultParams();
  params.add_projection_factors = true;
  Initialize(params);
  factor_adder_->AddMeasurement(measurements_[0]);
  // Add first factors
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), params.max_num_projection_factors);
  const auto expected_noise =
    localization_common::Robust(gtsam::SharedIsotropic(gtsam::noiseModel::Isotropic::Sigma(
                                  2, params_.projection_noise_scale * params_.cam_noise->sigma())),
                                params_.huber_k);
  EXPECT_SAME_NOISE(0, expected_noise);
}

TEST_F(LocFactorAdderTest, ScaleProjectionNoiseWithNumLandmarks) {
  auto params = DefaultParams();
  params.add_projection_factors = true;
  params.scale_projection_noise_with_num_landmarks = true;
  Initialize(params);
  // Add a measurement with only one match
  auto measurement_0 = measurements_[0];
  // Remove all measurements after the first
  measurement_0.matched_projections.erase(measurement_0.matched_projections.begin() + 1,
                                          measurement_0.matched_projections.end());
  factor_adder_->AddMeasurement(measurement_0);
  // Add first factors
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 1);
  EXPECT_EQ(factors_.size(), 1);
  {
    // Ratio is 1 for the first measurement
    const double num_landmarks_ratio = 1;
    const double noise_scale = params_.projection_noise_scale * std::pow(num_landmarks_ratio, 2);
    const auto expected_noise = localization_common::Robust(
      gtsam::SharedIsotropic(gtsam::noiseModel::Isotropic::Sigma(2, noise_scale * params_.cam_noise->sigma())),
      params_.huber_k);
    EXPECT_SAME_NOISE(0, expected_noise);
  }
  // Add second measurement with all matches
  factor_adder_->AddMeasurement(measurements_[1]);
  // Add second factors
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 3);
  EXPECT_EQ(factors_.size(), 4);
  {
    // New average is (1+3)/2 = 2, so ratio is 2/3 for the second measurement
    const double num_landmarks_ratio = 2.0 / 3.0;
    const double noise_scale = params_.projection_noise_scale * std::pow(num_landmarks_ratio, 2);
    const auto expected_noise = localization_common::Robust(
      gtsam::SharedIsotropic(gtsam::noiseModel::Isotropic::Sigma(2, noise_scale * params_.cam_noise->sigma())),
      params_.huber_k);
    EXPECT_SAME_NOISE(1, expected_noise);
    EXPECT_SAME_NOISE(2, expected_noise);
    EXPECT_SAME_NOISE(3, expected_noise);
  }
}

TEST_F(LocFactorAdderTest, ScaleProjectionNoiseWithLandmarkDistance) {
  auto params = DefaultParams();
  params.add_projection_factors = true;
  params.scale_projection_noise_with_landmark_distance = true;
  Initialize(params);
  factor_adder_->AddMeasurement(measurements_[0]);
  // Add first factors
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), params.max_num_projection_factors);
  const auto world_T_body = node_adder_->values().Value<gtsam::Pose3>(factors_[0]->keys()[0]);
  ASSERT_TRUE(world_T_body != boost::none);
  // Check first factor noise
  {
    const Eigen::Vector3d& world_t_landmark = measurements_[0].matched_projections[0].map_point;
    const Eigen::Isometry3d nav_cam_T_world =
      localization_common::EigenPose(*world_T_body * params_.body_T_cam).inverse();
    const gtsam::Point3 nav_cam_t_landmark = nav_cam_T_world * world_t_landmark;
    // Don't use robust cost here to more effectively correct a drift occurance
    const auto expected_noise = gtsam::SharedIsotropic(
      gtsam::noiseModel::Isotropic::Sigma(2, params_.projection_noise_scale * 1.0 / nav_cam_t_landmark.z()));
    EXPECT_SAME_NOISE_NON_ROBUST(0, expected_noise);
  }
  // Check second factor noise
  {
    const Eigen::Vector3d& world_t_landmark = measurements_[0].matched_projections[1].map_point;
    const Eigen::Isometry3d nav_cam_T_world =
      localization_common::EigenPose(*world_T_body * params_.body_T_cam).inverse();
    const gtsam::Point3 nav_cam_t_landmark = nav_cam_T_world * world_t_landmark;
    // Don't use robust cost here to more effectively correct a drift occurance
    const auto expected_noise = gtsam::SharedIsotropic(
      gtsam::noiseModel::Isotropic::Sigma(2, params_.projection_noise_scale * 1.0 / nav_cam_t_landmark.z()));
    EXPECT_SAME_NOISE_NON_ROBUST(1, expected_noise);
  }
  // Check third factor noise
  {
    const Eigen::Vector3d& world_t_landmark = measurements_[0].matched_projections[2].map_point;
    const Eigen::Isometry3d nav_cam_T_world =
      localization_common::EigenPose(*world_T_body * params_.body_T_cam).inverse();
    const gtsam::Point3 nav_cam_t_landmark = nav_cam_T_world * world_t_landmark;
    // Don't use robust cost here to more effectively correct a drift occurance
    const auto expected_noise = gtsam::SharedIsotropic(
      gtsam::noiseModel::Isotropic::Sigma(2, params_.projection_noise_scale * 1.0 / nav_cam_t_landmark.z()));
    EXPECT_SAME_NOISE_NON_ROBUST(2, expected_noise);
  }
}

TEST_F(LocFactorAdderTest, MinReprojectionError) {
  auto params = DefaultParams();
  params.add_projection_factors = true;
  params.max_valid_projection_error = 1e6;
  Initialize(params);
  // Add a measurement with only one match
  auto measurement_0 = measurements_[0];
  // Remove all measurements after the first
  measurement_0.matched_projections.erase(measurement_0.matched_projections.begin() + 1,
                                          measurement_0.matched_projections.end());
  factor_adder_->AddMeasurement(measurement_0);
  // Add first factors
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 1);
  const auto world_T_body = node_adder_->values().Value<gtsam::Pose3>(factors_[0]->keys()[0]);
  ASSERT_TRUE(world_T_body != boost::none);
  double projection_error;
  // Check projection error
  {
    const auto projection_factor = dynamic_cast<gtsam::LocProjectionFactor<>*>(factors_[0].get());
    ASSERT_TRUE(projection_factor);
    projection_error = projection_factor->evaluateError(*world_T_body).norm();
  }

  const double epsilon = 1e-3;
  // Redo adding factors, but set projection error threshold slightly below previous error.
  // Factor shouldn't be added.
  {
    factors_ = gtsam::NonlinearFactorGraph();
    auto params = DefaultParams();
    params.add_projection_factors = true;
    params.max_valid_projection_error = projection_error - epsilon;
    Initialize(params);
    // Add a measurement with only one match
    auto measurement_0 = measurements_[0];
    // Remove all measurements after the first
    measurement_0.matched_projections.erase(measurement_0.matched_projections.begin() + 1,
                                            measurement_0.matched_projections.end());
    factor_adder_->AddMeasurement(measurement_0);
    // Factor should fail due to reprojection error
    EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 0);
  }
  // Redo adding factors, but set projection error threshold slightly above previous error.
  // Factor should be added.
  {
    factors_ = gtsam::NonlinearFactorGraph();
    auto params = DefaultParams();
    params.add_projection_factors = true;
    params.max_valid_projection_error = projection_error + epsilon;
    Initialize(params);
    // Add a measurement with only one match
    auto measurement_0 = measurements_[0];
    // Remove all measurements after the first
    measurement_0.matched_projections.erase(measurement_0.matched_projections.begin() + 1,
                                            measurement_0.matched_projections.end());
    factor_adder_->AddMeasurement(measurement_0);
    // Factor should succeed
    EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 1);
    EXPECT_SAME_PROJECTION_FACTOR(0, 0, 0);
  }
}

TEST_F(LocFactorAdderTest, PoseFactors) {
  auto params = DefaultParams();
  params.add_pose_priors = true;
  Initialize(params);
  factor_adder_->AddMeasurement(measurements_[0]);
  // Add first factor
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 1);
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_SAME_POSE_FACTOR(0, 0);
  const auto pose_noise = localization_common::Robust(
    gtsam::noiseModel::Diagonal::Sigmas(
      Eigen::Ref<const Eigen::VectorXd>(params_.pose_noise_scale * pose_prior_noise_sigmas_)),
    params_.huber_k);
  EXPECT_SAME_POSE_NOISE(0, pose_noise);

  factor_adder_->AddMeasurement(measurements_[1]);
  // Add second factor
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 1);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_POSE_FACTOR(0, 0);
  EXPECT_SAME_POSE_NOISE(0, pose_noise);
  EXPECT_SAME_POSE_FACTOR(1, 1);
  EXPECT_SAME_POSE_NOISE(1, pose_noise);
}

TEST_F(LocFactorAdderTest, PoseFactorsTooSoon) {
  auto params = DefaultParams();
  params.add_pose_priors = true;
  Initialize(params);
  factor_adder_->AddMeasurement(measurements_[0]);
  // Add first factor
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 1);
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_SAME_POSE_FACTOR(0, 0);
  // Add factor too soon, set node adder latest measurement before factor newest_allowed_time
  // so factor can't be created.
  node_adder_->latest_measurement_time_ = 0.5;
  factor_adder_->AddMeasurement(measurements_[1]);
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 0);
  EXPECT_EQ(factors_.size(), 1);
  EXPECT_SAME_POSE_FACTOR(0, 0);
  // Change node adder latest measurement time back, make sure measurement for factor
  // adder hasn't been removed and factor can still be created.
  node_adder_->latest_measurement_time_ = 2;
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 1);
  EXPECT_EQ(factors_.size(), 2);
  EXPECT_SAME_POSE_FACTOR(0, 0);
  EXPECT_SAME_POSE_FACTOR(1, 1);
}

TEST_F(LocFactorAdderTest, ScalePoseNoiseWithNumLandmarks) {
  auto params = DefaultParams();
  params.add_pose_priors = true;
  params.scale_pose_noise_with_num_landmarks = true;
  Initialize(params);
  // Add a measurement with only one match
  auto measurement_0 = measurements_[0];
  // Remove all measurements after the first
  measurement_0.matched_projections.erase(measurement_0.matched_projections.begin() + 1,
                                          measurement_0.matched_projections.end());
  factor_adder_->AddMeasurement(measurement_0);
  // Add first factors
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 1);
  EXPECT_EQ(factors_.size(), 1);
  {
    // Ratio is 1 for the first measurement
    const double num_landmarks_ratio = 1;
    const double noise_scale = params_.pose_noise_scale * std::pow(num_landmarks_ratio, 2);
    const auto expected_noise = localization_common::Robust(
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(noise_scale * pose_prior_noise_sigmas_)),
      params_.huber_k);
    EXPECT_SAME_POSE_NOISE(0, expected_noise);
  }
  // Add second measurement with all matches
  factor_adder_->AddMeasurement(measurements_[1]);
  // Add second factors
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 1);
  EXPECT_EQ(factors_.size(), 2);
  {
    // New average is (1+3)/2 = 2, so ratio is 2/3 for the second measurement
    const double num_landmarks_ratio = 2.0 / 3.0;
    const double noise_scale = params_.pose_noise_scale * std::pow(num_landmarks_ratio, 2);
    const auto expected_noise = localization_common::Robust(
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(noise_scale * pose_prior_noise_sigmas_)),
      params_.huber_k);
    EXPECT_SAME_POSE_NOISE(1, expected_noise);
  }
}

TEST_F(LocFactorAdderTest, PoseFallback) {
  auto params = DefaultParams();
  params.add_projection_factors = true;
  params.add_pose_priors = false;
  params.add_prior_if_projection_factors_fail = true;
  Initialize(params);
  factor_adder_->AddMeasurement(measurements_[0]);
  // Add first factors
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), num_projections_per_measurement_);
  EXPECT_EQ(factors_.size(), 3);
  EXPECT_SAME_PROJECTION_FACTOR(0, 0, 0);
  EXPECT_SAME_PROJECTION_FACTOR(1, 0, 1);
  EXPECT_SAME_PROJECTION_FACTOR(2, 0, 2);

  // Force cheirality error for projections, add second factor, expect pose prior to be added
  auto measurement_1 = measurements_[1];
  for (int i = 0; i < num_projections_per_measurement_; ++i) {
    measurement_1.matched_projections[i].map_point = lm::MapPoint(-100 + i, -99 + i, -98 + i);
  }
  factor_adder_->AddMeasurement(measurement_1);
  EXPECT_EQ(factor_adder_->AddFactors(0, 1, factors_), 1);
  EXPECT_EQ(factors_.size(), 4);
  EXPECT_SAME_PROJECTION_FACTOR(0, 0, 0);
  EXPECT_SAME_PROJECTION_FACTOR(1, 0, 1);
  EXPECT_SAME_PROJECTION_FACTOR(2, 0, 2);
  EXPECT_SAME_POSE_FACTOR(3, 1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
