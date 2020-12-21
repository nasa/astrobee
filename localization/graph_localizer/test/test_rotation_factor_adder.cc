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
#include <graph_localizer/rotation_factor_adder.h>
#include <localization_common/logger.h>
#include <localization_measurements/feature_point.h>

#include <gtsam/geometry/PinholePose.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PoseTranslationPrior.h>

#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

namespace gl = graph_localizer;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
class RotationFactorAdderTester : public ::testing::Test {
 protected:
  virtual void SetUp() {
    gl::FeatureTrackerParams feature_tracker_params;
    feature_tracker_params.sliding_window_duration = 2.0;
    feature_tracker_.reset(new gl::FeatureTracker(feature_tracker_params));
    rotation_factor_adder_params_.min_avg_disparity = 0.2;
    rotation_factor_adder_params_.rotation_stddev = 0.1;
    rotation_factor_adder_params_.max_percent_outliers = 0.5;
    Eigen::Matrix<double, 4, 4> body_T_nav_cam;
    body_T_nav_cam << 0, 0, 1, 0.1177, 1, 0, 0, -0.0422, 0, 1, 0, -0.0826, 0, 0, 0, 1;
    rotation_factor_adder_params_.body_T_nav_cam = gtsam::Pose3(body_T_nav_cam.matrix());
    rotation_factor_adder_params_.nav_cam_intrinsics = gtsam::Cal3_S2(608.807, 607.614, 0, 0, 0);
    rotation_factor_adder_.reset(new gl::RotationFactorAdder(rotation_factor_adder_params_, feature_tracker_));
  }

  void AddMeasurementsForTwoFrames(const gtsam::Pose3& world_T_cam_1, const gtsam::Pose3& world_T_cam_2,
                                   const int num_points, const localization_common::Time time_diff = 1.0) {
    const gtsam::PinholePose<gtsam::Cal3_S2> cam_1(
      world_T_cam_1, boost::make_shared<gtsam::Cal3_S2>(rotation_factor_adder_params_.nav_cam_intrinsics));
    const gtsam::PinholePose<gtsam::Cal3_S2> cam_2(
      world_T_cam_2, boost::make_shared<gtsam::Cal3_S2>(rotation_factor_adder_params_.nav_cam_intrinsics));
    const double x_min = -1.0;
    const double x_max = 1.0;
    const double y_min = -1.0;
    const double y_max = 1.0;
    const double z_min = 1.0;
    const double z_max = 10.0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_dis(x_min, x_max);
    std::uniform_real_distribution<> y_dis(y_min, y_max);
    std::uniform_real_distribution<> z_dis(z_min, z_max);
    lm::FeaturePoints feature_points_1;
    lm::FeaturePoints feature_points_2;
    for (int i = 0; i < num_points; ++i) {
      const double x = x_dis(gen);
      const double y = y_dis(gen);
      const double z = z_dis(gen);
      const gtsam::Point3 cam_1_t_point(x, y, z);
      const gtsam::Point3 world_t_point = world_T_cam_1 * cam_1_t_point;
      const auto result_1 = cam_1.projectSafe(world_t_point);
      const auto result_2 = cam_2.projectSafe(world_t_point);
      if (!result_1.second || !result_2.second) {
        continue;
      }
      feature_points_1.emplace_back(result_1.first.x(), result_1.first.y(), 1, i, 0);
      feature_points_2.emplace_back(result_2.first.x(), result_2.first.y(), 2, i, 0.1);
    }
    feature_tracker_->UpdateFeatureTracks(feature_points_1);
    feature_tracker_->UpdateFeatureTracks(feature_points_2);
    ASSERT_EQ(feature_points_1.size(), feature_points_2.size());
    ASSERT_GT(feature_points_1.size(), 5);
  }

  gl::RotationFactorAdderParams rotation_factor_adder_params_;
  std::shared_ptr<gl::FeatureTracker> feature_tracker_;
  std::unique_ptr<gl::RotationFactorAdder> rotation_factor_adder_;
};

TEST_F(RotationFactorAdderTester, CorrectRotation) {
  const gtsam::Pose3 world_T_cam_1;
  const gtsam::Point3 cam_1_t_cam_2(0.1, 0.1, 0.1);
  const gtsam::Rot3 cam_1_R_cam_2(gtsam::Rot3::RzRyRx(3.0 * M_PI / 180.0, 2.0 * M_PI / 180.0, 1.0 * M_PI / 180.0));
  const gtsam::Pose3 cam_1_T_cam_2(cam_1_R_cam_2, cam_1_t_cam_2);
  const gtsam::Pose3 world_T_cam_2 = world_T_cam_1 * cam_1_T_cam_2;
  AddMeasurementsForTwoFrames(world_T_cam_1, world_T_cam_2, 30);
  const auto factors_to_add_vec = rotation_factor_adder_->AddFactors(lm::FeaturePointsMeasurement());
  const auto rotation_factor =
    dynamic_cast<gtsam::PoseRotationFactor*>(factors_to_add_vec.front().Get().front().factor.get());
  ASSERT_TRUE(rotation_factor);
  const auto rotation = rotation_factor->rotation();
  const auto world_T_body_1 = world_T_cam_1 * rotation_factor_adder_params_.body_T_nav_cam.inverse();
  const auto world_T_body_2 = world_T_cam_2 * rotation_factor_adder_params_.body_T_nav_cam.inverse();
  const auto body_1_R_body_2 = (world_T_body_1.inverse() * world_T_body_2).rotation();
  ASSERT_TRUE(rotation.equals(body_1_R_body_2, 1e-6));
}

TEST_F(RotationFactorAdderTester, CorrectOptimization) {
  const gtsam::Pose3 world_T_cam_1;
  const gtsam::Point3 cam_1_t_cam_2(0.1, 0.1, 0.1);
  const gtsam::Rot3 cam_1_R_cam_2(gtsam::Rot3::RzRyRx(3.0 * M_PI / 180.0, 2.0 * M_PI / 180.0, 1.0 * M_PI / 180.0));
  const gtsam::Pose3 cam_1_T_cam_2(cam_1_R_cam_2, cam_1_t_cam_2);
  const gtsam::Pose3 world_T_cam_2 = world_T_cam_1 * cam_1_T_cam_2;
  AddMeasurementsForTwoFrames(world_T_cam_1, world_T_cam_2, 30);
  const auto factors_to_add_vec = rotation_factor_adder_->AddFactors(lm::FeaturePointsMeasurement());
  auto rotation_factor =
    dynamic_cast<gtsam::PoseRotationFactor*>(factors_to_add_vec.front().Get().front().factor.get());
  ASSERT_TRUE(rotation_factor);
  const gtsam::Pose3 world_T_body_1 = world_T_cam_1 * rotation_factor_adder_params_.body_T_nav_cam.inverse();
  const gtsam::Pose3 world_T_body_2 = world_T_cam_2 * rotation_factor_adder_params_.body_T_nav_cam.inverse();
  gtsam::Values values_;
  values_.insert(sym::P(1), world_T_body_1);
  // Use cam_1_T_cam_2 as a pertubation and make sure the optimizer corrects this
  const gtsam::Pose3 world_T_perturbed_body_2 = world_T_body_2 * cam_1_T_cam_2;
  values_.insert(sym::P(2), world_T_perturbed_body_2);
  gtsam::NonlinearFactorGraph graph_;
  const auto noise_1 = gtsam::noiseModel::Unit::Create(6);
  const gtsam::PriorFactor<gtsam::Pose3> world_T_body_1_prior(sym::P(1), world_T_body_1, noise_1);
  graph_.push_back(world_T_body_1_prior);
  const auto noise_2 = gtsam::noiseModel::Unit::Create(3);
  gtsam::PoseTranslationPrior<gtsam::Pose3> pose_translation_prior(sym::P(2), world_T_body_2.translation(), noise_2);
  graph_.push_back(pose_translation_prior);
  rotation_factor->keys() = gtsam::KeyVector({sym::P(1), sym::P(2)});
  graph_.push_back(*rotation_factor);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, values_);
  const auto optimized_values = optimizer.optimize();
  const auto optimized_world_T_body_1 = optimized_values.at<gtsam::Pose3>(sym::P(1));
  const auto optimized_world_T_body_2 = optimized_values.at<gtsam::Pose3>(sym::P(2));
  ASSERT_TRUE(optimized_world_T_body_1.equals(world_T_body_1, 1e-6));
  ASSERT_TRUE(optimized_world_T_body_2.equals(world_T_body_2, 1e-6));
}
