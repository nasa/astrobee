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

#include "test_utilities.h"  // NOLINT
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <node_updaters/pose_node_updater.h>

#include <gtest/gtest.h>

namespace go = graph_optimizer;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace nu = node_updaters;

class PoseNodeUpdaterTest : public ::testing::Test {
 public:
  PoseNodeUpdaterTest() : time_increment_(1.0 / 125.0), start_time_(time_increment_), num_measurements_(20) {
    params_ = nu::DefaultPoseNodeUpdaterParams();
std::shared_ptr<go::TimestampedNodes<gtsam::Pose3>> nodes(new go::TimestampedNodes<gtsam::Pose3>);
    std::shared_ptr<nu::PoseNodeUpdateModel> node_update_model(new nu::PoseNodeUpdateModel());
    pose_node_updater_.reset(new nu::PoseNodeUpdater(params_, nodes, node_update_model));
  }

  void SetUp() final {}

  std::unique_ptr<nu::PoseNodeUpdater> pose_node_updater_;
  nu::PoseNodeUpdaterParams params_;
 private:
  const double time_increment_;
  const lc::Time start_time_;
  const int num_measurements_;
};

TEST_F(PoseNodeUpdaterTest, AddRemoveCanUpdate) {
  EXPECT_FALSE(pose_node_updater_->CanUpdate(10.1));
  // Add measurement 0
  constexpr lc::Time time_0 = 1.1;
  const auto pose_0 = lm::TimestampedPoseWithCovariance(lc::RandomPoseWithCovariance(), time_0);
  pose_node_updater_->AddMeasurement(pose_0);
  EXPECT_TRUE(pose_node_updater_->CanUpdate(time_0));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_0 + 0.1));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_0 - 0.1));
  // Add measurement 1
  constexpr lc::Time time_1 = 2.2;
  const auto pose_1 = lm::TimestampedPoseWithCovariance(lc::RandomPoseWithCovariance(), time_1);
  pose_node_updater_->AddMeasurement(pose_1);
  EXPECT_TRUE(pose_node_updater_->CanUpdate(time_1));
  EXPECT_TRUE(pose_node_updater_->CanUpdate((time_0 + time_1)/2.0));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_0 - 0.1));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_1 + 0.1));
  // Add measurement 2
  constexpr lc::Time time_2 = 3.3;
  const auto pose_2 = lm::TimestampedPoseWithCovariance(lc::RandomPoseWithCovariance(), time_2);
  pose_node_updater_->AddMeasurement(pose_2);
  EXPECT_TRUE(pose_node_updater_->CanUpdate(time_2));
  EXPECT_TRUE(pose_node_updater_->CanUpdate(time_1 + 0.1));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_0 - 0.1));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_2 + 0.1));
  // Add measurement 3
  constexpr lc::Time time_3 = 4.45;
  const auto pose_3 = lm::TimestampedPoseWithCovariance(lc::RandomPoseWithCovariance(), time_3);
  pose_node_updater_->AddMeasurement(pose_3);
  EXPECT_TRUE(pose_node_updater_->CanUpdate(time_3));
  EXPECT_TRUE(pose_node_updater_->CanUpdate(time_1 + 0.1));
  EXPECT_TRUE(pose_node_updater_->CanUpdate(time_2 + 0.1));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_0 - 0.1));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_3 + 0.1));

  // Remove measurements 0 and 1
  pose_node_updater_->RemoveMeasurements(time_1 + 0.1);
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_0));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_1));
  EXPECT_TRUE(pose_node_updater_->CanUpdate(time_2));
  EXPECT_TRUE(pose_node_updater_->CanUpdate(time_2 + 0.1));
  EXPECT_TRUE(pose_node_updater_->CanUpdate(time_3));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_3 + 0.1));
}


TEST_F(PoseNodeUpdaterTest, AddInitialValuesAndPriorsUsingParams) {
  const auto& nodes = pose_node_updater_->nodes();
  EXPECT_TRUE(nodes.empty());
  gtsam::NonlinearFactorGraph factors;
  pose_node_updater_->AddInitialValuesAndPriors(factors);
  // TODO(rsoussan): check that prior factors have been added!
  EXPECT_EQ(nodes.size(), 1);
  EXPECT_TRUE(nodes.Contains(params_.starting_time));
  const auto node = nodes.Node(params_.starting_time);
  ASSERT_TRUE(node != boost::none);
  EXPECT_MATRIX_NEAR(node->matrix(), params_.start_node, 1e-6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
