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
    nu::PoseNodeUpdaterParams params = nu::DefaultPoseNodeUpdaterParams();
    params.SetStartNoiseModels();
std::shared_ptr<go::TimestampedNodes<gtsam::Pose3>> nodes(new go::TimestampedNodes<gtsam::Pose3>);
    std::shared_ptr<nu::PoseNodeUpdateModel> node_update_model(new nu::PoseNodeUpdateModel());
    pose_node_updater_.reset(new nu::PoseNodeUpdater(params, nodes, node_update_model));
  }

  void SetUp() final {}

  std::unique_ptr<nu::PoseNodeUpdater> pose_node_updater_;
 private:
  const double time_increment_;
  const lc::Time start_time_;
  const int num_measurements_;
};

TEST_F(PoseNodeUpdaterTest, AddRemoveCanUpdate) {
  EXPECT_FALSE(pose_node_updater_->CanUpdate(10.1));
  constexpr lc::Time time_0 = 1.1;
  const auto pose_0 = lm::TimestampedPoseWithCovariance(lc::RandomPoseWithCovariance(), time_0);
  pose_node_updater_->AddMeasurement(pose_0);
  EXPECT_TRUE(pose_node_updater_->CanUpdate(time_0));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_0 + 0.1));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_0 - 0.1));
  constexpr lc::Time time_1 = 2.2;
  const auto pose_1 = lm::TimestampedPoseWithCovariance(lc::RandomPoseWithCovariance(), time_1);
  pose_node_updater_->AddMeasurement(pose_1);
  EXPECT_TRUE(pose_node_updater_->CanUpdate(time_1));
  EXPECT_TRUE(pose_node_updater_->CanUpdate((time_0 + time_1)/2.0));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_0 - 0.1));
  EXPECT_FALSE(pose_node_updater_->CanUpdate(time_1 + 0.1));
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
