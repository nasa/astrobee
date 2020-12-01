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

#include <graph_localizer/rotation_factor_adder.h>

#include <glog/logging.h>
#include <gtest/gtest.h>

namespace gl = graph_localizer;
class RotationFactorAdderTester : public ::testing::Test {
 protected:
  virtual void SetUp() {
    gl::FeatureTrackerParams feature_tracker_params;
    feature_tracker_params.sliding_window_duration = 2.0;
    rotation_factor_adder_params_.min_avg_disparity = 0.2;
    rotation_factor_adder_params_.rotation_stddev = 0.1;
    Eigen::Matrix<double, 4, 4> body_T_nav_cam;
    body_T_nav_cam << 0, 0, 1, 0.1177, 1, 0, 0, -0.0422, 0, 1, 0, -0.0826, 0, 0, 0, 1;
    rotation_factor_adder_params_.body_T_nav_cam = gtsam::Pose3(body_T_nav_cam.matrix());
    rotation_factor_adder_params_.nav_cam_intrinsics = gtsam::Cal3_S2(608.807, 607.614, 0, 0, 0);
    rotation_factor_adder_.reset(new gl::RotationFactorAdder(rotation_factor_adder_params_, feature_tracker_));
  }

  void AddMeasurementsForTwoFrames(const gtsam::Pose3& cam_1_T_cam_2, const int num_points,
                                   const localization_common::Time time_diff = 1.0) {}

  gl::RotationFactorAdderParams rotation_factor_adder_params_;
  std::shared_ptr<gl::FeatureTracker> feature_tracker_;
  std::unique_ptr<gl::RotationFactorAdder> rotation_factor_adder_;
};

TEST_F(RotationFactorAdderTester, A) {
  // Make feature tracks with given rotation
  // Make call add factors on feature tracks
  // check resulting rotation!!
}
