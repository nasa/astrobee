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

#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <optimizers/nonlinear_optimizer.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace op = optimizers;

class NonlinearOptimizerTest : public ::testing::Test {
 public:
  NonlinearOptimizerTest() {}

  void SetUp() final { Initialize(DefaultNonlinearOptimizerParams()); }

  void Initialize(const op::NonlinearOptimizerParams& params) {
    nonlinear_optimizer_.reset(new op::NonlinearOptimizer(params));
  }

  op::NonlinearOptimizerParams DefaultNonlinearOptimizerParams() {
    op::NonlinearOptimizerParams params;
    params.max_iterations = 10;
    params.verbose = false;
    params.use_ceres_params = false;
    params.marginals_factorization = "qr";
    return params;
  }

  std::unique_ptr<op::NonlinearOptimizer> nonlinear_optimizer_;
  gtsam::Values values_;
};

// Optimize noisy value using perfect prior
TEST_F(NonlinearOptimizerTest, OptimizeNoisyValue) {
  gtsam::NonlinearFactorGraph factors;
  const auto pose_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);

  // Add perfect true pose to prior and noisy pose to values
  const auto true_pose = lc::RandomPose();
  const auto noisy_pose = true_pose * lc::RandomPose();
  const gtsam::Key key(0);
  gtsam::PriorFactor<gtsam::Pose3>::shared_ptr pose_prior_factor(
    new gtsam::PriorFactor<gtsam::Pose3>(key, true_pose, pose_noise));
  factors.push_back(pose_prior_factor);
  values_.insert(key, noisy_pose);

  {
    // Marginals shouldn't exist before optimization is performed.
    const auto marginals = nonlinear_optimizer_->marginals();
    EXPECT_TRUE(marginals == boost::none);
  }

  // Optimize
  nonlinear_optimizer_->Optimize(factors, values_);

  // Pose value should match prior after optimization
  const auto optimized_pose = values_.at<gtsam::Pose3>(key);
  EXPECT_MATRIX_NEAR(optimized_pose, true_pose, 1e-6);

  // Covariance should match prior noise
  const auto covariance = nonlinear_optimizer_->Covariance(key);
  ASSERT_TRUE(covariance != boost::none);
  EXPECT_MATRIX_NEAR(gtsam::Matrix(*covariance), pose_noise->covariance(), 1e-6);

  // Marginals should be available
  const auto marginals = nonlinear_optimizer_->marginals();
  EXPECT_TRUE(marginals != boost::none);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
