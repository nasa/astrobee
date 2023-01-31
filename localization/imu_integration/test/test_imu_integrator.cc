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

#include <imu_integration/imu_integrator.h>
#include <localization_common/test_utilities.h>
#include <localization_measurements/imu_measurement.h>

#include <gtest/gtest.h>

namespace ii = imu_integrator;
namespace lc = localization_common;
namespace lm = localization_measurements;

class ConstantAccelerationTest : public ::testing::Test {
 public:
  // Start at time increment so first IMU measurement is after starting combined nav state time
  ConstantAccelerationTest()
      : acceleration_i_(0.1),
        acceleration_(acceleration_i_, acceleration_i_, acceleration_i_),
        time_increment_(1.0 / 125.0),
        start_time_(time_increment_),
        num_measurements_(20) {
    const ii::ImuIntegratorParams params = ii::DefaultImuIntegratorParams();
    imu_integrator_.reset(new ii::ImuIntegrator(params));
  }

  void SetUp() final {
    const std::vector<lm::ImuMeasurement> imu_measurements =
      ii::ConstantAccelerationMeasurements(acceleration_, num_measurements_, start_time_, time_increment_);
    for (const auto& imu_measurement : imu_measurements) {
      imu_integrator_->BufferImuMeasurement(imu_measurement);
    }
  }

  ii::ImuIntegrator& imu_integrator() { return *imu_integrator_; }

  double acceleration_i() { return acceleration_i_; }

  const Eigen::Vector3d& acceleration() { return acceleration_; }

  double time_increment() { return time_increment_; }

  double start_time() { return start_time_; }

  int num_measurements() { return num_measurements_; }

 private:
  std::unique_ptr<ii::ImuIntegrator> imu_integrator_;
  const double acceleration_i_;
  const Eigen::Vector3d acceleration_;
  const double time_increment_;
  const lc::Time start_time_;
  const int num_measurements_;
};

class ConstantAngularVelocityTest : public ::testing::Test {
 public:
  // Start at time increment so first IMU measurement is after starting combined nav state time
  ConstantAngularVelocityTest()
      : angular_velocity_i_(0.1),
        angular_velocity_(angular_velocity_i_, angular_velocity_i_, angular_velocity_i_),
        time_increment_(1.0 / 125.0),
        start_time_(time_increment_),
        num_measurements_(20) {
    const ii::ImuIntegratorParams params = ii::DefaultImuIntegratorParams();
    imu_integrator_.reset(new ii::ImuIntegrator(params));
  }

  void SetUp() final {
    imu_measurements_ =
      ii::ConstantAngularVelocityMeasurements(angular_velocity_, num_measurements_, start_time_, time_increment_);
    for (const auto& imu_measurement : imu_measurements_) {
      imu_integrator_->BufferImuMeasurement(imu_measurement);
    }
  }

  ii::ImuIntegrator& imu_integrator() { return *imu_integrator_; }

  double angular_velocity_i() { return angular_velocity_i_; }

  const Eigen::Vector3d& angular_velocity() { return angular_velocity_; }

  double time_increment() { return time_increment_; }

  double start_time() { return start_time_; }

  int num_measurements() { return num_measurements_; }

  const std::vector<lm::ImuMeasurement>& imu_measurements() { return imu_measurements_; }

 private:
  std::unique_ptr<ii::ImuIntegrator> imu_integrator_;
  const double angular_velocity_i_;
  const Eigen::Vector3d angular_velocity_;
  const double time_increment_;
  const lc::Time start_time_;
  const int num_measurements_;
  std::vector<lm::ImuMeasurement> imu_measurements_;
};

TEST_F(ConstantAccelerationTest, AddAllMeasurements) {
  const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(), 0);
  lc::CombinedNavState imu_augmented_state = initial_state;
  imu_integrator().PimPredict(initial_state, imu_augmented_state);

  EXPECT_NEAR(imu_augmented_state.timestamp(), num_measurements() * time_increment(), 1e-6);
  const double expected_velocity_i = acceleration_i() * num_measurements() * time_increment();
  const gtsam::Vector3 expected_velocity(expected_velocity_i, expected_velocity_i, expected_velocity_i);
  EXPECT_MATRIX_NEAR(imu_augmented_state.velocity(), expected_velocity, 1e-6);
  // x = 1/2*a*t^2
  const double expected_position_i = acceleration_i() * 0.5 * std::pow(num_measurements() * time_increment(), 2);
  const gtsam::Vector3 expected_position(expected_position_i, expected_position_i, expected_position_i);
  EXPECT_MATRIX_NEAR(imu_augmented_state.pose().translation(), expected_position, 1e-6);
}

TEST_F(ConstantAccelerationTest, AddHalfOfMeasurements) {
  const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(), 0);
  const lc::Time imu_augmented_state_start_time = num_measurements() / 2 * time_increment();
  lc::CombinedNavState imu_augmented_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(), imu_augmented_state_start_time);
  imu_integrator().PimPredict(initial_state, imu_augmented_state);

  EXPECT_NEAR(imu_augmented_state.timestamp(), num_measurements() * time_increment(), 1e-6);
  const double expected_velocity_i = acceleration_i() * num_measurements() / 2 * time_increment();
  const gtsam::Vector3 expected_velocity(expected_velocity_i, expected_velocity_i, expected_velocity_i);
  EXPECT_MATRIX_NEAR(imu_augmented_state.velocity(), expected_velocity, 1e-6);
  // x = 1/2*a*t^2
  const double expected_position_i = acceleration_i() * 0.5 * std::pow(num_measurements() / 2 * time_increment(), 2);
  const gtsam::Vector3 expected_position(expected_position_i, expected_position_i, expected_position_i);
  EXPECT_MATRIX_NEAR(imu_augmented_state.pose().translation(), expected_position, 1e-6);
}

TEST_F(ConstantAccelerationTest, AddAllMeasurementsWithAccelBias) {
  const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(acceleration(), gtsam::Vector3::Zero()), 0);
  lc::CombinedNavState imu_augmented_state = initial_state;
  imu_integrator().PimPredict(initial_state, imu_augmented_state);

  EXPECT_NEAR(imu_augmented_state.timestamp(), num_measurements() * time_increment(), 1e-6);
  EXPECT_MATRIX_NEAR(imu_augmented_state.velocity(), gtsam::Vector3::Zero(), 1e-6);
  EXPECT_MATRIX_NEAR(imu_augmented_state.pose().translation(), gtsam::Vector3::Zero(), 1e-6);
}

TEST_F(ConstantAngularVelocityTest, AddAllMeasurements) {
  const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(), 0);
  lc::CombinedNavState imu_augmented_state = initial_state;
  imu_integrator().PimPredict(initial_state, imu_augmented_state);

  EXPECT_NEAR(imu_augmented_state.timestamp(), num_measurements() * time_increment(), 1e-6);
  gtsam::Rot3 expected_orientation =
    ii::IntegrateAngularVelocities(imu_measurements(), gtsam::Rot3::identity(), initial_state.timestamp());
  EXPECT_MATRIX_NEAR(imu_augmented_state.pose().rotation(), expected_orientation, 1e-6);
}

TEST_F(ConstantAngularVelocityTest, AddHalfOfMeasurements) {
  const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(), 0);
  const lc::Time imu_augmented_state_start_time = num_measurements() / 2 * time_increment();
  lc::CombinedNavState imu_augmented_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(), imu_augmented_state_start_time);
  imu_integrator().PimPredict(initial_state, imu_augmented_state);

  EXPECT_NEAR(imu_augmented_state.timestamp(), num_measurements() * time_increment(), 1e-6);
  gtsam::Rot3 expected_orientation =
    ii::IntegrateAngularVelocities(imu_measurements(), gtsam::Rot3::identity(), imu_augmented_state_start_time);
  EXPECT_MATRIX_NEAR(imu_augmented_state.pose().rotation(), expected_orientation, 1e-6);
}

TEST_F(ConstantAngularVelocityTest, AddAllMeasurementsWithAccelBias) {
  const lc::CombinedNavState initial_state(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(),
                                           gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(), angular_velocity()), 0);
  lc::CombinedNavState imu_augmented_state = initial_state;
  imu_integrator().PimPredict(initial_state, imu_augmented_state);

  EXPECT_NEAR(imu_augmented_state.timestamp(), num_measurements() * time_increment(), 1e-6);
  EXPECT_MATRIX_NEAR(imu_augmented_state.pose().rotation(), gtsam::Rot3::identity(), 1e-6);
}


/*TEST(ImuIntegratorTester, ConstantVelocity) {
  // TODO(rsoussan): add default imu integrator params! pass these!
  ii::ImuIntegrator imu_integrator;
  constexpr double kInitialVelocity = 0.1;
  // params.graph_initializer.global_V_body_start = Eigen::Vector3d(kInitialVelocity, 0, 0);
  constexpr int kNumIterations = 100;
  constexpr double kTimeDiff = 0.1;
  lc::Time time = 0.0;
  const Eigen::Vector3d relative_translation = kTimeDiff * params.graph_initializer.global_V_body_start;
  Eigen::Isometry3d current_pose = lc::EigenPose(params.graph_initializer.global_T_body_start);
  // Add initial zero acceleration value so the imu integrator has more than one measurement when the subsequent
  // measurement is added
  const lm::ImuMeasurement initial_zero_imu_measurement(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time);
  imu_integrator.AddImuMeasurement(initial_zero_imu_measurement);
  for (int i = 0; i < kNumIterations; ++i) {
    time += kTimeDiff;
    const lm::ImuMeasurement zero_imu_measurement(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time);
    imu_integrator.AddImuMeasurement(zero_imu_measurement);
    const Eigen::Isometry3d relative_pose = lc::Isometry3d(relative_translation, Eigen::Matrix3d::Identity());
    current_pose = current_pose * relative_pose;
    const lc::Time source_time = time - kTimeDiff;
    const lc::Time target_time = time;
    // const auto latest_combined_nav_state = graph_localizer.LatestCombinedNavState();
    // TODO(rsoussan): get predicted pose from imu integrator!!! (add this as a function???)
    ASSERT_TRUE(latest_combined_nav_state != boost::none);
    EXPECT_NEAR(latest_combined_nav_state->timestamp(), time, 1e-6);
    EXPECT_MATRIX_NEAR(latest_combined_nav_state->pose(), current_pose, 1e-5);
  }
}*/

/*TEST(ImuIntegratorTester, ConstantAcceleration) {
  auto params = gl::DefaultGraphLocalizerParams();
  // Use depth odometry factor adder since it can add relative pose factors
  params.factor.depth_odometry_adder = gl::DefaultDepthOdometryFactorAdderParams();
  gl::GraphLocalizer graph_localizer(params);
  constexpr int kNumIterations = 100;
  constexpr double kTimeDiff = 0.1;
  lc::Time time = 0.0;
  Eigen::Isometry3d current_pose = lc::EigenPose(params.graph_initializer.global_T_body_start);
  const Eigen::Vector3d acceleration(0.01, 0.02, 0.03);
  Eigen::Vector3d velocity = params.graph_initializer.global_V_body_start;
  // Add initial zero acceleration value so the imu integrator has more than one measurement when the subsequent
  // measurement is added
  const lm::ImuMeasurement zero_imu_measurement(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time);
  graph_localizer.AddImuMeasurement(zero_imu_measurement);
  for (int i = 0; i < kNumIterations; ++i) {
    time += kTimeDiff;
    const lm::ImuMeasurement imu_measurement(acceleration, Eigen::Vector3d::Zero(), time);
    graph_localizer.AddImuMeasurement(imu_measurement);
    const Eigen::Vector3d relative_translation = velocity * kTimeDiff + 0.5 * acceleration * kTimeDiff * kTimeDiff;
    velocity += acceleration * kTimeDiff;
    const Eigen::Isometry3d relative_pose = lc::Isometry3d(relative_translation, Eigen::Matrix3d::Identity());
    current_pose = current_pose * relative_pose;
    const lc::Time source_time = time - kTimeDiff;
    const lc::Time target_time = time;
    const lm::DepthOdometryMeasurement constant_acceleration_measurement =
      gl::DepthOdometryMeasurementFromPose(relative_pose, source_time, target_time);
    graph_localizer.AddDepthOdometryMeasurement(constant_acceleration_measurement);
    graph_localizer.Update();
    const auto latest_combined_nav_state = graph_localizer.LatestCombinedNavState();
    ASSERT_TRUE(latest_combined_nav_state != boost::none);
    EXPECT_NEAR(latest_combined_nav_state->timestamp(), time, 1e-6);
    EXPECT_MATRIX_NEAR(latest_combined_nav_state->pose(), current_pose, 1e-5);
  }
}

TEST(ImuIntegratorTester, ConstantAccelerationNonZeroBias) {
  const Eigen::Vector3d acceleration(0.01, 0.02, 0.03);
  const Eigen::Vector3d angular_velocity(0.04, 0.05, 0.06);
  const Eigen::Vector3d acceleration_bias = 0.5 * acceleration;
  const Eigen::Vector3d angular_velocity_bias = angular_velocity;
  const Eigen::Vector3d bias_corrected_acceleration = acceleration - acceleration_bias;
  const Eigen::Vector3d bias_corrected_angular_velocity = angular_velocity - angular_velocity_bias;
  auto params = gl::DefaultGraphLocalizerParams();
  params.graph_initializer.initial_imu_bias = gtsam::imuBias::ConstantBias(acceleration_bias, angular_velocity_bias);
  // Use depth odometry factor adder since it can add relative pose factors
  params.factor.depth_odometry_adder = gl::DefaultDepthOdometryFactorAdderParams();
  gl::GraphLocalizer graph_localizer(params);
  constexpr int kNumIterations = 100;
  constexpr double kTimeDiff = 0.1;
  lc::Time time = 0.0;
  Eigen::Isometry3d current_pose = lc::EigenPose(params.graph_initializer.global_T_body_start);
  Eigen::Vector3d velocity = params.graph_initializer.global_V_body_start;
  // Add initial zero imu value so the imu integrator has more than one measurement when the subsequent
  // measurement is added
  const lm::ImuMeasurement zero_imu_measurement(acceleration_bias, angular_velocity_bias, time);
  graph_localizer.AddImuMeasurement(zero_imu_measurement);
  for (int i = 0; i < kNumIterations; ++i) {
    time += kTimeDiff;
    const lm::ImuMeasurement imu_measurement(acceleration, angular_velocity, time);
    graph_localizer.AddImuMeasurement(imu_measurement);
    const Eigen::Vector3d relative_translation =
      velocity * kTimeDiff + 0.5 * bias_corrected_acceleration * kTimeDiff * kTimeDiff;
    velocity += bias_corrected_acceleration * kTimeDiff;
    const Eigen::Isometry3d relative_pose = lc::Isometry3d(relative_translation, Eigen::Matrix3d::Identity());
    current_pose = current_pose * relative_pose;
    const lc::Time source_time = time - kTimeDiff;
    const lc::Time target_time = time;
    const lm::DepthOdometryMeasurement constant_acceleration_measurement =
      gl::DepthOdometryMeasurementFromPose(relative_pose, source_time, target_time);
    graph_localizer.AddDepthOdometryMeasurement(constant_acceleration_measurement);
    graph_localizer.Update();
    const auto latest_combined_nav_state = graph_localizer.LatestCombinedNavState();
    ASSERT_TRUE(latest_combined_nav_state != boost::none);
    EXPECT_NEAR(latest_combined_nav_state->timestamp(), time, 1e-6);
    EXPECT_MATRIX_NEAR(latest_combined_nav_state->pose(), current_pose, 1e-5);
  }
}

TEST(ImuIntegratorTester, ConstantAccelerationConstantAngularVelocityNonZeroBias) {
  const Eigen::Vector3d acceleration(0.01, 0.02, 0.03);
  const Eigen::Vector3d angular_velocity(0.04, 0.05, 0.06);
  const Eigen::Vector3d acceleration_bias = 0.5 * acceleration;
  const Eigen::Vector3d angular_velocity_bias = 0.5 * angular_velocity;
  const Eigen::Vector3d bias_corrected_acceleration = acceleration - acceleration_bias;
  const Eigen::Vector3d bias_corrected_angular_velocity = angular_velocity - angular_velocity_bias;
  auto params = gl::DefaultGraphLocalizerParams();
  params.graph_initializer.initial_imu_bias = gtsam::imuBias::ConstantBias(acceleration_bias, angular_velocity_bias);
  // Use depth odometry factor adder since it can add relative pose factors
  params.factor.depth_odometry_adder = gl::DefaultDepthOdometryFactorAdderParams();
  gl::GraphLocalizer graph_localizer(params);
  constexpr int kNumIterations = 100;
  constexpr double kTimeDiff = 0.1;
  lc::Time time = 0.0;
  Eigen::Isometry3d current_pose = lc::EigenPose(params.graph_initializer.global_T_body_start);
  Eigen::Vector3d velocity = params.graph_initializer.global_V_body_start;
  // Add initial zero imu value so the imu integrator has more than one measurement when the subsequent
  // measurement is added
  const lm::ImuMeasurement zero_imu_measurement(acceleration_bias, angular_velocity_bias, time);
  graph_localizer.AddImuMeasurement(zero_imu_measurement);
  for (int i = 0; i < kNumIterations; ++i) {
    time += kTimeDiff;
    const lm::ImuMeasurement imu_measurement(acceleration, angular_velocity, time);
    graph_localizer.AddImuMeasurement(imu_measurement);
    const Eigen::Matrix3d relative_orientation =
      (gtsam::Rot3::Expmap(bias_corrected_angular_velocity * kTimeDiff)).matrix();
    const Eigen::Vector3d relative_translation =
      velocity * kTimeDiff + 0.5 * bias_corrected_acceleration * kTimeDiff * kTimeDiff;
    velocity += bias_corrected_acceleration * kTimeDiff;
    // Put velocity in new body frame after integrating accelerations
    velocity = relative_orientation.transpose() * velocity;
    const Eigen::Isometry3d relative_pose = lc::Isometry3d(relative_translation, relative_orientation);
    current_pose = current_pose * relative_pose;
    const lc::Time source_time = time - kTimeDiff;
    const lc::Time target_time = time;
    const lm::DepthOdometryMeasurement constant_acceleration_measurement =
      gl::DepthOdometryMeasurementFromPose(relative_pose, source_time, target_time);
    graph_localizer.AddDepthOdometryMeasurement(constant_acceleration_measurement);
    graph_localizer.Update();
    const auto latest_combined_nav_state = graph_localizer.LatestCombinedNavState();
    ASSERT_TRUE(latest_combined_nav_state != boost::none);
    EXPECT_NEAR(latest_combined_nav_state->timestamp(), time, 1e-6);
    EXPECT_MATRIX_NEAR(latest_combined_nav_state->pose(), current_pose, 1e-5);
  }
}*/

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
