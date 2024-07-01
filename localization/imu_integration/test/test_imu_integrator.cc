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
#include <imu_integration/test_utilities.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/imu_measurement.h>

#include <gtest/gtest.h>

namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;

namespace {
// Defaults to random start pose and velocity and zero IMU/IMU bias values
struct TestParams {
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerometer_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyroscope_bias = Eigen::Vector3d::Zero();
  gtsam::Pose3 body_T_sensor = gtsam::Pose3::identity();
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  gtsam::Pose3 initial_pose = lc::RandomPose();
  Eigen::Vector3d initial_velocity = lc::RandomVector3d();
  double integration_start_time = 0;
};

Eigen::Vector3d AccelerationOnlyIntegratedPosition(const TestParams& params, const double duration) {
  return params.initial_pose.translation() + params.initial_velocity * duration +
         params.initial_pose.rotation() * (params.acceleration - params.accelerometer_bias) * 0.5 *
           std::pow(duration, 2);
}

Eigen::Vector3d AccelerationOnlyIntegratedVelocity(const TestParams& params, const double duration) {
  return params.initial_velocity +
         params.initial_pose.rotation() * (params.acceleration - params.accelerometer_bias) * duration;
}
}  // namespace

class ConstantIMUTest : public ::testing::Test {
 public:
  // Start at time increment so first IMU measurement is after starting combined nav state time
  ConstantIMUTest() : time_increment_(1.0 / 125.0), start_time_(time_increment_), num_measurements_(20) {
    const ii::ImuIntegratorParams params = ii::DefaultImuIntegratorParams();
    imu_integrator_.reset(new ii::ImuIntegrator(params));
  }

  void SetUp() final {}

  void SetAndAddMeasurements(const Eigen::Vector3d& acceleration, const Eigen::Vector3d& angular_velocity) {
    acceleration_ = acceleration;
    angular_velocity_ = angular_velocity;
    imu_measurements_ =
      ii::ConstantMeasurements(acceleration_, angular_velocity_, num_measurements_, start_time_, time_increment_);
    for (const auto& imu_measurement : imu_measurements_) {
      imu_integrator_->AddImuMeasurement(imu_measurement);
    }
  }

  void SetAndAddMeasurements(const double acceleration = 0, const double angular_velocity = 0) {
    SetAndAddMeasurements(Eigen::Vector3d(acceleration, acceleration, acceleration),
                          Eigen::Vector3d(angular_velocity, angular_velocity, angular_velocity));
  }

  void TestAccelerationOnly(const TestParams& params) {
    SetAndAddMeasurements(params.acceleration, params.angular_velocity);
    const lc::CombinedNavState initial_state(
      params.initial_pose, params.initial_velocity,
      gtsam::imuBias::ConstantBias(params.accelerometer_bias, params.gyroscope_bias), params.integration_start_time);
    const auto imu_augmented_state = imu_integrator().ExtrapolateLatest(initial_state);
    ASSERT_TRUE(imu_augmented_state);
    const double duration = Duration() - params.integration_start_time;
    EXPECT_NEAR(imu_augmented_state->timestamp(), Duration(), 1e-6);
    const Eigen::Vector3d expected_velocity = AccelerationOnlyIntegratedVelocity(params, duration);
    EXPECT_MATRIX_NEAR(imu_augmented_state->velocity(), expected_velocity, 1e-6);
    const Eigen::Vector3d expected_position = AccelerationOnlyIntegratedPosition(params, duration);
    EXPECT_MATRIX_NEAR(imu_augmented_state->pose().translation(), expected_position, 1e-6);
    EXPECT_MATRIX_NEAR(imu_augmented_state->bias().accelerometer(), initial_state.bias().accelerometer(), 1e-6);
    EXPECT_MATRIX_NEAR(imu_augmented_state->bias().gyroscope(), initial_state.bias().gyroscope(), 1e-6);
  }

  void Test(const TestParams& params) {
    SetAndAddMeasurements(params.acceleration, params.angular_velocity);
    const lc::CombinedNavState initial_state(
      params.initial_pose, params.initial_velocity,
      gtsam::imuBias::ConstantBias(params.accelerometer_bias, params.gyroscope_bias), params.integration_start_time);

    const auto imu_augmented_state = imu_integrator().ExtrapolateLatest(initial_state);
    ASSERT_TRUE(imu_augmented_state);

    const Eigen::Vector3d corrected_angular_velocity =
      params.body_T_sensor.rotation() * (params.angular_velocity - params.gyroscope_bias);
    const Eigen::Vector3d centrifugal_acceleration =
      corrected_angular_velocity.cross(corrected_angular_velocity.cross(params.body_T_sensor.translation()));
    const Eigen::Vector3d corrected_acceleration =
      params.body_T_sensor.rotation() * (params.acceleration - params.accelerometer_bias) - centrifugal_acceleration;
    Eigen::Vector3d velocity = params.initial_velocity;
    gtsam::Pose3 pose = params.initial_pose;
    for (const auto& imu_measurement : imu_measurements()) {
      if (imu_measurement.timestamp <= params.integration_start_time) continue;
      // Catch float equality
      if (std::abs(imu_measurement.timestamp - params.integration_start_time) < 1e-6) continue;
      const Eigen::Matrix3d relative_orientation =
        (gtsam::Rot3::Expmap(corrected_angular_velocity * time_increment())).matrix();
      const Eigen::Vector3d relative_velocity =
        pose.rotation() * (corrected_acceleration * time_increment()) + params.gravity * time_increment();
      // Convert velocity to body frame
      const Eigen::Vector3d relative_translation =
        pose.rotation().inverse() * (velocity * time_increment()) +

        pose.rotation().inverse() * (0.5 * params.gravity * time_increment() * time_increment()) +
        0.5 * corrected_acceleration * time_increment() * time_increment();
      velocity += relative_velocity;
      const Eigen::Isometry3d relative_pose = lc::Isometry3d(relative_translation, relative_orientation);
      pose = pose * lc::GtPose(relative_pose);
    }
    EXPECT_MATRIX_NEAR(imu_augmented_state->velocity(), velocity, 1e-6);
    EXPECT_MATRIX_NEAR(imu_augmented_state->pose(), pose, 1e-6);
  }

  ii::ImuIntegrator& imu_integrator() { return *imu_integrator_; }

  void SetImuIntegrator(const ii::ImuIntegratorParams& params) { imu_integrator_.reset(new ii::ImuIntegrator(params)); }

  const Eigen::Vector3d& acceleration() const { return acceleration_; }

  const Eigen::Vector3d& angular_velocity() const { return angular_velocity_; }

  const std::vector<lm::ImuMeasurement>& imu_measurements() const { return imu_measurements_; }

  double time_increment() const { return time_increment_; }

  double Duration() const { return num_measurements_ * time_increment_; }

  double start_time() const { return start_time_; }

  int num_measurements() const { return num_measurements_; }

 private:
  std::unique_ptr<ii::ImuIntegrator> imu_integrator_;
  Eigen::Vector3d acceleration_;
  Eigen::Vector3d angular_velocity_;
  std::vector<lm::ImuMeasurement> imu_measurements_;
  const double time_increment_;
  const lc::Time start_time_;
  const int num_measurements_;
};

TEST_F(ConstantIMUTest, ConstVelocity) {
  TestParams params;
  TestAccelerationOnly(params);
  Test(params);
}

TEST_F(ConstantIMUTest, ConstVelocityNonZeroAccBias) {
  TestParams params;
  params.accelerometer_bias = lc::RandomVector3d();
  TestAccelerationOnly(params);
  Test(params);
}

TEST_F(ConstantIMUTest, ConstAcc) {
  TestParams params;
  params.acceleration = lc::RandomVector3d();
  TestAccelerationOnly(params);
  Test(params);
}

TEST_F(ConstantIMUTest, ConstAccHalfMeasurements) {
  TestParams params;
  params.acceleration = lc::RandomVector3d();
  params.integration_start_time = Duration() / 2;
  TestAccelerationOnly(params);
  Test(params);
}

TEST_F(ConstantIMUTest, ConstAccNonZeroAccBias) {
  TestParams params;
  params.acceleration = lc::RandomVector3d();
  params.accelerometer_bias = lc::RandomVector3d();
  TestAccelerationOnly(params);
  Test(params);
}

TEST_F(ConstantIMUTest, ConstAngularVelocity) {
  TestParams params;
  params.angular_velocity = lc::RandomVector3d();
  Test(params);
}

TEST_F(ConstantIMUTest, ConstAngularVelocityNonZeroAngBias) {
  TestParams params;
  params.angular_velocity = lc::RandomVector3d();
  params.gyroscope_bias = lc::RandomVector3d();
  Test(params);
}

TEST_F(ConstantIMUTest, ConstAccelerationAngularVelocity) {
  TestParams params;
  params.acceleration = lc::RandomVector3d();
  params.angular_velocity = lc::RandomVector3d();
  Test(params);
}

TEST_F(ConstantIMUTest, ConstAccelerationAngularVelocityGravity) {
  TestParams params;
  params.acceleration = lc::RandomVector3d();
  params.angular_velocity = lc::RandomVector3d();
  params.gravity = lc::RandomVector3d();
  auto integrator_params = ii::DefaultImuIntegratorParams();
  integrator_params.gravity = params.gravity;
  SetImuIntegrator(integrator_params);
  Test(params);
}

TEST_F(ConstantIMUTest, ConstAccelerationAngularVelocityGravitySensorOffset) {
  TestParams params;
  params.acceleration = lc::RandomVector3d();
  params.angular_velocity = lc::RandomVector3d();
  params.gravity = lc::RandomVector3d();
  params.body_T_sensor = lc::RandomPose();
  auto integrator_params = ii::DefaultImuIntegratorParams();
  integrator_params.gravity = params.gravity;
  integrator_params.body_T_imu = params.body_T_sensor;
  SetImuIntegrator(integrator_params);
  Test(params);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
