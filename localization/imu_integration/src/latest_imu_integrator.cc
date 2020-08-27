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

#include <imu_integration/latest_imu_integrator.h>

#include <glog/logging.h>

namespace imu_integration {
namespace lc = localization_common;
namespace lm = localization_measurements;
LatestImuIntegrator::LatestImuIntegrator(const Eigen::Isometry3d& body_T_imu, const Eigen::Vector3d& gyro_bias,
                                         const Eigen::Vector3d& accelerometer_bias, const lc::Time start_time,
                                         const Eigen::Vector3d& gravity)
    : ImuIntegrator(body_T_imu, gravity), start_time_(start_time), last_added_imu_measurement_time_(0) {
  pim_.reset(new gtsam::PreintegratedCombinedMeasurements(pim_params()));
  ResetPimIntegrationAndSetBias(gtsam::imuBias::ConstantBias(accelerometer_bias, gyro_bias));
}

const gtsam::PreintegratedCombinedMeasurements& LatestImuIntegrator::pim() const { return *pim_; }

void LatestImuIntegrator::ResetPimIntegrationAndSetBias(const gtsam::imuBias::ConstantBias& bias) {
  pim_->resetIntegrationAndSetBias(bias);
}

bool LatestImuIntegrator::IntegrateLatestImuMeasurements(const lc::Time end_time) {
  if (Size() < 2) {
    LOG(ERROR) << "IntegrateLatestImuMeasurements: Less than 2 measurements "
                  "available.";
    return false;
  }

  if (last_added_imu_measurement_time_ == 0) {
    VLOG(2) << "IntegrateLatestImuMeasurements: Adding first imu measurement.";
    last_added_imu_measurement_time_ = start_time_;
  }

  const auto last_added_imu_measurement_time =
      IntegrateImuMeasurements(last_added_imu_measurement_time_, end_time, *pim_);
  if (!last_added_imu_measurement_time) {
    LOG(ERROR) << "IntegrateLatestImuMeasurements: Failed to integrate measurements.";
    return false;
  }

  last_added_imu_measurement_time_ = *last_added_imu_measurement_time;
  return true;
}
}  // namespace imu_integration
