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

#ifndef IMU_INTEGRATION_LATEST_IMU_INTEGRATOR_H_
#define IMU_INTEGRATION_LATEST_IMU_INTEGRATOR_H_

#include <imu_integration/imu_integrator.h>

#include <memory>

namespace imu_integration {
// Adds functionality to imu integrator to integrate latest measurements, meaning it keeps track of
// last integrated measurement and only integrates measurements more recent than that measurement.
class LatestImuIntegrator : public ImuIntegrator {
 public:
  LatestImuIntegrator(const Eigen::Isometry3d& body_T_imu, const Eigen::Vector3d& gyro_bias,
                      const Eigen::Vector3d& accelerometer_bias, const localization_common::Time start_time,
                      const Eigen::Vector3d& gravity);

  const gtsam::PreintegratedCombinedMeasurements& pim() const;

  void ResetPimIntegrationAndSetBias(const gtsam::imuBias::ConstantBias& bias);

  // Integrates all imu measurements that have not been added up to end_time.
  bool IntegrateLatestImuMeasurements(const localization_common::Time end_time);

 private:
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> pim_;
  localization_common::Time start_time_;
  localization_common::Time last_added_imu_measurement_time_;
};
}  // namespace imu_integration

#endif  // IMU_INTEGRATION_LATEST_IMU_INTEGRATOR_H_
