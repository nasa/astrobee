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
#ifndef IMU_INTEGRATION_IMU_INTEGRATOR_PARAMS_H_
#define IMU_INTEGRATION_IMU_INTEGRATOR_PARAMS_H_

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace imu_integration {
struct ImuIntegratorParams {
  gtsam::Vector3 gravity;
  gtsam::Pose3 body_T_imu;
  // From gtsam: Angular and velocity random walk expressed in degrees respectively m/s per sqrt(hr).
  double gyro_sigma;
  double accel_sigma;
  double accel_bias_sigma;
  double gyro_bias_sigma;
  double integration_variance;
  double bias_acc_omega_int;
};
}  // namespace imu_integration

#endif  // IMU_INTEGRATION_IMU_INTEGRATOR_PARAMS_H_
