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

#ifndef LOCALIZATION_MEASUREMENTS_IMU_MEASUREMENT_H_
#define LOCALIZATION_MEASUREMENTS_IMU_MEASUREMENT_H_

#include <localization_common/time.h>
#include <localization_measurements/measurement.h>

#include <Eigen/Core>

#include <sensor_msgs/Imu.h>

namespace localization_measurements {
struct ImuMeasurement : public Measurement {
  explicit ImuMeasurement(const sensor_msgs::Imu& imu_msg) {
    acceleration.x() = imu_msg.linear_acceleration.x;
    acceleration.y() = imu_msg.linear_acceleration.y;
    acceleration.z() = imu_msg.linear_acceleration.z;
    angular_velocity.x() = imu_msg.angular_velocity.x;
    angular_velocity.y() = imu_msg.angular_velocity.y;
    angular_velocity.z() = imu_msg.angular_velocity.z;
    // Ros headers are stored as seconds and nanoseconds
    timestamp = imu_msg.header.stamp.sec + 1e-9 * imu_msg.header.stamp.nsec;
  }
  ImuMeasurement(const Eigen::Vector3d& acceleration, const Eigen::Vector3d& angular_velocity,
                 const localization_common::Time timestamp)
      : acceleration(acceleration), angular_velocity(angular_velocity), timestamp(timestamp) {}

  Eigen::Vector3d acceleration;
  Eigen::Vector3d angular_velocity;
};
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_IMU_MEASUREMENT_H_
