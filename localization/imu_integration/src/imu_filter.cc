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
#include <imu_integration/imu_filter.h>
#include <imu_integration/utilities.h>
#include <localization_common/logger.h>

namespace imu_integration {
namespace lm = localization_measurements;
ImuFilter::ImuFilter(const ImuFilterParams& params) {
  // Default to nominal filters
  acceleration_x_filter_ = LoadFilter(params.nominal_accel);
  acceleration_y_filter_ = LoadFilter(params.nominal_accel);
  acceleration_z_filter_ = LoadFilter(params.nominal_accel);
  angular_velocity_x_filter_ = LoadFilter(params.nominal_ang_vel);
  angular_velocity_y_filter_ = LoadFilter(params.nominal_ang_vel);
  angular_velocity_z_filter_ = LoadFilter(params.nominal_ang_vel);
}

boost::optional<lm::ImuMeasurement> ImuFilter::AddMeasurement(const lm::ImuMeasurement& imu_measurement) {
  const double filtered_acceleration_x = acceleration_x_filter_->AddValue(imu_measurement.acceleration.x());
  const double filtered_acceleration_y = acceleration_y_filter_->AddValue(imu_measurement.acceleration.y());
  const double filtered_acceleration_z = acceleration_z_filter_->AddValue(imu_measurement.acceleration.z());
  const double filtered_angular_velocity_x = angular_velocity_x_filter_->AddValue(imu_measurement.angular_velocity.x());
  const double filtered_angular_velocity_y = angular_velocity_y_filter_->AddValue(imu_measurement.angular_velocity.y());
  const double filtered_angular_velocity_z = angular_velocity_z_filter_->AddValue(imu_measurement.angular_velocity.z());

  // Use original timestamp
  // TODO(rsoussan): incorporate phase delay into timestamp?
  auto filtered_imu_measurement = imu_measurement;
  filtered_imu_measurement.acceleration << filtered_acceleration_x, filtered_acceleration_y, filtered_acceleration_z;
  filtered_imu_measurement.angular_velocity << filtered_angular_velocity_x, filtered_angular_velocity_y,
    filtered_angular_velocity_z;
  return filtered_imu_measurement;
}
}  // namespace imu_integration
