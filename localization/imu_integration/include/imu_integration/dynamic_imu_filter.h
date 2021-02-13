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

#ifndef IMU_INTEGRATION_DYNAMIC_IMU_FILTER_H_
#define IMU_INTEGRATION_DYNAMIC_IMU_FILTER_H_

#include <imu_integration/filter.h>
#include <imu_integration/imu_filter.h>
#include <imu_integration/imu_filter_params.h>
#include <localization_measurements/imu_measurement.h>

#include <boost/optional.hpp>

#include <memory>

namespace imu_integration {
enum class FanSpeedMode {
  kQuiet,    // 2000 rpm
  kNominal,  // 2500 rpm
  kFast      // 2800 rpm
};

class DynamicImuFilter {
 public:
  DynamicImuFilter();
  // Returns filtered measurement if one is available
  boost::optional<localization_measurements::ImuMeasurement> AddMeasurement(
    const localization_measurements::ImuMeasurement& imu_measurement);

  // Notch filter depends on fan speed, change filter as fan speed changes
  void SetMode(const FanSpeedMode fan_speed_mode);

 private:
  // Acceleration Filters
  std::unique_ptr<Filter> acceleration_x_filter_;
  std::unique_ptr<Filter> acceleration_y_filter_;
  std::unique_ptr<Filter> acceleration_z_filter_;
  // Angular Velocity Filters
  std::unique_ptr<Filter> angular_velocity_x_filter_;
  std::unique_ptr<Filter> angular_velocity_y_filter_;
  std::unique_ptr<Filter> angular_velocity_z_filter_;
  FanSpeedMode fan_speed_mode_;
};
}  // namespace imu_integration

#endif  // IMU_INTEGRATION_DYNAMIC_IMU_FILTER_H_
