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

#ifndef IMU_INTEGRATION_TEST_IMU_FILTER_H_
#define IMU_INTEGRATION_TEST_IMU_FILTER_H_

#include <imu_integration/imu_filter.h>
#include <localization_measurements/imu_measurement.h>

#include <glog/logging.h>

namespace imu_integration {
class TestImuFilter : public ImuFilter {
 public:
  TestImuFilter();
  // Returns filtered measurement if one is available
  boost::optional<localization_measurements::ImuMeasurement> AddMeasurement(
      const localization_measurements::ImuMeasurement& imu_measurement) final;
};
}  // namespace imu_integration

#endif  // IMU_INTEGRATION_TEST_IMU_FILTER_H_
