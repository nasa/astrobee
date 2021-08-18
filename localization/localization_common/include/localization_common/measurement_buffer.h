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

#ifndef LOCALIZATION_COMMON_MEASUREMENT_BUFFER_H_
#define LOCALIZATION_COMMON_MEASUREMENT_BUFFER_H_

#include <localization_common/time.h>

namespace localization_common {
template <typename MeasurementType>
class MeasurementBuffer {
  void AddMeasurement(const Time time, const MeasurementType& measurement) { measurements_.emplace(time, measurement); }

  boost::optional<MeasurementType> GetMeasurement(const localization_common::Time time) {
    const auto measurement_it = measurements_.find(time);
    if (measurement_it == measurements_.end()) return boost::none;
    return measurement_it->second;
  }

  void ClearBuffer(const Time oldest_allowed_time) {
    const auto measurement_it = measurements_.find(oldest_allowed_time);
    if (measurement_it == measurements_.end()) return;
    measurements_.erase(measurements_.begin(), measurement_it);
  }

 private:
  std::map<Time, MeasurementType> measurements_;
};
}  // namespace localization_common
#endif  // LOCALIZATION_COMMON_MEASUREMENT_BUFFER_H_
