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
 public:
  void Add(const Time time, const MeasurementType& measurement) { measurements_.emplace(time, measurement); }

  boost::optional<MeasurementType> Get(const localization_common::Time time) {
    const auto measurement_it = measurements_.find(time);
    if (measurement_it == measurements_.end()) return boost::none;
    return measurement_it->second;
  }

  boost::optional<MeasurementType> GetNearby(const localization_common::Time time, const double tolerance) {
    const auto upper_bound_it = measurements_.lower_bound(time);
    if (upper_bound_it == measurements_.begin()) {
      if (ValidTime(upper_bound_it->first, time, tolerance)) {
        return upper_bound_it->second;
      } else {
        return boost::none;
      }
    }
    if (upper_bound_it == measurements_.end()) {
      const auto last_measurement_it = measurements_.rbegin();
      if (ValidTime(last_measurement_it->first, time, tolerance)) {
        return last_measurement_it->second;
      } else {
        return boost::none;
      }
    }

    const auto lower_bound_it = std::prev(upper_bound_it);
    const double lower_bound_time_diff = std::abs(lower_bound_it->first - time);
    const double upper_bound_time_diff = std::abs(upper_bound_it->first - time);
    const auto closest_measurement_it =
      lower_bound_time_diff <= upper_bound_time_diff ? lower_bound_it : upper_bound_it;
    if (ValidTime(closest_measurement_it->first, time, tolerance)) {
      return closest_measurement_it->second;
    }
    return boost::none;
  }

  void EraseUpTo(const Time oldest_allowed_time) {
    const auto measurement_it = measurements_.lower_bound(oldest_allowed_time);
    measurements_.erase(measurements_.begin(), measurement_it);
  }

  void EraseIncluding(const Time oldest_allowed_time) {
    const auto measurement_it = measurements_.upper_bound(oldest_allowed_time);
    measurements_.erase(measurements_.begin(), measurement_it);
  }

  const std::map<Time, MeasurementType>& measurements() { return measurements_; }

 private:
  bool ValidTime(const localization_common::Time time_a, const localization_common::Time time_b,
                 const double tolerance) {
    return std::abs(time_a - time_b) <= tolerance;
  }
  std::map<Time, MeasurementType> measurements_;
};
}  // namespace localization_common
#endif  // LOCALIZATION_COMMON_MEASUREMENT_BUFFER_H_
