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

#ifndef FACTOR_ADDERS_MEASUREMENT_BASED_FACTOR_ADDER_H_
#define FACTOR_ADDERS_MEASUREMENT_BASED_FACTOR_ADDER_H_

#include <factor_adders/factor_adder.h>
#include <localization_common/timestamped_set.h>

namespace factor_adder {
template <typename MeasurementType, bool SingleMeasurementPerFactor = true>
// Adds factors for a specific measurement type. Stores measurements in a measurement buffer.
// Optionally adds factors for a single measurement at a time
// if the SingleMeasurementPerFactor template param is enabled (in which case
// AddFactors(measurement) needs to be specialized),
// otherwise AddFactors(oldest_allowed_timestamp, newest_allowed_timestamp, factors) needs to be specialized.
class MeasurementBasedFactorAdder : public FactorAdder {
 public:
  explicit MeasurementBasedFactorAdder(const FactorAdderParams& params);
  virtual ~MeasurementBasedFactorAdder() = default;

  // Add measurement to measurement buffer.
  void AddMeasurement(const MeasurementType& measurement);

  // Add factors for all measurements in valid time range to existing factor graph.
  // Returns number of added factors.
  // Specialize AddFactors(measurement) function if SingleMeasurementPerFactor is true
  // to call AddFactors(measurement) for each measurement in range, otherwise
  // specialize this.
  int AddFactors(const localization_common::Time oldest_allowed_time,
                 const localization_common::Time newest_allowed_time, gtsam::NonlinearFactorGraph& factors) final;

  // Remove old measurements from measurement buffer.
  void RemoveOldMeasurements(const localization_common::Time oldest_allowed_time);

 protected:
  // Specialize this if SingleMeasurementPerFactor is true.
  virtual int AddFactors(const MeasurementType& measurement);

 private:
  FactorAdderParams params_;
  TimestampedSet<MeasurementType> measurements_;
};

// Implementation
template <typename MeasurementType, bool SingleMeasurementPerFactor>
MeasurementBasedFactorAdder::MeasurementBasedFactorAdder(const FactorAdderParams& params) : FactorAdder(params) {}

template <typename MeasurementType, bool SingleMeasurementPerFactor>
virtual int AddFactors(const localization_common::Time oldest_allowed_time,
                       const localization_common::Time newest_allowed_time, gtsam::NonlinearFactorGraph& factors) {
  static_assert(SingleMeasurementPerFactor,
                "This needs to be specialized if not using a single measurement per factor.");
  int num_added_factors = 0;
  const auto lower_and_upper_bound_its =
    measurements_.InRangeValues(oldest_allowed_timestamp, latest_allowed_timestamp);
  for (auto it = lower_and_upper_bound_its.first; i != lower_and_upper_bound_its.second; ++it) {
    num_added_factors += AddFactors(it->value);
  }

  return num_added_factors;
}

template <typename MeasurementType, bool SingleMeasurementPerFactor>
void MeasurementBasedFactorAdder::AddMeasurement(const MeasurementType& measurement) {
  measurements_.Add(measurement);
}

template <typename MeasurementType, bool SingleMeasurementPerFactor>
void MeasurementBasedFactorAdder::RemoveOldMeasurements(const localization_common::Time oldest_allowed_timestamp) {
  measurements_.RemoveOldValues(oldest_allowed_timestamp);
}

template <typename MeasurementType, bool SingleMeasurementPerFactor>
int MeasurementBasedFactorAdder::AddFactors(const MeasurementType& measurement) {
  static_assert(!SingleMeasurementPerFactor, "This needs to be specialized if using a single measurement per factor.");
}
}  // namespace factor_adder

#endif  // FACTOR_ADDERS_MEASUREMENT_BASED_FACTOR_ADDER_H_
