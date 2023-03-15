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
// FactorAdder that stores measurements in a measurement buffer
// and uses these to create factors given a time range.
class MeasurementBasedFactorAdder : public FactorAdder {
 public:
  explicit MeasurementBasedFactorAdder(const FactorAdderParams& params);
  virtual ~MeasurementBasedFactorAdder() = default;

  // Add measurement to measurement buffer.
  void AddMeasurement(const MeasurementType& measurement);

  // Remove old measurements from measurement buffer.
  void RemoveOldMeasurements(const localization_common::Time oldest_allowed_time);

 private:
  FactorAdderParams params_;
  TimestampedSet<MeasurementType> measurements_;
};

// Implementation
template <typename MeasurementType, bool SingleMeasurementPerFactor>
MeasurementBasedFactorAdder::MeasurementBasedFactorAdder(const FactorAdderParams& params) : FactorAdder(params) {}

template <typename MeasurementType, bool SingleMeasurementPerFactor>
void MeasurementBasedFactorAdder::AddMeasurement(const MeasurementType& measurement) {
  measurements_.Add(measurement);
}

template <typename MeasurementType, bool SingleMeasurementPerFactor>
void MeasurementBasedFactorAdder::RemoveOldMeasurements(const localization_common::Time oldest_allowed_timestamp) {
  measurements_.RemoveOldValues(oldest_allowed_timestamp);
}
}  // namespace factor_adder

#endif  // FACTOR_ADDERS_MEASUREMENT_BASED_FACTOR_ADDER_H_
