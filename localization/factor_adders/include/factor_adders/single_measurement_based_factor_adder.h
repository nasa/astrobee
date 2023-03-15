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

#ifndef FACTOR_ADDERS_SINGLE_MEASUREMENT_BASED_FACTOR_ADDER_H_
#define FACTOR_ADDERS_SINGLE_MEASUREMENT_BASED_FACTOR_ADDER_H_

#include <factor_adders/measurement_based_factor_adder.h>

namespace factor_adder {
template <typename MeasurementType>
// MeasurementBasedFactorAdder that adds factors using single measurements at a time
// for measurements in a provided time range.
class SingleMeasurementBasedFactorAdder : public MeasurementBasedFactorAdder<MeasurementType> {
 public:
  explicit SingleMeasurementBasedFactorAdder(const FactorAdderParams& params);
  virtual ~SingleMeasurementBasedFactorAdder() = default;

  // Add factors for all measurements in valid time range to existing factor graph.
  // Calls AddFactors(measurement) for each measurement in range.
  // Returns number of added factors.
  int AddMeasurementBasedFactors(const localization_common::Time oldest_allowed_time,
                 const localization_common::Time newest_allowed_time, gtsam::NonlinearFactorGraph& factors) final;

 protected:
  // Add factors given a single measurement.
  virtual int AddFactors(const MeasurementType& measurement, gtsam::NonlinearFactorGraph& factors) = 0;
};

// Implementation
template <typename MeasurementType>
SingleMeasurementBasedFactorAdder::SingleMeasurementBasedFactorAdder(const FactorAdderParams& params)
    : MeasurementBasedFactorAdder<MeasurementType>(params) {}

template <typename MeasurementType>
virtual int AddFactors(const localization_common::Time oldest_allowed_time,
                       const localization_common::Time newest_allowed_time, gtsam::NonlinearFactorGraph& factors) {
  int num_added_factors = 0;
  const auto lower_and_upper_bound_its =
    measurements_.InRangeValues(oldest_allowed_timestamp, latest_allowed_timestamp);
  for (auto it = lower_and_upper_bound_its.first; it != lower_and_upper_bound_its.second; ++it) {
    num_added_factors += AddFactors(it->second);
  }

  return num_added_factors;
}
}  // namespace factor_adder

#endif  // FACTOR_ADDERS_SINGLE_MEASUREMENT_BASED_FACTOR_ADDER_H_
