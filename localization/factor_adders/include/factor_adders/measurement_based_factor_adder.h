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

namespace factor_adders {
template <typename MeasurementType>
// FactorAdder that stores measurements in a measurement buffer
// and uses these to create factors given a time range.
class MeasurementBasedFactorAdder : public FactorAdder {
 public:
  explicit MeasurementBasedFactorAdder(const FactorAdderParams& params);
  virtual ~MeasurementBasedFactorAdder() = default;

  // Adds factors then removes old measurements.
  // Requires implementation of AddMeasurementBasedFactors.
  int AddFactors(const localization_common::Time oldest_allowed_time,
                 const localization_common::Time newest_allowed_time, gtsam::NonlinearFactorGraph& factors) final;

  // Add measurement to measurement buffer.
  void AddMeasurement(const MeasurementType& measurement);

  // Remove old measurements from measurement buffer.
  void RemoveOldMeasurements(const localization_common::Time oldest_allowed_time);

 protected:
  // Helper function to process measurements in valid time range and remove processed ones
  // from the measurement buffer.
  // Removes the measurement if the process_measurement_function returns true.
  void ProcessMeasurements(
    const localization_common::Time oldest_allowed_time, const localization_common::Time newest_allowed_time,
    const std::function<bool(const MeasurementType&, gtsam::NonlinearFactorGraph&)>& process_measurement_function,
    gtsam::NonlinearFactorGraph& factors);

  // Wrapper for ProcessMeasurements that doesn't use factors.
  void ProcessMeasurements(const localization_common::Time oldest_allowed_time,
                           const localization_common::Time newest_allowed_time,
                           const std::function<bool(const MeasurementType&)>& process_measurement_function);

  localization_common::TimestampedSet<MeasurementType> measurements_;

 private:
  // Adds factors based on measurements.
  // Removes used measurements from measurement buffer.
  virtual int AddMeasurementBasedFactors(const localization_common::Time oldest_allowed_time,
                                         const localization_common::Time newest_allowed_time,
                                         gtsam::NonlinearFactorGraph& factors) = 0;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(FactorAdder);
    ar& BOOST_SERIALIZATION_NVP(measurements_);
  }
};

// Implementation
template <typename MeasurementType>
MeasurementBasedFactorAdder<MeasurementType>::MeasurementBasedFactorAdder(const FactorAdderParams& params)
    : FactorAdder(params) {}

template <typename MeasurementType>
int MeasurementBasedFactorAdder<MeasurementType>::AddFactors(const localization_common::Time oldest_allowed_time,
                                                             const localization_common::Time newest_allowed_time,
                                                             gtsam::NonlinearFactorGraph& factors) {
  const int num_added_factors = AddMeasurementBasedFactors(oldest_allowed_time, newest_allowed_time, factors);
  RemoveOldMeasurements(oldest_allowed_time);
  return num_added_factors;
}

template <typename MeasurementType>
void MeasurementBasedFactorAdder<MeasurementType>::AddMeasurement(const MeasurementType& measurement) {
  measurements_.Add(measurement.timestamp, measurement);
}

template <typename MeasurementType>
void MeasurementBasedFactorAdder<MeasurementType>::RemoveOldMeasurements(
  const localization_common::Time oldest_allowed_timestamp) {
  measurements_.RemoveOldValues(oldest_allowed_timestamp);
}

template <typename MeasurementType>
void MeasurementBasedFactorAdder<MeasurementType>::ProcessMeasurements(
  const localization_common::Time oldest_allowed_time, const localization_common::Time newest_allowed_time,
  const std::function<bool(const MeasurementType&, gtsam::NonlinearFactorGraph&)>& process_measurement_function,
  gtsam::NonlinearFactorGraph& factors) {
  auto& measurements = measurements_.set();
  for (auto it = measurements.begin(); it != measurements.end();) {
    const auto& measurement = it->second;
    const auto timestamp = it->first;
    if (timestamp >= oldest_allowed_time && timestamp <= newest_allowed_time &&
        process_measurement_function(measurement, factors)) {
      // Remove used measurements.
      it = measurements.erase(it);
    } else {
      ++it;
    }
  }
}

template <typename MeasurementType>
void MeasurementBasedFactorAdder<MeasurementType>::ProcessMeasurements(
  const localization_common::Time oldest_allowed_time, const localization_common::Time newest_allowed_time,
  const std::function<bool(const MeasurementType&)>& process_measurement_function) {
  gtsam::NonlinearFactorGraph dummy_factors;
  ProcessMeasurements(
    oldest_allowed_time, newest_allowed_time,
    [&process_measurement_function](const MeasurementType& measurement, gtsam::NonlinearFactorGraph& factors) {
      return process_measurement_function(measurement);
    },
    dummy_factors);
}
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_MEASUREMENT_BASED_FACTOR_ADDER_H_
