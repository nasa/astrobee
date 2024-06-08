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

namespace factor_adders {
template <typename MeasurementType>
// MeasurementBasedFactorAdder that adds factors using single measurements at a time
// for measurements in a provided time range.
class SingleMeasurementBasedFactorAdder : public MeasurementBasedFactorAdder<MeasurementType> {
 public:
  explicit SingleMeasurementBasedFactorAdder(const FactorAdderParams& params);
  virtual ~SingleMeasurementBasedFactorAdder() = default;

 private:
  // Add factors for all measurements in valid time range to existing factor graph.
  // Calls AddFactors(measurement) for each measurement in range.
  // Subsequently removes measurements that are addable (for example, if a corresponding
  // node adder can create a node at it's timestamp) as determined by CanAddFactor().
  // Returns number of added factors.
  int AddMeasurementBasedFactors(const localization_common::Time oldest_allowed_time,
                                 const localization_common::Time newest_allowed_time,
                                 gtsam::NonlinearFactorGraph& factors) final;

  // Add factors given a single measurement.
  // Returns number of added factors.
  virtual int AddFactorsForSingleMeasurement(const MeasurementType& measurement,
                                             gtsam::NonlinearFactorGraph& factors) = 0;

  // Whether a factor can be added for the given measurement.
  virtual bool CanAddFactor(const MeasurementType& measurement) const = 0;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(MeasurementBasedFactorAdder<MeasurementType>);
  }
};

// Implementation
template <typename MeasurementType>
SingleMeasurementBasedFactorAdder<MeasurementType>::SingleMeasurementBasedFactorAdder(const FactorAdderParams& params)
    : MeasurementBasedFactorAdder<MeasurementType>(params) {}

template <typename MeasurementType>
int SingleMeasurementBasedFactorAdder<MeasurementType>::AddMeasurementBasedFactors(
  const localization_common::Time oldest_allowed_time, const localization_common::Time newest_allowed_time,
  gtsam::NonlinearFactorGraph& factors) {
  int num_added_factors = 0;
  this->ProcessMeasurements(
    oldest_allowed_time, newest_allowed_time,
    [this, &num_added_factors](const MeasurementType& measurement, gtsam::NonlinearFactorGraph& factors) {
      if (!CanAddFactor(measurement)) return false;
      num_added_factors += AddFactorsForSingleMeasurement(measurement, factors);
      return true;
    },
    factors);
  return num_added_factors;
}
}  // namespace factor_adders

#endif  // FACTOR_ADDERS_SINGLE_MEASUREMENT_BASED_FACTOR_ADDER_H_
