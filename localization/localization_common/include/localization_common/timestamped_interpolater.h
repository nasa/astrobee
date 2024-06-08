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

#ifndef LOCALIZATION_COMMON_TIMESTAMPED_INTERPOLATER_H_
#define LOCALIZATION_COMMON_TIMESTAMPED_INTERPOLATER_H_

#include <ff_common/eigen_vectors.h>
#include <localization_common/time.h>
#include <localization_common/timestamped_set.h>
#include <gtsam/base/Matrix.h>

#include <Eigen/Geometry>

#include <boost/optional.hpp>

#include <vector>

namespace localization_common {
// Empty default interpolater params
struct DefaultInterpolationParams {};

// Provides interpolation functions that extends a timestamp set for interpolatable objects.
// Optionally stores params to help with interpolation.
template <typename T, class ParamsT = DefaultInterpolationParams>
class TimestampedInterpolater : public TimestampedSet<T> {
 public:
  explicit TimestampedInterpolater(const boost::optional<int> max_size = boost::none);
  TimestampedInterpolater(const std::vector<Time>& timestamps, const std::vector<T>& objects,
                          const boost::optional<int> max_size = boost::none);

  // Returns the interpolated object (or exact object if timestamps match) at the provided timestamp.
  boost::optional<T> Interpolate(const Time timestamp) const;

  // Interpolates two objects given an alpha [0,1], where alpha = 0 should return object a,
  // alpha = 1 should return object b, and otherwise should return the appropriate combination
  // of object a and b.
  // This needs to be specialized
  T Interpolate(const T& a, const T& b, const double alpha) const;

  // Returns the relative object given timestamps a and b.
  boost::optional<T> Relative(const Time timestamp_a, const Time timestamp_b) const;

  // Computes a relative object given objects a and b.
  // This needs to be specialized.
  T Relative(const T& a, const T& b) const;

  // Accessor to params.
  ParamsT& params();

  // Const accessor to params.
  const ParamsT& params() const;

  mutable boost::optional<gtsam::Matrix> covariance_a_b;

 private:
  TimestampedSet<T> timestamped_objects_;
  ParamsT params_;
};

// Implementation
template <typename T, class ParamsT>
TimestampedInterpolater<T, ParamsT>::TimestampedInterpolater(const boost::optional<int> max_size)
    : TimestampedSet<T>(max_size) {}

template <typename T, class ParamsT>
TimestampedInterpolater<T, ParamsT>::TimestampedInterpolater(const std::vector<Time>& timestamps,
                                                             const std::vector<T>& objects,
                                                             const boost::optional<int> max_size)
    : TimestampedSet<T>(timestamps, objects, max_size) {}

template <typename T, class ParamsT>
boost::optional<T> TimestampedInterpolater<T, ParamsT>::Interpolate(const Time timestamp) const {
  const auto lower_and_upper_bound = TimestampedSet<T>::LowerAndUpperBound(timestamp);
  // Check if equal timestamp exists
  if (lower_and_upper_bound.second && lower_and_upper_bound.second->timestamp == timestamp)
    return lower_and_upper_bound.second->value;
  if (!lower_and_upper_bound.first || !lower_and_upper_bound.second) {
    LogDebug("Interpolate: Failed to get lower and upper bound timestamps for query timestamp " << timestamp << ".");
    return boost::none;
  }

  const Time lower_bound_time = lower_and_upper_bound.first->timestamp;
  const Time upper_bound_time = lower_and_upper_bound.second->timestamp;
  const double alpha = (timestamp - lower_bound_time) / (upper_bound_time - lower_bound_time);
  return Interpolate(lower_and_upper_bound.first->value, lower_and_upper_bound.second->value, alpha);
}

template <typename T, class ParamsT>
T TimestampedInterpolater<T, ParamsT>::Interpolate(const T& a, const T& b, const double alpha) const {
  static_assert(sizeof(T) == std::size_t(-1), "This needs to be specialized by template class.");
}

template <typename T, class ParamsT>
boost::optional<T> TimestampedInterpolater<T, ParamsT>::Relative(const Time timestamp_a, const Time timestamp_b) const {
  const auto a = Interpolate(timestamp_a);
  const auto b = Interpolate(timestamp_b);
  if (!a || !b) {
    LogDebug("Relative: Failed to get interpolated objects for query timestamps " << timestamp_a << " and "
                                                                                  << timestamp_b << ".");
    return boost::none;
  }

  return Relative(*a, *b);
}

template <typename T, class ParamsT>
T TimestampedInterpolater<T, ParamsT>::Relative(const T& a, const T& b) const {
  static_assert(sizeof(T) == std::size_t(-1), "This needs to be specialized by template class.");
}

template <typename T, class ParamsT>
ParamsT& TimestampedInterpolater<T, ParamsT>::params() {
  return params_;
}

template <typename T, class ParamsT>
const ParamsT& TimestampedInterpolater<T, ParamsT>::params() const {
  return params_;
}
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_TIMESTAMPED_INTERPOLATER_H_
