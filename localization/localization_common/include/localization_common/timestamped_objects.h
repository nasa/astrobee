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

#ifndef LOCALIZATION_COMMON_TIMESTAMPED_OBJECTS_H_
#define LOCALIZATION_COMMON_TIMESTAMPED_OBJECTS_H_

#include <localization_common/time.h>

#include <boost/optional.hpp>
#include <boost/serialization/serialization.hpp>

#include <map>
#include <utility>
#include <vector>

namespace localization_common {
template <typename T>
class TimestampedObjects {
  using TimestampedObject = std::pair<localization_common::Timestamp, T>;

 public:
  TimestampedObjects();

  // Assumes objects indices have corresponding timestamps in timestamps vectors and each timestamp is unique.
  // TODO(rsoussan): test this!
  TimestampedObjects(const std::vector<T>& objects, const std::vector<localization_common>& timestamps);

  bool Add(const localization_common::Time timestamp, const T& object);

  bool Remove(const localization_common::Time timestamp);

  boost::optional<T> Get(const localization_common::Time timestamp) const;

  size_t size() const;

  bool empty() const;

  boost::optional<TimestampedObject> Oldest() const;

  boost::optional<TimestampedObject> Latest() const;

  // Return lower and upper bounds.  Equal values are set as upper bound only.
  std::pair<boost::optional<TimestampedObject>, boost::optional<TimestampedObject>> LowerAndUpperBound(
    const localization_common::Time timestamp) const;

  boost::optional<TimestampedObject> Closest(const localization_common::Time timestamp) const;

  boost::optional<TimestampedObject> LowerBoundOrEqual(const localization_common::Time timestamp) const;

  std::vector<localization_common::Time> Timestamps() const;

  double Duration() const;

  bool Contains(const localization_common::Time timestamp) const;

  std::vector<TimestampedObjects> OldObjects(const localization_common::Time oldest_allowed_timestamp) const;

  int RemoveOldObjects(const localization_common::Time oldest_allowed_timestamp);

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/);

  std::map<localization_common::Time, T> timestamp_object_map_;
};

// Implementation
template <typename T>
TimestampedObjects<T>::TimestampedObjects() {}

template <typename T>
TimestampedObjects<T>::TimestampedObjects(const std::vector<T>& objects,
                                          const std::vector<localization_common>& timestamps) {
  for (int i = 0; i < objects.size(); ++i) {
    Add(timestamps[i], objects[i]);
  }
}

template <typename T>
bool TimestampedObjects<T>::Add(const localization_common::Time timestamp, const T& object) {
  if (Contains(timestamp)) return false;
  timestamp_object_map_.emplace(timestamp, object);
  return true;
}

template <typename T>
bool TimestampedObjects<T>::Remove(const localization_common::Time timestamp) {
  if (!Contains(timestamp)) return false;
  timestamp_object_map_.erase(timestamp);
  return true;
}

template <typename T>
boost::optional<T> TimestampedObjects<T>::Get(const localization_common::Time timestamp) const {
  if (!Contains(timestamp)) return boost::none;
  return timestamp_object_map_.at(timestamp);
}

template <typename T>
size_t TimestampedObjects<T>::size() const {
  return timestamp_object_map_.size();
}

template <typename T>
bool TimestampedObjects<T>::empty() const {
  return timestamp_object_map_.empty();
}

template <typename T>
boost::optional<TimestampedObject> TimestampedObjects<T>::Oldest() const {
  if (empty()) {
    LogDebug("Oldest: No timestamps available.");
    return boost::none;
  }
  return timestamp_object_map_.cbegin();
}

template <typename T>
boost::optional<TimestampedObject> TimestampedObjects<T>::Latest() const {
  if (empty()) {
    LogDebug("Latest: No objects available.");
    return boost::none;
  }
  return timestamp_object_map_.crbegin();
}

template <typename T>
std::pair<boost::optional<TimestampedObject>, boost::optional<TimestampedObject>>
TimestampedObjects<T>::LowerAndUpperBound(const localization_common::Time timestamp) const {
  if (empty()) {
    LogDebug("LowerAndUpperBound: No timestamps available.");
    return {boost::none, boost::none};
  }

  // lower bound returns first it >= query, call this upper bound
  const auto upper_bound_it = timestamp_object_map_.lower_bound(timestamp);
  if (upper_bound_it == timestamp_object_map_.cend()) {
    LogDebug("LowerAndUpperBoundTimestamps: No upper bound timestamp exists.");
    const localization_common::Time lower_bound_time = (timestamp_object_map_.crbegin())->first;
    return {boost::optional<localization_common::Time>(lower_bound_time), boost::none};
  } else if (upper_bound_it == timestamp_object_map_.cbegin()) {
    LogDebug("LowerAndUpperBoundTimestamps: No lower bound timestamp exists.");
    return {boost::none, boost::optional<localization_common::Time>(upper_bound_it->first)};
  }
  const auto lower_bound_it = std::prev(upper_bound_it);

  return {*lower_bound_it, *upper_bound_it};
}

template <typename T>
boost::optional<TimestampedObject> TimestampedObjects<T>::LowerBoundOrEqual(
  const localization_common::Time timestamp) const {
  const auto lower_and_upper_bound_objects = LowerAndUpperBound(timestamp);
  if (!lower_and_upper_bound_timestamp.first && !lower_and_upper_bound_timestamp.second) {
    LogDebug("LowerBoundOrEqual: Failed to get lower or upper bound objects.");
    return boost::none;
  }

  // Only return upper bound timestamp if it is equal to timestamp
  if (lower_and_upper_bound_objects.second && lower_and_upper_bound_timestamp.second->first == timestamp) {
    return lower_and_upper_bound_timestamp.second;
  }

  return lower_and_upper_bound_timestamp.first;
}

template <typename T>
std::vector<localization_common::Time> TimestampedObjects<T>::Timestamps() const {
  std::vector<localization_common::Time> timestamps;
  for (const auto& timestamped_object : timestamp_object_map_) {
    timestamps.emplace_back(timestamped_object.first);
  }
  return timestamps;
}

template <typename T>
double TimestampedObjects<T>::Duration() const {
  if (empty()) return 0;
  return (Latest()->first - OldestTimestamp()->first);
}

template <typename T>
boost::optional<TimestampedObject> TimestampedObjects<T>::Closest(const localization_common::Time timestamp) const {
  if (empty()) {
    LogDebug("Closest: No objects available.");
    return boost::none;
  }

  const auto lower_and_upper_bound_objects = LowerAndUpperBound(timestamp);
  if (!lower_and_upper_bound_objects.first && !lower_and_upper_bound_objects.second) {
    LogDebug("Closest: Failed to get lower or upper bound objects.");
    return boost::none;
  }

  if (!lower_and_upper_bound_object.first) {
    return lower_and_upper_bound_timestamp.second;
  } else if (!lower_and_upper_bound_object.second) {
    return lower_and_upper_bound_timestamp.first;
  } else {
    const localization_common::Time lower_bound_timestamp = lower_and_upper_bound_object.first->first;
    const localization_common::Time upper_bound_timestamp = lower_and_upper_bound_object.second->first;
    const double upper_bound_dt = std::abs(timestamp - upper_bound_timestamp);
    const double lower_bound_dt = std::abs(timestamp - lower_bound_timestamp);
    return (upper_bound_dt < lower_bound_dt) ? lower_and_upper_bound_object.second : lower_and_upper_bound_object.first;
  }
}

template <typename T>
bool TimestampedObjects<T>::Contains(const localization_common::Time timestamp) const {
  return timestamp_object_map_.count(timestamp) > 0;
}

template <typename T>
std::vector<TimestampedObjects> TimestampedObjects<T>::OldObjects(
  const localization_common::Time oldest_allowed_timestamp) const {
  std::vector<TimestampedObjects> old_objects;
  for (const auto& timestamped_object : timestamp_object_map_) {
    if (timestamped_object.first >= oldest_allowed_timestamp) break;
    old_objects.emplace_back(timestamped_object);
  }

  return old_objects;
}

template <typename T>
int TimestampedObjects<T>::RemoveOldObjects(const localization_common::Time oldest_allowed_timestamp) {
  int num_removed_objects = 0;
  for (auto it = timestamp_object_map_.begin(); it != timestamped_map_.end();) {
    if (it->first >= oldest_allowed_timestamp) break;
    it = timestamp_object_map_.erase(it);
    ++num_removed_objects;
  }

  return num_removed_objects;
}

template <typename T>
template <class ARCHIVE>
void TimestampedObjects<T>::serialize(ARCHIVE& ar, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(timestamp_object_map_);
}
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_TIMESTAMPED_OBJECTS_H_
