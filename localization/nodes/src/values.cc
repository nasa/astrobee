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
#include <nodes/values.h>

namespace nodes {
Values::Values(std::shared_ptr<gtsam::Values> values) : values_(std::move(values)), latest_key_(0) {}

bool Values::Contains(const gtsam::Key& key) const { return values_->exists(key); }

bool Values::Remove(const gtsam::Key& key) {
  if (!Contains(key)) return false;
  values_->erase(key);
  return true;
}

bool Values::Remove(const gtsam::KeyVector& keys) {
  bool removed_value = false;
  for (const auto& key : keys) {
    removed_value = Remove(key) || removed_value;
  }
  return removed_value;
}

size_t Values::size() const { return values_->size(); }
}  // namespace nodes
