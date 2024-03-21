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
#ifndef NODE_ADDERS_UTILITIES_H_
#define NODE_ADDERS_UTILITIES_H_

#include <localization_common/logger.h>
#include <localization_common/time.h>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace node_adders {
// Removes the FactorType factor in factors containing each of the provided keys if it exists
template <typename FactorType>
bool RemoveFactor(const gtsam::KeyVector& keys, gtsam::NonlinearFactorGraph& factors);

// Removes the FactorType factor in factors containing each of the keys for nodes at timestamp a and b if it exists.
template <typename FactorType, typename NodesType>
bool RemoveRelativeFactor(const localization_common::Time timestamp_a, const localization_common::Time timestamp_b,
                          const NodesType& nodes, gtsam::NonlinearFactorGraph& factors);

// Returns the covariance from a robust, gaussian shared noise model.
gtsam::Matrix Covariance(const gtsam::SharedNoiseModel& robust_gaussian_noise);

// Implementation
template <typename FactorType>
bool RemoveFactor(const gtsam::KeyVector& keys, gtsam::NonlinearFactorGraph& factors) {
  for (auto factor_it = factors.begin(); factor_it != factors.end(); ++factor_it) {
    if (!dynamic_cast<FactorType*>(factor_it->get())) continue;
    bool contains_keys = true;
    for (const auto& key : keys) {
      if ((*factor_it)->find(key) == std::end((*factor_it)->keys())) {
        contains_keys = false;
        continue;
      }
    }
    if (contains_keys) {
      factors.erase(factor_it);
      return true;
    }
  }
  return false;
}

template <typename FactorType, typename NodesType>
bool RemoveRelativeFactor(const localization_common::Time timestamp_a, const localization_common::Time timestamp_b,
                          const NodesType& nodes, gtsam::NonlinearFactorGraph& factors) {
  const auto keys_a = nodes.Keys(timestamp_a);
  if (keys_a.empty()) {
    LogError("RemoveRelativeFactor: Failed to get keys for timestamp_a.");
    return false;
  }

  const auto keys_b = nodes.Keys(timestamp_b);
  if (keys_b.empty()) {
    LogError("RemoveRelativeFactor: Failed to get keys for timestamp_b.");
    return false;
  }

  gtsam::KeyVector combined_keys = keys_a;
  combined_keys.insert(combined_keys.cend(), keys_b.cbegin(), keys_b.cend());
  return RemoveFactor<FactorType>(combined_keys, factors);
}
}  // namespace node_adders

#endif  // NODE_ADDERS_UTILITIES_H_
