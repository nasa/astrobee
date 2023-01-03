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

#include <graph_optimizer/utilities.h>

namespace graph_optimizer {
gtsam::noiseModel::Robust::shared_ptr Robust(const gtsam::SharedNoiseModel& noise, const double huber_k) {
  return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(huber_k), noise);
}

gtsam::NonlinearFactorGraph RemoveOldFactors(const gtsam::KeyVector& old_keys, gtsam::NonlinearFactorGraph& graph) {
  gtsam::NonlinearFactorGraph removed_factors;
  if (old_keys.empty()) return removed_factors;

  for (auto factor_it = graph.begin(); factor_it != graph.end();) {
    bool found_key = false;
    for (const auto& key : old_keys) {
      if ((*factor_it)->find(key) != (*factor_it)->end()) {
        found_key = true;
        break;
      }
    }
    if (found_key) {
      removed_factors.push_back(*factor_it);
      factor_it = graph.erase(factor_it);
    } else {
      ++factor_it;
    }
  }

  return removed_factors;
}
}  // namespace graph_optimizer
