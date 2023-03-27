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

#ifndef GRAPH_FACTORS_CUMULATIVE_FACTOR_H_
#define GRAPH_FACTORS_CUMULATIVE_FACTOR_H_

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <unordered_set>

namespace gtsam {
// Base case for cumulative factors which have a variable-sized set of measurements and keys accumulated
// over time. When using with a sliding window graph, old measurements and keys should
// be removed instead of removing the entire factor.
class CumulativeFactor {
 public:
  virtual ~CumulativeFactor() = default;

  // Returns a copy of the factor with keys in keys_to_remove removed along with
  // any associated measurements.
  virtual boost::shared_ptr<NonlinearFactor> PrunedCopy(const std::unordered_set<gtsam::Key>& keys_to_remove) const = 0;
};
}  // namespace gtsam

#endif  // GRAPH_FACTORS_CUMULATIVE_FACTOR_H_
