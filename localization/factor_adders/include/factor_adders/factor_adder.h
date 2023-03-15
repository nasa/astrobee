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

#ifndef FACTOR_ADDERS_FACTOR_ADDER_H_
#define FACTOR_ADDERS_FACTOR_ADDER_H_

#include <factor_adders/factor_adder_params.h>

#include <vector>

namespace factor_adder {
// Adds factors to a graph. Base class for measurement-based factor adder.
class FactorAdder {
 public:
  explicit FactorAdder(const FactorAdderParams& params) : params_(params) {}
  virtual ~FactorAdder() = default;

  // Add factors in valid time range to existing factor graph.
  // Returns number of added factors.
  virtual int AddFactors(const localization_common::Time oldest_allowed_time,
                 const localization_common::Time newest_allowed_time, gtsam::NonlinearFactorGraph& factors) = 0;

 private:
  FactorAdderParams params_;
};
}  // namespace factor_adder

#endif  // FACTOR_ADDERS_FACTOR_ADDER_H_
