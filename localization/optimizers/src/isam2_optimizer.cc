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

#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <optimizers/isam2_optimizer.h>

namespace optimizers {
namespace lc = localization_common;

ISAM2Optimizer::ISAM2Optimizer(const ISAM2OptimizerParams& params) : Optimizer(params), params_(params) {
  SetOptimizationParams();
  isam2_.reset(new gtsam::ISAM2(isam2_params_));
}

bool ISAM2Optimizer::Optimize(const gtsam::NonlinearFactorGraph& factors, gtsam::Values& values) {
  LogDebug("Optimize: Optimizing.");

  // Only add new factors. Assumes factors consist of previous_factors_ followed by
  // new factors.
  gtsam::NonlinearFactorGraph new_factors;
  for (int i = previous_factors_.size(); i < factors.size(); ++i) {
    new_factors.push_back(factors[i]);
  }

  // Only add new values.
  const boost::function<bool(gtsam::Key)> new_value = [this](gtsam::Key key) {
    return !(previous_values_.exists(key));
  };
  gtsam::Values::Filtered<gtsam::Value> new_values = values.filter(new_value);

  const auto result = isam2_->update(new_factors, new_values);
  // Update values based on latest linearization point
  values = isam2_->getLinearizationPoint();
  CalculateMarginals(factors, values);
  // Cache factors and values so they can be referenced for removing already existing
  // factors and values added to the isam2 optimizer
  previous_factors_ = factors;
  previous_values_ = values;
  return true;
}

int ISAM2Optimizer::iterations() const { return 1; }

void ISAM2Optimizer::SetOptimizationParams() {
  // TODO(rsoussan): Add more params ISAM2OptimizerParams to custom set gtsam::ISAM2Params
}
}  // namespace optimizers
