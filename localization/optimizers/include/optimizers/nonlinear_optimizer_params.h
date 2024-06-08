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
#ifndef OPTIMIZERS_NONLINEAR_OPTIMIZER_PARAMS_H_
#define OPTIMIZERS_NONLINEAR_OPTIMIZER_PARAMS_H_

#include <optimizers/optimizer_params.h>

namespace optimizers {
struct NonlinearOptimizerParams : public OptimizerParams {
  // Maximum optimization iterations to perform
  int max_iterations;
  // Verbose printing for Levenberg-Marquardt optimization
  bool verbose;
  // Use ceres-style params for nonlinear optimization
  bool use_ceres_params;
};
}  // namespace optimizers

#endif  // OPTIMIZERS_NONLINEAR_OPTIMIZER_PARAMS_H_
