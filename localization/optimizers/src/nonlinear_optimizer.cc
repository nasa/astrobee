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

#include <optimizers/nonlinear_optimizer.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

namespace optimizers {
namespace lc = localization_common;

NonlinearOptimizer::NonlinearOptimizer(const NonlinearOptimizerParams& params) : Optimizer(params), params_(params) {
  SetOptimizationParams();
}

bool NonlinearOptimizer::Optimize(const gtsam::NonlinearFactorGraph& factors, gtsam::Values& values) {
  LogDebug("Optimize: Optimizing.");

  // Perform Levenberg Marquart nonlinear optimization.
  // Update the values with the newly optmized values.
  gtsam::LevenbergMarquardtOptimizer optimizer(factors, values, levenberg_marquardt_params_);

  try {
    values = optimizer.optimize();
    // Calculate marginals after optimizing so covariances and marginal factors
    // can be generated if desired.
    CalculateMarginals(factors, values);
    iterations_ = optimizer.iterations();
  } catch (gtsam::IndeterminantLinearSystemException) {
    LogError("Optimize: Graph optimization failed, indeterminant linear system.");
    return false;
  } catch (gtsam::InvalidNoiseModel) {
    LogError("Optimize: Graph optimization failed, invalid noise model.");
    return false;
  } catch (gtsam::InvalidMatrixBlock) {
    LogError("Optimize: Graph optimization failed, invalid matrix block.");
    return false;
  } catch (gtsam::InvalidDenseElimination) {
    LogError("Optimize: Graph optimization failed, invalid dense elimination.");
    return false;
  } catch (const std::exception& exception) {
    LogError("Optimize: Graph optimization failed, " << exception.what());
    return false;
  } catch (...) {
    LogError("Optimize: Graph optimization failed.");
    return false;
  }
  return true;
}

int NonlinearOptimizer::iterations() const { return iterations_; }

void NonlinearOptimizer::SetOptimizationParams() {
  // Initialize lm params
  if (params_.verbose) {
    levenberg_marquardt_params_.verbosityLM = gtsam::LevenbergMarquardtParams::VerbosityLM::TRYDELTA;
    levenberg_marquardt_params_.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::LINEAR;
  }
  if (params_.use_ceres_params) {
    gtsam::LevenbergMarquardtParams::SetCeresDefaults(&levenberg_marquardt_params_);
  }

  levenberg_marquardt_params_.maxIterations = params_.max_iterations;
  levenberg_marquardt_params_.orderingType = gtsam::Ordering::COLAMD;
}

void NonlinearOptimizer::SetOrdering(const gtsam::NonlinearFactorGraph& factors, const gtsam::KeyVector& old_keys) {
  const auto ordering = gtsam::Ordering::ColamdConstrainedFirst(factors, old_keys);
  levenberg_marquardt_params_.setOrdering(ordering);
}
}  // namespace optimizers
