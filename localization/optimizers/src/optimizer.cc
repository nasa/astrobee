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

#include <optimizers/optimizer.h>
#include <localization_common/logger.h>

namespace optimizers {

Optimizer::Optimizer(const OptimizerParams& params) : params_(params) { SetMarginalsFactorization(); }

boost::optional<gtsam::Matrix> Optimizer::Covariance(const gtsam::Key& key) const {
  if (!marginals_) return boost::none;
  try {
    return marginals_->marginalCovariance(key);
  } catch (...) {
    LogError("Covariance: Failed to get covariance for key " << key);
    return boost::none;
  }
}

boost::optional<gtsam::Matrix> Optimizer::Covariance(const gtsam::Key& key_a, const gtsam::Key& key_b) const {
  if (!marginals_) return boost::none;
  try {
    return marginals_->jointMarginalCovariance({key_a, key_b}).fullMatrix();
  } catch (...) {
    LogError("Covariance: Failed to get covariance between key " << key_a << " and " << key_b);
    return boost::none;
  }
}

void Optimizer::CalculateMarginals(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values) {
  try {
    marginals_ = gtsam::Marginals(factors, values, marginals_factorization_);
  } catch (gtsam::IndeterminantLinearSystemException) {
    LogError("Update: Indeterminant linear system error during computation of marginals.");
    marginals_ = boost::none;
  } catch (const std::exception& exception) {
    LogError("Update: Computing marginals failed. " + std::string(exception.what()));
    marginals_ = boost::none;
  } catch (...) {
    LogError("Update: Computing marginals failed.");
    marginals_ = boost::none;
  }
}

boost::optional<const gtsam::Marginals&> Optimizer::marginals() const {
  if (!marginals_) return boost::none;
  return *marginals_;
}

void Optimizer::SetMarginalsFactorization() {
  if (params_.marginals_factorization == "qr") {
    marginals_factorization_ = gtsam::Marginals::Factorization::QR;
  } else if (params_.marginals_factorization == "cholesky") {
    marginals_factorization_ = gtsam::Marginals::Factorization::CHOLESKY;
  } else {
    LogError("Optimizer: No marginals factorization entered, defaulting to qr.");
    marginals_factorization_ = gtsam::Marginals::Factorization::QR;
  }
}
}  // namespace optimizers
