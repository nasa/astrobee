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

#ifndef OPTIMIZERS_OPTIMIZER_H_
#define OPTIMIZERS_OPTIMIZER_H_

#include <optimizers/optimizer_params.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/optional.hpp>

namespace optimizers {
// Base class optimizer.
class Optimizer {
 public:
  explicit Optimizer(const OptimizerParams& params);
  virtual ~Optimizer() = default;

  // Performs optimization using the provided factors and values and updates the values
  // with the optimized estimates.
  virtual bool Optimize(const gtsam::NonlinearFactorGraph& factors, gtsam::Values& values) = 0;

  // Returns the covariance matrix for the provided value's key if it exists.
  // By default uses the calculated marginals to compute the covariances.
  virtual boost::optional<gtsam::Matrix> Covariance(const gtsam::Key& key) const;

  // Calculates the covariance matrix wrt two nodes using the provided keys.
  // Requires a successful round of optimization to have been performed.
  boost::optional<gtsam::Matrix> Covariance(const gtsam::Key& key_a, const gtsam::Key& key_b) const;

  // Returns marginals if they have been calculated.
  boost::optional<const gtsam::Marginals&> marginals() const;

  // Returns number of optimization iterations used for most recent optimize call.
  virtual int iterations() const = 0;

 protected:
  // Calculates marginals.
  void CalculateMarginals(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values);

 private:
  // Set solver used for calculating marginals.
  void SetMarginalsFactorization();

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(marginals_);
    ar& BOOST_SERIALIZATION_NVP(marginals_factorization_);
  }

  OptimizerParams params_;
  boost::optional<gtsam::Marginals> marginals_;
  gtsam::Marginals::Factorization marginals_factorization_;
};
}  // namespace optimizers

#endif  // OPTIMIZERS_OPTIMIZER_H_
