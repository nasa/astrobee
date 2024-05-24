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

#ifndef OPTIMIZERS_NONLINEAR_OPTIMIZER_H_
#define OPTIMIZERS_NONLINEAR_OPTIMIZER_H_

#include <optimizers/nonlinear_optimizer_params.h>
#include <optimizers/optimizer.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/serialization/serialization.hpp>

namespace optimizers {
class NonlinearOptimizer : public Optimizer {
 public:
  explicit NonlinearOptimizer(const NonlinearOptimizerParams& params);

  // Default constructor for serialization only
  NonlinearOptimizer() = default;

  virtual ~NonlinearOptimizer() = default;

  // Performs Levenberg-Marquardt nonlinear optimization using GTSAM on the factor graph.
  bool Optimize(const gtsam::NonlinearFactorGraph& factors, gtsam::Values& values) final;

  int iterations() const final;

 private:
  // Set optimization params based on provided NonlinearOptimizerParams.
  void SetOptimizationParams();

  // Order nodes in the graph for keys that will be marginalized.
  // Allows for more efficient optimization.
  void SetOrdering(const gtsam::NonlinearFactorGraph& factors, const gtsam::KeyVector& old_keys);

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(levenberg_marquardt_params_);
  }

  NonlinearOptimizerParams params_;
  gtsam::LevenbergMarquardtParams levenberg_marquardt_params_;
  int iterations_;
};
}  // namespace optimizers

#endif  // OPTIMIZERS_NONLINEAR_OPTIMIZER_H_
