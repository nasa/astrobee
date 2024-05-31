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

#ifndef OPTIMIZERS_ISAM2_OPTIMIZER_H_
#define OPTIMIZERS_ISAM2_OPTIMIZER_H_

#include <optimizers/isam2_optimizer_params.h>
#include <optimizers/optimizer.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>

#include <boost/serialization/serialization.hpp>

#include <memory>

namespace optimizers {
class ISAM2Optimizer : public Optimizer {
 public:
  explicit ISAM2Optimizer(const ISAM2OptimizerParams& params);

  // Default constructor for serialization only
  ISAM2Optimizer() = default;

  virtual ~ISAM2Optimizer() = default;

  // Performs Levenberg-Marquardt nonlinear optimization using GTSAM on the factor graph.
  bool Optimize(const gtsam::NonlinearFactorGraph& factors, gtsam::Values& values) final;

  int iterations() const final;

 private:
  // Set optimization params based on provided ISAM2OptimizerParams.
  void SetOptimizationParams();

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(isam2_params_);
    ar& BOOST_SERIALIZATION_NVP(isam2_);
    ar& BOOST_SERIALIZATION_NVP(previous_factors_);
    ar& BOOST_SERIALIZATION_NVP(previous_values_);
  }

  ISAM2OptimizerParams params_;
  gtsam::ISAM2Params isam2_params_;
  std::unique_ptr<gtsam::ISAM2> isam2_;
  // Previous factors and values, stored since ISAM2 only expects new factors
  // and values to be added during optimization.
  gtsam::NonlinearFactorGraph previous_factors_;
  gtsam::Values previous_values_;
};
}  // namespace optimizers

#endif  // OPTIMIZERS_ISAM2_OPTIMIZER_H_
