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
#include <graph_vio/test_utilities.h>
#include <localization_common/test_utilities.h>

#include <gtsam/geometry/Point3.h>

namespace graph_vio {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace go = graph_optimizer;
namespace gv = graph_values;

gv::gv::CombinedNavStateGraphValuesParams Defaultgv::CombinedNavStateGraphValuesParams() {
  gv::gv::CombinedNavStateGraphValuesParams params;
  params.ideal_duration = 3;
  params.min_num_states = 3;
  params.max_num_states = 20;
  return params;
}

go::GraphOptimizerParams DefaultGraphOptimizerParams() {
  go::GraphOptimizerParams params;
  params.verbose = false;
  params.fatal_failures = false;
  params.log_on_destruction = false;
  params.print_factor_info = false;
  params.use_ceres_params = false;
  params.max_iterations = 4;
  params.marginals_factorization = "qr";
  params.add_marginal_factors = false;
  params.huber_k = 1.345;
  params.log_rate = 100;
  return params;
}

CombinedNavStateNodeUpdaterParams DefaultCombinedNavStateNodeUpdaterParams() {
  CombinedNavStateNodeUpdaterParams params;
  params.starting_prior_translation_stddev = 0.02;
  params.starting_prior_quaternion_stddev = 0.01;
  params.starting_prior_velocity_stddev = 0.01;
  params.starting_prior_accel_bias_stddev = 0.001;
  params.starting_prior_gyro_bias_stddev = 0.001;
  params.huber_k = 1.345;
  params.global_N_body_start =
    lc::CombinedNavState(gtsam::Pose3::identity(), gtsam::Velocity3::Zero(), gtsam::imuBias::ConstantBias(), 0.0);
  params.add_priors = true;
  params.graph_values = Defaultgv::CombinedNavStateGraphValuesParams();
  params.threshold_bias_uncertainty = false;
  return params;
}

GraphInitializerParams DefaultGraphInitializerParams() {
  GraphInitializerParams params;
  params.global_V_body_start = gtsam::Velocity3::Zero();
  params.num_bias_estimation_measurements = 1;
  params.start_time = 0;
  params.initial_imu_bias = gtsam::imuBias::ConstantBias();
  params.gravity = gtsam::Vector3::Zero();
  params.body_T_imu = gtsam::Pose3::identity();
  params.filter = imu_integration::ImuFilterParams();
  params.gyro_sigma = 0.001;
  params.accel_sigma = 0.001;
  params.accel_bias_sigma = 0.001;
  params.gyro_bias_sigma = 0.001;
  params.integration_variance = 0.001;
  params.bias_acc_omega_int = 0.001;
  return params;
}

GraphVIOParams DefaultGraphVIOParams() {
  GraphVIOParams params;
  params.combined_nav_state_node_updater = DefaultCombinedNavStateNodeUpdaterParams();
  params.graph_optimizer = DefaultGraphOptimizerParams();
  params.graph_initializer = DefaultGraphInitializerParams();
  params.huber_k = 1.345;
  return params;
}
}  // namespace graph_vio
