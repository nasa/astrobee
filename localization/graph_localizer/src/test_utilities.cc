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
#include <graph_localizer/test_utilities.h>
#include <localization_common/test_utilities.h>

#include <gtsam/geometry/Point3.h>

namespace graph_localizer {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace go = graph_optimizer;

localization_measurements::Plane RandomPlane() {
  gtsam::Point3 point = lc::RandomPoint3d();
  gtsam::Vector3 normal = lc::RandomVector3d().normalized();
  return localization_measurements::Plane(point, normal);
}

lm::DepthOdometryMeasurement DepthOdometryMeasurementFromPose(const Eigen::Isometry3d& pose, const lc::Time source_time,
                                                              const lc::Time target_time) {
  lm::Odometry odometry;
  odometry.source_time = source_time;
  odometry.target_time = target_time;
  odometry.sensor_F_source_T_target.pose = pose;
  odometry.sensor_F_source_T_target.covariance = Eigen::Matrix<double, 6, 6>::Identity();
  odometry.body_F_source_T_target.pose = pose;
  odometry.body_F_source_T_target.covariance = Eigen::Matrix<double, 6, 6>::Identity();
  std::vector<Eigen::Vector3d> empty_points;
  lm::DepthCorrespondences correspondences(empty_points, empty_points);
  return lm::DepthOdometryMeasurement(odometry, correspondences, target_time);
}

CombinedNavStateGraphValuesParams DefaultCombinedNavStateGraphValuesParams() {
  CombinedNavStateGraphValuesParams params;
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
  params.graph_values = DefaultCombinedNavStateGraphValuesParams();
  params.threshold_bias_uncertainty = false;
  return params;
}

GraphInitializerParams DefaultGraphInitializerParams() {
  GraphInitializerParams params;
  params.global_T_body_start = gtsam::Pose3::identity();
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

GraphLocalizerParams DefaultGraphLocalizerParams() {
  GraphLocalizerParams params;
  params.combined_nav_state_node_updater = DefaultCombinedNavStateNodeUpdaterParams();
  params.graph_optimizer = DefaultGraphOptimizerParams();
  params.graph_initializer = DefaultGraphInitializerParams();
  params.huber_k = 1.345;
  return params;
}

DepthOdometryFactorAdderParams DefaultDepthOdometryFactorAdderParams() {
  DepthOdometryFactorAdderParams params;
  params.enabled = true;
  params.huber_k = 1.345;
  params.noise_scale = 1.0;
  params.use_points_between_factor = false;
  params.position_covariance_threshold = 100;
  params.orientation_covariance_threshold = 100;
  params.point_to_point_error_threshold = 100.0;
  params.pose_translation_norm_threshold = 100.0;
  params.max_num_points_between_factors = 100;
  params.body_T_sensor = gtsam::Pose3::identity();
  params.enabled = true;
  return params;
}
}  // namespace graph_localizer
