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

#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>
#include <parameter_reader/imu_integration.h>
#include <parameter_reader/node_adders.h>

namespace parameter_reader {
namespace lc = localization_common;
namespace mc = msg_conversions;
namespace na = node_adders;

void LoadTimestampedNodeAdderModelParams(config_reader::ConfigReader& config,
                                         na::TimestampedNodeAdderModelParams& params, const std::string& prefix) {
  LOAD_PARAM(params.huber_k, config, prefix);
}

void LoadCombinedNavStateNodeAdderParams(config_reader::ConfigReader& config,
                                         na::CombinedNavStateNodeAdder::Params& params, const std::string& prefix) {
  // Note that starting measurement, timestamp, and nodes come from measurements.
  // Starting noise models should be loaded from params.
  LoadBaseTimestampedNodeAdderParams<lc::CombinedNavState>(config, params, prefix);
  const double starting_pose_translation_stddev = mc::LoadDouble(config, "starting_pose_translation_stddev", prefix);
  const double starting_pose_quaternion_stddev = mc::LoadDouble(config, "starting_pose_quaternion_stddev", prefix);
  const double starting_velocity_stddev = mc::LoadDouble(config, "starting_velocity_stddev", prefix);
  const double starting_accel_bias_stddev = mc::LoadDouble(config, "starting_accel_bias_stddev", prefix);
  const double starting_gyro_bias_stddev = mc::LoadDouble(config, "starting_gyro_bias_stddev", prefix);
  const gtsam::Vector6 pose_noise_sigmas((gtsam::Vector(6) << starting_pose_translation_stddev,
                                          starting_pose_translation_stddev, starting_pose_translation_stddev,
                                          starting_pose_quaternion_stddev, starting_pose_quaternion_stddev,
                                          starting_pose_quaternion_stddev)
                                           .finished());
  const gtsam::Vector3 velocity_noise_sigmas(
    (gtsam::Vector(3) << starting_velocity_stddev, starting_velocity_stddev, starting_velocity_stddev).finished());
  const gtsam::Vector6 bias_noise_sigmas((gtsam::Vector(6) << starting_accel_bias_stddev, starting_accel_bias_stddev,
                                          starting_accel_bias_stddev, starting_gyro_bias_stddev,
                                          starting_gyro_bias_stddev, starting_gyro_bias_stddev)
                                           .finished());
  const auto pose_noise = lc::Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_noise_sigmas)), params.huber_k);
  const auto velocity_noise = lc::Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_noise_sigmas)), params.huber_k);
  const auto bias_noise = lc::Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(bias_noise_sigmas)), params.huber_k);
  params.start_noise_models.emplace_back(pose_noise);
  params.start_noise_models.emplace_back(velocity_noise);
  params.start_noise_models.emplace_back(bias_noise);
}

void LoadCombinedNavStateNodeAdderModelParams(config_reader::ConfigReader& config,
                                              na::CombinedNavStateNodeAdderModelParams& params,
                                              const std::string& prefix) {
  // Don't use prefix for IMU integrator params since these come from their own
  // config file.
  LoadImuIntegratorParams(config, params.imu_integrator);
  LoadTimestampedNodeAdderModelParams(config, params, prefix);
}

void LoadPoseNodeAdderParams(config_reader::ConfigReader& config, na::PoseNodeAdderParams& params,
                             const std::string& prefix) {
  LOAD_PARAM(params.starting_prior_translation_stddev, config, prefix);
  LOAD_PARAM(params.starting_prior_quaternion_stddev, config, prefix);
  params.SetStartNoiseModels();
  LoadBaseTimestampedNodeAdderParams<gtsam::Pose3>(config, params, prefix);
  // Load start node from measurements
}
}  // namespace parameter_reader
