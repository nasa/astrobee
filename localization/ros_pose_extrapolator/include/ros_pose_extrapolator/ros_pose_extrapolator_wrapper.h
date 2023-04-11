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
#ifndef ROS_POSE_EXTRAPOLATOR_ROS_POSE_EXTRAPOLATOR_WRAPPER_H_
#define ROS_POSE_EXTRAPOLATOR_ROS_POSE_EXTRAPOLATOR_WRAPPER_H_

#include <ff_msgs/EkfState.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/GraphLocState.h>
#include <ff_msgs/GraphVIOState.h>
#include <imu_integration/imu_integrator.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <localization_common/pose_interpolater.h>
#include <localization_common/rate_timer.h>
#include <localization_measurements/imu_measurement.h>
#include <ros_pose_extrapolator/ros_pose_extrapolator_params.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <memory>
#include <string>
#include <utility>

namespace ros_pose_extrapolator {
// Extrapolates latest localization pose using VIO odometry and IMU measurements.
// No extrapolation is performed during standstill to avoid adding jittery IMU data.
class RosPoseExtrapolatorWrapper {
 public:
  explicit RosPoseExtrapolatorWrapper(const std::string& graph_config_path_prefix = "");

  explicit RosPoseExtrapolatorWrapper(const RosPoseExtrapolatorParams& params);

  // Updates latest loc pose with msg pose.
  void LocalizationStateCallback(const ff_msgs::GraphLocState& graph_loc_state_msg);

  // Updates odometry pose interpolator with latest VIO msg pose.
  // Also stores latest velocity and bias estimates for future creation of latest
  // combined nav state.
  void GraphVIOStateCallback(const ff_msgs::GraphVIOState& graph_vio_state_msg);

  // Updates IMU interpolator with latest IMU measurement.
  void ImuCallback(const sensor_msgs::Imu& imu_msg);

  // Updates IMU interpolator with latest flight mode to adjust IMU measurement filter appropriately.
  void FlightModeCallback(const ff_msgs::FlightMode& flight_mode);

  // Extrapolates latest loc pose using relative VIO odometry measurements and finally
  // interpolated IMU data for timestamps past the last VIO measurement.
  boost::optional<std::pair<localization_common::CombinedNavState, localization_common::CombinedNavStateCovariances>>
  LatestExtrapolatedStateAndCovariances();

  // Constructs EkfState msg (historically named, contains pose/velocity/bias and covariances)
  // using extrapolated loc pose and covariance.
  boost::optional<ff_msgs::EkfState> LatestExtrapolatedLocalizationMsg();

 private:
  // Initializes extrapolator based on provided params.
  void Initialize(const RosPoseExtrapolatorParams& params);

  // Returns if currently in standstill. Uses latest VIO measurement to determine this.
  bool standstill() const;

  std::unique_ptr<imu_integration::ImuIntegrator> imu_integrator_;
  boost::optional<ff_msgs::GraphLocState> latest_loc_msg_;
  boost::optional<ff_msgs::GraphVIOState> latest_vio_msg_;
  boost::optional<localization_common::CombinedNavState> latest_extrapolated_vio_state_;
  boost::optional<localization_common::CombinedNavStateCovariances> latest_vio_state_covariances_;
  boost::optional<gtsam::Pose3> world_T_odom_;
  std::unique_ptr<gtsam::TangentPreintegration> preintegration_helper_;
  RosPoseExtrapolatorParams params_;
  boost::optional<bool> standstill_;
  localization_common::RateTimer loc_state_timer_ = localization_common::RateTimer("Loc State Msg");
  localization_common::PoseInterpolater odom_interpolator_;
};
}  // namespace ros_pose_extrapolator
#endif  // ROS_POSE_EXTRAPOLATOR_ROS_POSE_EXTRAPOLATOR_WRAPPER_H_
