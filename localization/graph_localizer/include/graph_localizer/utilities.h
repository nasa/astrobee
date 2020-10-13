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

#ifndef GRAPH_LOCALIZER_UTILITIES_H_
#define GRAPH_LOCALIZER_UTILITIES_H_

#include <ff_msgs/EkfState.h>
#include <ff_msgs/LocalizationGraph.h>
#include <ff_msgs/VisualLandmarks.h>
#include <graph_localizer/graph_loc_initialization.h>
#include <graph_localizer/graph_localizer.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <localization_measurements/feature_point.h>
#include <localization_measurements/imu_measurement.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <Eigen/Core>

#include <boost/optional.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>

#include <deque>
#include <string>
#include <vector>

namespace graph_localizer {
void EstimateAndSetImuBiases(const localization_measurements::ImuMeasurement& imu_measurement,
                             const int num_imu_measurements_per_bias_estimate,
                             std::vector<localization_measurements::ImuMeasurement>& imu_bias_measurements,
                             GraphLocInitialization& graph_loc_initialization);

void RemoveGravityFromBias(const gtsam::Vector3& global_F_gravity, const gtsam::Pose3& body_T_imu,
                           const gtsam::Pose3& global_T_body, gtsam::imuBias::ConstantBias& imu_bias);

bool ValidPointSet(const std::deque<localization_measurements::FeaturePoint>& points,
                   const double average_distance_from_mean, const double min_avg_distance_from_mean);

bool ShouldAddStandstillPrior(const double standstill_feature_tracks_average_distance_from_mean,
                              const int num_standstill_feature_tracks, const FactorParams& params);

double AverageDistanceFromMean(const std::deque<localization_measurements::FeaturePoint>& points);

bool ValidVLMsg(const ff_msgs::VisualLandmarks& visual_landmarks_msg);

ff_msgs::EkfState EkfStateMsg(const localization_common::CombinedNavState& combined_nav_state,
                              const Eigen::Vector3d& acceleration, const Eigen::Vector3d& angular_velocity,
                              const localization_common::CombinedNavStateCovariances& covariances,
                              const int num_optical_flow_features_in_last_measurement,
                              const int num_sparse_mapping_features_in_last_measurement, const bool estimating_bias,
                              const double position_log_det_threshold, const double orientation_log_det_threshold);

ff_msgs::LocalizationGraph GraphMsg(const GraphLocalizer& graph_localizer);

geometry_msgs::PoseStamped PoseMsg(const Eigen::Isometry3d& global_T_body, const std_msgs::Header& header);

geometry_msgs::PoseStamped PoseMsg(const Eigen::Isometry3d& global_T_body, const localization_common::Time time);

gtsam::noiseModel::Robust::shared_ptr Robust(const gtsam::SharedNoiseModel& noise);

}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_UTILITIES_H_
