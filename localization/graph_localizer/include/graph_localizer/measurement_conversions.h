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

#ifndef GRAPH_LOCALIZER_MEASUREMENT_CONVERSIONS_H_
#define GRAPH_LOCALIZER_MEASUREMENT_CONVERSIONS_H_

#include <ff_msgs/EkfState.h>
#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/VisualLandmarks.h>
#include <graph_localizer/combined_nav_state.h>
#include <graph_localizer/combined_nav_state_covariances.h>
#include <graph_localizer/feature_points_measurement.h>
#include <graph_localizer/graph_localizer.h>
#include <graph_localizer/imu_measurement.h>
#include <graph_localizer/matched_projections_measurement.h>

#include <gtsam/geometry/Pose3.h>

#include <Eigen/Core>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Header.h>

namespace graph_localizer {
Eigen::Isometry3d EigenPose(const ff_msgs::VisualLandmarks& vl_features, const Eigen::Isometry3d& nav_cam_T_body);

MatchedProjectionsMeasurement MakeMatchedProjectionsMeasurement(const ff_msgs::VisualLandmarks& visual_landmarks);

void FrameChangeMatchedProjectionsMeasurement(MatchedProjectionsMeasurement& matched_projections_measurement,
                                              const gtsam::Pose3& b_T_a);

FeaturePointsMeasurement MakeFeaturePointsMeasurement(const ff_msgs::Feature2dArray& optical_flow_tracks);

ros::Time RosTimeFromHeader(const std_msgs::Header& header);

Time TimeFromHeader(const std_msgs::Header& header);

ff_msgs::EkfState EkfStateMsg(const CombinedNavState& combined_nav_state, const Eigen::Vector3d& acceleration,
                              const Eigen::Vector3d& angular_velocity, const CombinedNavStateCovariances& covariances);

geometry_msgs::PoseWithCovarianceStamped PoseMsg(const Eigen::Isometry3d& global_T_body,
                                                 const std_msgs::Header& header);

geometry_msgs::PoseWithCovarianceStamped LatestPoseMsg(const GraphLocalizer& graph_localizer);
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_MEASUREMENT_CONVERSIONS_H_
