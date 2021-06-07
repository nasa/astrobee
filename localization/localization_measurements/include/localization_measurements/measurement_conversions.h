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

#ifndef LOCALIZATION_MEASUREMENTS_MEASUREMENT_CONVERSIONS_H_
#define LOCALIZATION_MEASUREMENTS_MEASUREMENT_CONVERSIONS_H_

#include <ff_msgs/DepthLandmarks.h>
#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/VisualLandmarks.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <localization_measurements/fan_speed_mode.h>
#include <localization_measurements/feature_points_measurement.h>
#include <localization_measurements/handrail_points_measurement.h>
#include <localization_measurements/imu_measurement.h>
#include <localization_measurements/matched_projections_measurement.h>
#include <localization_measurements/timestamped_handrail_pose.h>

#include <Eigen/Core>

namespace localization_measurements {
MatchedProjectionsMeasurement MakeMatchedProjectionsMeasurement(const ff_msgs::VisualLandmarks& visual_landmarks);

HandrailPointsMeasurement MakeHandrailPointsMeasurement(const ff_msgs::DepthLandmarks& depth_landmarks,
                                                        const TimestampedHandrailPose& sensor_T_handrail);

MatchedProjectionsMeasurement FrameChangeMatchedProjectionsMeasurement(
  const MatchedProjectionsMeasurement& matched_projections_measurement,
  const gtsam::Pose3& new_frame_T_measurement_origin);

FeaturePointsMeasurement MakeFeaturePointsMeasurement(const ff_msgs::Feature2dArray& optical_flow_tracks);

FanSpeedMode ConvertFanSpeedMode(const uint8_t speed);
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_MEASUREMENT_CONVERSIONS_H_
