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

#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/VisualLandmarks.h>
#include <localization_measurements/combined_nav_state.h>
#include <localization_measurements/combined_nav_state_covariances.h>
#include <localization_measurements/feature_points_measurement.h>
#include <localization_measurements/imu_measurement.h>
#include <localization_measurements/matched_projections_measurement.h>

#include <gtsam/geometry/Pose3.h>

#include <Eigen/Core>

namespace localization_measurements {
gtsam::Pose3 GtPose(const Eigen::Isometry3d &eigen_pose);

MatchedProjectionsMeasurement MakeMatchedProjectionsMeasurement(const ff_msgs::VisualLandmarks &visual_landmarks);

void FrameChangeMatchedProjectionsMeasurement(MatchedProjectionsMeasurement &matched_projections_measurement,
                                              const gtsam::Pose3 &b_T_a);

FeaturePointsMeasurement MakeFeaturePointsMeasurement(const ff_msgs::Feature2dArray &optical_flow_tracks);
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_MEASUREMENT_CONVERSIONS_H_
