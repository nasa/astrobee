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
#include <localization_measurements/measurement_conversions.h>

namespace localization_measurements {
namespace lc = localization_common;
MatchedProjectionsMeasurement MakeMatchedProjectionsMeasurement(const ff_msgs::VisualLandmarks& visual_landmarks) {
  MatchedProjectionsMeasurement matched_projections_measurement;
  matched_projections_measurement.matched_projections.reserve(visual_landmarks.landmarks.size());
  const lc::Time timestamp = lc::TimeFromHeader(visual_landmarks.header);
  matched_projections_measurement.timestamp = timestamp;

  for (const auto& landmark : visual_landmarks.landmarks) {
    const ImagePoint image_point(landmark.u, landmark.v);
    const MapPoint map_point(landmark.x, landmark.y, landmark.z);
    matched_projections_measurement.matched_projections.emplace_back(image_point, map_point, timestamp);
  }

  matched_projections_measurement.global_T_cam = lc::PoseFromMsg(visual_landmarks.pose);

  return matched_projections_measurement;
}

HandrailPointsMeasurement MakeHandrailPointsMeasurement(const ff_msgs::DepthLandmarks& depth_landmarks,
                                                        const TimestampedHandrailPose& handrail_pose) {
  HandrailPointsMeasurement handrail_points_measurement;
  handrail_points_measurement.handrail_pose = handrail_pose;
  const lc::Time timestamp = lc::TimeFromHeader(depth_landmarks.header);
  handrail_points_measurement.timestamp = timestamp;

  // TODO(rsoussan): This is hardcoded in the handrail node. The line and plane points should really
  // be separate fields in the depth landmarks msg.  Remove this if that is updated.
  constexpr int num_line_points = 4;
  int index = 0;
  for (; index < num_line_points; ++index) {
    const auto& landmark = depth_landmarks.landmarks[index];
    handrail_points_measurement.sensor_t_line_points.emplace_back(landmark.u, landmark.v, landmark.w);
  }
  // The next 6 points are plane points
  for (; index < depth_landmarks.landmarks.size(); ++index) {
    const auto& landmark = depth_landmarks.landmarks[index];
    handrail_points_measurement.sensor_t_plane_points.emplace_back(landmark.u, landmark.v, landmark.w);
  }

  return handrail_points_measurement;
}

MatchedProjectionsMeasurement FrameChangeMatchedProjectionsMeasurement(
  const MatchedProjectionsMeasurement& matched_projections_measurement,
  const gtsam::Pose3& new_frame_T_measurement_origin) {
  auto frame_changed_measurement = matched_projections_measurement;
  for (auto& matched_projection : frame_changed_measurement.matched_projections) {
    matched_projection.map_point = new_frame_T_measurement_origin * matched_projection.map_point;
  }
  frame_changed_measurement.global_T_cam = new_frame_T_measurement_origin * frame_changed_measurement.global_T_cam;
  return frame_changed_measurement;
}

FeaturePointsMeasurement MakeFeaturePointsMeasurement(const ff_msgs::Feature2dArray& optical_flow_feature_points) {
  FeaturePointsMeasurement feature_points_measurement;
  feature_points_measurement.feature_points.reserve(optical_flow_feature_points.feature_array.size());
  lc::Time timestamp = lc::TimeFromHeader(optical_flow_feature_points.header);
  feature_points_measurement.timestamp = timestamp;
  // TODO(rsoussan): put this somewhere else?
  static int image_id = 0;
  ++image_id;

  for (const auto& feature : optical_flow_feature_points.feature_array) {
    feature_points_measurement.feature_points.emplace_back(
      FeaturePoint(feature.x, feature.y, image_id, feature.id, timestamp));
  }

  return feature_points_measurement;
}

FanSpeedMode ConvertFanSpeedMode(const uint8_t speed) {
  switch (speed) {
    case 0:
      return FanSpeedMode::kOff;
    case 1:
      return FanSpeedMode::kQuiet;
    case 2:
      return FanSpeedMode::kNominal;
    case 3:
      return FanSpeedMode::kAggressive;
  }
  // Shouldn't get here
  return FanSpeedMode::kOff;
}
}  // namespace localization_measurements
