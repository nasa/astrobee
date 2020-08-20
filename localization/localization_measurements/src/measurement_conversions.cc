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

#include <localization_measurements/measurement_conversions.h>

namespace localization_measurements {
gtsam::Pose3 GtPose(const Eigen::Isometry3d& eigen_pose) {
  return gtsam::Pose3(gtsam::Rot3(eigen_pose.linear().matrix()), eigen_pose.translation());
  // return Eigen::Ref<const Eigen::MatrixXd>(eigen_pose.matrix());
}

MatchedProjectionsMeasurement MakeMatchedProjectionsMeasurement(const ff_msgs::VisualLandmarks& visual_landmarks) {
  MatchedProjectionsMeasurement matched_projections_measurement;
  matched_projections_measurement.matched_projections.reserve(visual_landmarks.landmarks.size());
  const Time timestamp = GetTime(visual_landmarks.header.stamp.sec, visual_landmarks.header.stamp.nsec);
  matched_projections_measurement.timestamp = timestamp;

  for (const auto& landmark : visual_landmarks.landmarks) {
    const ImagePoint image_point(landmark.u, landmark.v);
    const MapPoint map_point(landmark.x, landmark.y, landmark.z);
    matched_projections_measurement.matched_projections.emplace_back(image_point, map_point, timestamp);
  }

  return matched_projections_measurement;
}

void FrameChangeMatchedProjectionsMeasurement(MatchedProjectionsMeasurement& matched_projections_measurement,
                                              const gtsam::Pose3& b_T_a) {
  for (auto& matched_projection : matched_projections_measurement.matched_projections) {
    matched_projection.map_point = b_T_a * matched_projection.map_point;
  }
}

FeaturePointsMeasurement MakeFeaturePointsMeasurement(const ff_msgs::Feature2dArray& optical_flow_feature_points) {
  FeaturePointsMeasurement feature_points_measurement;
  feature_points_measurement.feature_points.reserve(optical_flow_feature_points.feature_array.size());
  Time timestamp = GetTime(optical_flow_feature_points.header.stamp.sec, optical_flow_feature_points.header.stamp.nsec);
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
}  // namespace localization_measurements
