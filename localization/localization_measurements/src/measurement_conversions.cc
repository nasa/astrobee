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

#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>
#include <msg_conversions/msg_conversions.h>

#include <gtsam/geometry/Point3.h>

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Point32.h>

namespace localization_measurements {
namespace lc = localization_common;
namespace mc = msg_conversions;
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

Plane MakeHandrailPlane(const gtsam::Pose3& world_T_handrail, const double distance_to_wall) {
  // Assumes plane normal is aligned with x-axis of handrail and distance to wall is the distance along the negative x
  // axis to the wall from the handrail
  const gtsam::Point3 handrail_t_handrail_plane_point(-1.0 * distance_to_wall, 0.0, 0.0);
  const gtsam::Point3 handrail_F_handrail_plane_normal(1.0, 0.0, 0.0);
  const gtsam::Point3 world_t_handrail_plane_point = world_T_handrail * handrail_t_handrail_plane_point;
  const gtsam::Vector3 world_F_handrail_plane_normal = world_T_handrail.rotation() * handrail_F_handrail_plane_normal;
  return Plane(world_t_handrail_plane_point, world_F_handrail_plane_normal);
}

std::pair<gtsam::Point3, gtsam::Point3> MakeHandrailEndpoints(const gtsam::Pose3& world_T_handrail,
                                                              const double length) {
  // Assumes handrail endpoints are on z axis and handrail z is the center of the handrail
  const gtsam::Point3 handrail_t_handrail_endpoint(0, 0, length / 2.0);
  const gtsam::Point3 world_F_handrail_t_handrail_endpoint = world_T_handrail.rotation() * handrail_t_handrail_endpoint;
  const gtsam::Point3 world_t_handrail_endpoint_a =
    world_T_handrail.translation() + world_F_handrail_t_handrail_endpoint;
  const gtsam::Point3 world_t_handrail_endpoint_b =
    world_T_handrail.translation() - world_F_handrail_t_handrail_endpoint;
  return std::make_pair(world_t_handrail_endpoint_a, world_t_handrail_endpoint_b);
}

HandrailPointsMeasurement MakeHandrailPointsMeasurement(const ff_msgs::DepthLandmarks& depth_landmarks,
                                                        const TimestampedHandrailPose& world_T_handrail) {
  HandrailPointsMeasurement handrail_points_measurement;
  handrail_points_measurement.world_T_handrail = world_T_handrail;
  handrail_points_measurement.world_T_handrail_plane =
    MakeHandrailPlane(world_T_handrail.pose, world_T_handrail.distance_to_wall);
  if (world_T_handrail.accurate_z_position) {
    handrail_points_measurement.world_t_handrail_endpoints =
      MakeHandrailEndpoints(world_T_handrail.pose, world_T_handrail.length);
  }
  const lc::Time timestamp = lc::TimeFromHeader(depth_landmarks.header);
  handrail_points_measurement.timestamp = timestamp;

  for (const auto& sensor_t_line_point : depth_landmarks.sensor_t_line_points) {
    handrail_points_measurement.sensor_t_line_points.emplace_back(
      mc::VectorFromMsg<gtsam::Point3, geometry_msgs::Point32>(sensor_t_line_point));
  }
  for (const auto& sensor_t_line_endpoint : depth_landmarks.sensor_t_line_endpoints) {
    handrail_points_measurement.sensor_t_line_endpoints.emplace_back(
      mc::VectorFromMsg<gtsam::Point3, geometry_msgs::Point>(sensor_t_line_endpoint));
  }
  for (const auto& sensor_t_plane_point : depth_landmarks.sensor_t_plane_points) {
    handrail_points_measurement.sensor_t_plane_points.emplace_back(
      mc::VectorFromMsg<gtsam::Point3, geometry_msgs::Point32>(sensor_t_plane_point));
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

boost::optional<ImageMeasurement> MakeImageMeasurement(const sensor_msgs::ImageConstPtr& image_msg,
                                                       const std::string& encoding) {
  cv_bridge::CvImagePtr cv_image;
  try {
    cv_image = cv_bridge::toCvCopy(image_msg, encoding);
  } catch (cv_bridge::Exception& e) {
    LogError("cv_bridge exception: " << e.what());
    return boost::none;
  }
  const auto timestamp = lc::TimeFromHeader(image_msg->header);
  return ImageMeasurement(cv_image->image, timestamp);
}

boost::optional<DepthImageMeasurement> MakeDepthImageMeasurement(const sensor_msgs::ImageConstPtr& depth_image_msg) {
  cv_bridge::CvImagePtr cv_depth_image;
  try {
    cv_depth_image = cv_bridge::toCvCopy(depth_image_msg, sensor_msgs::image_encodings::TYPE_32FC4);
  } catch (cv_bridge::Exception& e) {
    LogError("cv_bridge exception: " << e.what());
    return boost::none;
  }

  // Picoflexx organizes depth images into 4 channels: distance, amplitude, intensity, flags
  // Each of these are floats except flags, which is a uint32
  const auto& pico_depth_image = cv_depth_image->image;
  cv::Mat distance(pico_depth_image.rows, pico_depth_image.cols, CV_32FC1);
  cv::Mat amplitude(pico_depth_image.rows, pico_depth_image.cols, CV_32FC1);
  cv::Mat float_intensity(pico_depth_image.rows, pico_depth_image.cols, CV_32FC1);
  cv::Mat float_flags(pico_depth_image.rows, pico_depth_image.cols, CV_32FC1);
  cv::Mat channels[] = {distance, amplitude, float_intensity, float_flags};
  cv::split(pico_depth_image, channels);
  // Convert intensities to unsigned byte as this is expected for feature detectors
  cv::Mat intensity;
  float_intensity.convertTo(intensity, CV_8UC1);
  // TODO(rsoussan): Avoid intermediate float conversion?
  cv::Mat flags;
  float_flags.convertTo(flags, CV_32SC1);

  const auto timestamp = lc::TimeFromHeader(depth_image_msg->header);
  return DepthImageMeasurement(distance, amplitude, intensity, flags, timestamp);
}
}  // namespace localization_measurements
