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

PointCloudMeasurement MakePointCloudMeasurement(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
  const lc::Time timestamp = lc::TimeFromHeader(point_cloud_msg->header);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*point_cloud_msg, *point_cloud);
  return PointCloudMeasurement(timestamp, point_cloud);
}

boost::optional<DepthImageMeasurement> MakeDepthImageMeasurement(
  const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg, const sensor_msgs::ImageConstPtr& image_msg,
  const Eigen::Affine3d image_A_depth_cam) {
  const auto timestamp = lc::TimeFromHeader(image_msg->header);
  // TODO(rsoussan): Unify image and point cloud conversion with other functions
  cv_bridge::CvImagePtr cv_image;
  try {
    cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    LogError("cv_bridge exception: " << e.what());
    return boost::none;
  }
  const auto& intensities = cv_image->image;

  pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*depth_cloud_msg, *depth_cloud);

  if (static_cast<int>(intensities.cols) != static_cast<int>(depth_cloud->width) ||
      static_cast<int>(intensities.rows) != static_cast<int>(depth_cloud->height)) {
    LogError("MakeDepthImageMeasurement: Image and Point Cloud dimensions do not match.");
    return boost::none;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr depth_cloud_with_intensities(new pcl::PointCloud<pcl::PointXYZI>());
  depth_cloud_with_intensities->width = intensities.cols;
  depth_cloud_with_intensities->height = intensities.rows;
  depth_cloud_with_intensities->points.resize(depth_cloud_with_intensities->width *
                                              depth_cloud_with_intensities->height);

  int index = 0;
  for (int row = 0; row < intensities.rows; ++row) {
    for (int col = 0; col < intensities.cols; ++col) {
      const auto& point = depth_cloud->points[index];
      const Eigen::Vector3d depth_cam_t_point(point.x, point.y, point.z);
      const Eigen::Vector3d image_t_point = image_A_depth_cam * depth_cam_t_point;
      depth_cloud_with_intensities->points[index].x = image_t_point.x();
      depth_cloud_with_intensities->points[index].y = image_t_point.y();
      depth_cloud_with_intensities->points[index].z = image_t_point.z();
      depth_cloud_with_intensities->points[index].intensity = static_cast<float>(intensities.at<ushort>(row, col));
      ++index;
    }
  }
  return DepthImageMeasurement(intensities, depth_cloud_with_intensities, timestamp);
}
}  // namespace localization_measurements
