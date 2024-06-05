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
#include <depth_odometry/depth_odometry_wrapper.h>
#include <depth_odometry/image_features_with_known_correspondences_aligner_depth_odometry.h>
#include <depth_odometry/image_features_with_known_correspondences_aligner_depth_odometry_params.h>
#include <depth_odometry/parameter_reader.h>
#include <depth_odometry/point_to_plane_icp_depth_odometry.h>
#include <depth_odometry/utilities.h>
#include <ff_common/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>

#include <sensor_msgs/image_encodings.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

DepthOdometryWrapper::DepthOdometryWrapper(const std::string& config_prefix) {
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("transforms.config");
  config.AddFile("geometry.config");
  config.AddFile((config_prefix + "depth_odometry.config").c_str());
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  DepthOdometryWrapperParams params;
  LoadDepthOdometryWrapperParams(config, params);
  Initialize(params);
}

DepthOdometryWrapper::DepthOdometryWrapper(const DepthOdometryWrapperParams& params) { Initialize(params); }

void DepthOdometryWrapper::Initialize(const DepthOdometryWrapperParams& params) {
  params_ = params;
  if (params_.method == "icp") {
    depth_odometry_.reset(new PointToPlaneICPDepthOdometry(params.icp));
  } else if (params_.method == "image_feature") {
    depth_odometry_.reset(new ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry(params.image_features));
  } else {
    LogFatal("DepthOdometryWrapper: Invalid depth odometry method selected.");
  }

  point_cloud_buffer_ = localization_common::TimestampedSet<sensor_msgs::PointCloud2ConstPtr>(params.max_buffer_size);
  image_buffer_ = localization_common::TimestampedSet<sensor_msgs::ImageConstPtr>(params.max_buffer_size);
}

std::vector<ff_msgs::DepthOdometry> DepthOdometryWrapper::PointCloudCallback(
  const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
  point_cloud_buffer_.Add(lc::TimeFromHeader(point_cloud_msg->header), point_cloud_msg);
  return ProcessDepthImageIfAvailable();
}

std::vector<ff_msgs::DepthOdometry> DepthOdometryWrapper::ImageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
  image_buffer_.Add(lc::TimeFromHeader(image_msg->header), image_msg);
  return ProcessDepthImageIfAvailable();
}

std::vector<ff_msgs::DepthOdometry> DepthOdometryWrapper::ProcessDepthImageIfAvailable() {
  std::vector<lm::DepthImageMeasurement> depth_image_measurements;
  boost::optional<lc::Time> latest_added_point_cloud_msg_time;
  boost::optional<lc::Time> latest_added_image_msg_time;
  int added_depth_images = 0;
  // Point clouds and depth images for the same measurement arrive on different topics.
  // Correlate pairs of these if possible.
  // Only add up to max_depth_images per cycle.
  for (auto image_msg_it = image_buffer_.set().rbegin();
       image_msg_it != image_buffer_.set().rend() && added_depth_images < params_.max_depth_images; ++image_msg_it) {
    const auto image_msg_timestamp = image_msg_it->first;
    const auto point_cloud_msg = point_cloud_buffer_.Closest(image_msg_timestamp);
    if (point_cloud_msg &&
        std::abs(point_cloud_msg->timestamp - image_msg_timestamp) <= params_.max_image_and_point_cloud_time_diff) {
      const auto depth_image_measurement =
        lm::MakeDepthImageMeasurement(point_cloud_msg->value, image_msg_it->second, params_.haz_cam_A_haz_depth);
      if (!depth_image_measurement) {
        LogError("ProcessDepthImageIfAvailable: Failed to create depth image measurement.");
        continue;
      }
      depth_image_measurements.emplace_back(*depth_image_measurement);
      ++added_depth_images;
      if (!latest_added_point_cloud_msg_time) latest_added_point_cloud_msg_time = point_cloud_msg->timestamp;
      if (!latest_added_image_msg_time) latest_added_image_msg_time = image_msg_timestamp;
    }
  }

  // Remove any measuremets older than measurements used for depth image creation
  // Also remove measurements used for depth image creation
  if (latest_added_point_cloud_msg_time) {
    point_cloud_buffer_.RemoveOldValues(*latest_added_point_cloud_msg_time);
    point_cloud_buffer_.Remove(*latest_added_point_cloud_msg_time);
  }
  if (latest_added_image_msg_time) {
    image_buffer_.RemoveOldValues(*latest_added_image_msg_time);
    image_buffer_.Remove(*latest_added_image_msg_time);
  }

  std::vector<ff_msgs::DepthOdometry> depth_odometry_msgs;
  for (const auto& depth_image_measurement : depth_image_measurements) {
    timer_.Start();
    auto sensor_F_source_T_target = depth_odometry_->DepthImageCallback(depth_image_measurement);
    timer_.Stop();
    if (sensor_F_source_T_target) {
      const lc::PoseWithCovariance body_F_source_T_target = lc::FrameChangeRelativePoseWithCovariance(
        sensor_F_source_T_target->pose_with_covariance, params_.body_T_haz_cam);
      ff_msgs::DepthOdometry depth_odometry_msg =
        DepthOdometryMsg(*sensor_F_source_T_target, body_F_source_T_target, timer_.last_value());
      depth_odometry_msgs.emplace_back(depth_odometry_msg);
    }
  }
  return depth_odometry_msgs;
}
}  // namespace depth_odometry
