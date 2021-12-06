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
#include <ff_util/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>

#include <sensor_msgs/image_encodings.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

DepthOdometryWrapper::DepthOdometryWrapper() {
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("transforms.config");
  config.AddFile("geometry.config");
  config.AddFile("localization/depth_odometry.config");
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
  // Point clouds and depth images for the same measurement arrive on different topics.
  // Correlate pairs of these if possible.
  for (const auto& image_msg : image_buffer_.measurements()) {
    const auto image_msg_timestamp = image_msg.first;
    const auto point_cloud_msg =
      point_cloud_buffer_.GetNearby(image_msg_timestamp, params_.max_image_and_point_cloud_time_diff);
    if (point_cloud_msg) {
      const auto depth_image_measurement =
        lm::MakeDepthImageMeasurement(*point_cloud_msg, image_msg.second, params_.haz_cam_A_haz_depth);
      if (!depth_image_measurement) {
        LogError("ProcessDepthImageIfAvailable: Failed to create depth image measurement.");
        continue;
      }
      depth_image_measurements.emplace_back(*depth_image_measurement);
      latest_added_point_cloud_msg_time = lc::TimeFromHeader((*point_cloud_msg)->header);
      latest_added_image_msg_time = image_msg_timestamp;
    }
  }

  if (latest_added_point_cloud_msg_time) point_cloud_buffer_.EraseUpToAndIncluding(*latest_added_point_cloud_msg_time);
  if (latest_added_image_msg_time) image_buffer_.EraseUpToAndIncluding(*latest_added_image_msg_time);

  std::vector<ff_msgs::DepthOdometry> depth_odometry_msgs;
  for (const auto& depth_image_measurement : depth_image_measurements) {
    auto sensor_F_source_T_target = depth_odometry_->DepthImageCallback(depth_image_measurement);
    if (sensor_F_source_T_target) {
      const lc::PoseWithCovariance body_F_source_T_target = lc::FrameChangeRelativePoseWithCovariance(
        sensor_F_source_T_target->pose_with_covariance, params_.body_T_haz_cam);
      ff_msgs::DepthOdometry depth_odometry_msg = DepthOdometryMsg(*sensor_F_source_T_target, body_F_source_T_target);
      depth_odometry_msgs.emplace_back(depth_odometry_msg);
    }
  }
  return depth_odometry_msgs;
}
}  // namespace depth_odometry
