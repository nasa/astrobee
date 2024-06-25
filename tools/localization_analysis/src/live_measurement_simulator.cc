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

#include <localization_analysis/live_measurement_simulator.h>
#include <localization_analysis/utilities.h>
#include <localization_common/logger.h>

#include <image_transport/image_transport.h>

#include <vector>

namespace localization_analysis {
namespace lc = localization_common;
LiveMeasurementSimulator::LiveMeasurementSimulator(const LiveMeasurementSimulatorParams& params)
    : bag_(params.bag_name, rosbag::bagmode::Read),
      map_(params.map_file, true),
      map_feature_matcher_(&map_),
      params_(params),
      kImageTopic_(params.image_topic),
      imu_buffer_(params.imu),
      flight_mode_buffer_(params.flight_mode),
      depth_odometry_buffer_(params.depth_odometry),
      of_buffer_(params.of),
      vl_buffer_(params.vl),
      ar_buffer_(params.ar) {
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");
  config.AddFile("localization.config");
  config.AddFile("optical_flow.config");

  if (!config.ReadFiles()) {
    LogError("Failed to read config files.");
    exit(0);
  }

  map_feature_matcher_.ReadParams(config);
  optical_flow_tracker_.ReadParams(&config);
  std::vector<std::string> topics;
  topics.push_back(std::string("/") + TOPIC_HARDWARE_IMU);
  topics.push_back(TOPIC_HARDWARE_IMU);
  topics.push_back(std::string("/") + kImageTopic_);
  topics.push_back(kImageTopic_);
  if (params_.use_bag_image_feature_msgs) {
    topics.push_back(std::string("/") + TOPIC_LOCALIZATION_OF_FEATURES);
    topics.push_back(TOPIC_LOCALIZATION_OF_FEATURES);
    topics.push_back(std::string("/") + TOPIC_LOCALIZATION_ML_FEATURES);
    topics.push_back(TOPIC_LOCALIZATION_ML_FEATURES);
  }
  // Only use recorded ar features
  topics.push_back(std::string("/") + TOPIC_LOCALIZATION_AR_FEATURES);
  topics.push_back(TOPIC_LOCALIZATION_AR_FEATURES);

  topics.push_back(std::string("/") + TOPIC_LOCALIZATION_DEPTH_ODOM);
  topics.push_back(TOPIC_LOCALIZATION_DEPTH_ODOM);

  topic_localization_depth_image_ = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                    static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                    static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_EXTENDED) +
                                    static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_AMPLITUDE_IMAGE);
  topics.push_back(std::string("/") + topic_localization_depth_image_);
  topics.push_back(topic_localization_depth_image_);

  topic_localization_depth_cloud_ = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                    static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                    static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX);
  topics.push_back(std::string("/") + topic_localization_depth_cloud_);
  topics.push_back(topic_localization_depth_cloud_);

  topics.push_back(std::string("/") + TOPIC_MOBILITY_FLIGHT_MODE);
  topics.push_back(TOPIC_MOBILITY_FLIGHT_MODE);

  view_.reset(new rosbag::View(bag_, rosbag::TopicQuery(topics)));
  current_time_ = lc::TimeFromRosTime(view_->getBeginTime());
}

ff_msgs::Feature2dArray LiveMeasurementSimulator::GenerateOFFeatures(const sensor_msgs::ImageConstPtr& image_msg) {
  ff_msgs::Feature2dArray of_features;
  optical_flow_tracker_.OpticalFlow(image_msg, &of_features);
  return of_features;
}

bool LiveMeasurementSimulator::GenerateVLFeatures(const sensor_msgs::ImageConstPtr& image_msg,
                                                  ff_msgs::VisualLandmarks& vl_features) {
  // Convert image to cv image
  cv_bridge::CvImageConstPtr image;
  try {
    image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  if (!map_feature_matcher_.Localize(image, &vl_features)) return false;
  return true;
}

bool LiveMeasurementSimulator::ProcessMessage() {
  if (!view_it_)
    view_it_ = view_->begin();
  else
    ++(*view_it_);
  if (*view_it_ == view_->end()) return false;
  const auto& msg = **view_it_;
  current_time_ = lc::TimeFromRosTime(msg.getTime());
  /*if (string_ends_with(msg.getTopic(), TOPIC_MOBILITY_FLIGHT_MODE)) {
    const ff_msgs::FlightModeConstPtr flight_mode = msg.instantiate<ff_msgs::FlightMode>();
    flight_mode_buffer_.BufferMessage(*flight_mode);
  }*/
  if (string_ends_with(msg.getTopic(), TOPIC_HARDWARE_IMU)) {
    sensor_msgs::ImuConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
    imu_buffer_.BufferMessage(*imu_msg);
  } else if (params_.use_bag_depth_odom_msgs && string_ends_with(msg.getTopic(), TOPIC_LOCALIZATION_DEPTH_ODOM)) {
    const ff_msgs::DepthOdometryConstPtr depth_odometry = msg.instantiate<ff_msgs::DepthOdometry>();
    depth_odometry_buffer_.BufferMessage(*depth_odometry);
  } else if (!params_.use_bag_depth_odom_msgs && string_ends_with(msg.getTopic(), topic_localization_depth_image_)) {
    const sensor_msgs::ImageConstPtr depth_image = msg.instantiate<sensor_msgs::Image>();
    const auto depth_odometry_msgs = depth_odometry_wrapper_.ImageCallback(depth_image);
    for (const auto& depth_odometry_msg : depth_odometry_msgs) {
      depth_odometry_buffer_.BufferMessage(depth_odometry_msg);
    }
  } else if (!params_.use_bag_depth_odom_msgs && string_ends_with(msg.getTopic(), topic_localization_depth_cloud_)) {
    const sensor_msgs::PointCloud2ConstPtr depth_cloud = msg.instantiate<sensor_msgs::PointCloud2>();
    const auto depth_odometry_msgs = depth_odometry_wrapper_.PointCloudCallback(depth_cloud);
    for (const auto& depth_odometry_msg : depth_odometry_msgs) {
      depth_odometry_buffer_.BufferMessage(depth_odometry_msg);
    }
  } else if (string_ends_with(msg.getTopic(), TOPIC_LOCALIZATION_AR_FEATURES)) {
    // Always use ar features until have data with dock cam images
    const ff_msgs::VisualLandmarksConstPtr ar_features = msg.instantiate<ff_msgs::VisualLandmarks>();
    ar_buffer_.BufferMessage(*ar_features);
  } else if (params_.use_bag_image_feature_msgs && string_ends_with(msg.getTopic(), TOPIC_LOCALIZATION_OF_FEATURES)) {
    const ff_msgs::Feature2dArrayConstPtr of_features = msg.instantiate<ff_msgs::Feature2dArray>();
    of_buffer_.BufferMessage(*of_features);
  } else if (params_.use_bag_image_feature_msgs && string_ends_with(msg.getTopic(), TOPIC_LOCALIZATION_ML_FEATURES)) {
    const ff_msgs::VisualLandmarksConstPtr vl_features = msg.instantiate<ff_msgs::VisualLandmarks>();
    vl_buffer_.BufferMessage(*vl_features);
  } else if (string_ends_with(msg.getTopic(), kImageTopic_)) {
    sensor_msgs::ImageConstPtr image_msg = msg.instantiate<sensor_msgs::Image>();
    if (params_.save_optical_flow_images) {
      img_buffer_.emplace(localization_common::TimeFromHeader(image_msg->header), image_msg);
    }
    if (!params_.use_bag_image_feature_msgs) {
      const ff_msgs::Feature2dArray of_features = GenerateOFFeatures(image_msg);
      of_buffer_.BufferMessage(of_features);

      ff_msgs::VisualLandmarks vl_features;
      if (GenerateVLFeatures(image_msg, vl_features)) {
        vl_buffer_.BufferMessage(vl_features);
      }
    }
  }
  return true;
}

lc::Time LiveMeasurementSimulator::CurrentTime() { return current_time_; }

boost::optional<sensor_msgs::Imu> LiveMeasurementSimulator::GetImuMessage(const lc::Time current_time) {
  return imu_buffer_.GetMessage(current_time);
}
boost::optional<ff_msgs::FlightMode> LiveMeasurementSimulator::GetFlightModeMessage(const lc::Time current_time) {
  return flight_mode_buffer_.GetMessage(current_time);
}
boost::optional<ff_msgs::Feature2dArray> LiveMeasurementSimulator::GetOFMessage(const lc::Time current_time) {
  return of_buffer_.GetMessage(current_time);
}
boost::optional<ff_msgs::DepthOdometry> LiveMeasurementSimulator::GetDepthOdometryMessage(const lc::Time current_time) {
  return depth_odometry_buffer_.GetMessage(current_time);
}
boost::optional<ff_msgs::VisualLandmarks> LiveMeasurementSimulator::GetVLMessage(const lc::Time current_time) {
  return vl_buffer_.GetMessage(current_time);
}
boost::optional<ff_msgs::VisualLandmarks> LiveMeasurementSimulator::GetARMessage(const lc::Time current_time) {
  return ar_buffer_.GetMessage(current_time);
}
boost::optional<sensor_msgs::ImageConstPtr> LiveMeasurementSimulator::GetImageMessage(const lc::Time current_time) {
  const auto img_it = img_buffer_.find(current_time);
  if (img_it == img_buffer_.end()) return boost::none;
  const auto current_img = img_it->second;
  // Clear buffer up to current time
  img_buffer_.erase(img_buffer_.begin(), img_it);

  return current_img;
}
}  // namespace localization_analysis
