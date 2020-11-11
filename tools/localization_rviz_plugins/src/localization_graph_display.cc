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

#include <ff_util/ff_names.h>
#include <graph_bag/utilities.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/time.h>
#include <localization_common/utilities.h>

#include <gtsam/base/serialization.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <cv_bridge/cv_bridge.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <glog/logging.h>

#include <string>
#include <utility>

#include "localization_graph_display.h"  // NOLINT
#include "utilities.h"                   // NOLINT

namespace localization_rviz_plugins {
namespace lc = localization_common;

LocalizationGraphDisplay::LocalizationGraphDisplay() {
  show_pose_axes_.reset(new rviz::BoolProperty("Show Pose Axes", true, "Show graph poses as axes.", this));
  pose_axes_size_.reset(new rviz::FloatProperty("Pose Axes Size", 0.1, "Pose axes size.", this));
  show_imu_factor_arrows_.reset(
    new rviz::BoolProperty("Show Imu Factor Arrows", true, "Show imu factors as arrows.", this));
  imu_factor_arrows_diameter_.reset(
    new rviz::FloatProperty("Imu Factor Arrows Diameter", 0.01, "Imu factor arrows diameter.", this));

  image_transport::ImageTransport image_transport(nh_);
  image_sub_ = image_transport.subscribe(TOPIC_HARDWARE_NAV_CAM, 1, &LocalizationGraphDisplay::imageCallback, this);
  optical_flow_image_pub_ = image_transport.advertise("/graph_localizer/optical_flow_feature_tracks", 1);
  smart_factor_projection_image_pub_ = image_transport.advertise("/graph_localizer/smart_factor_projections", 1);

  // TODO(rsoussan): avoid this and pass config path directly to config reader!!!!
  // make sure body_T_nav cam is correct!!!! (bsharp not bumble)
  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  char* argv = "script";
  char** argv_ptr = &argv;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv_ptr);
  const std::string config_path = "/home/rsoussan/astrobee/astrobee";
  const std::string world = "granite";
  const std::string robot_config_file = "config/robots/bsharp.config";
  lc::SetEnvironmentConfigs(config_path, world, robot_config_file);

  config_reader::ConfigReader config;
  config.AddFile("cameras.config");

  if (!config.ReadFiles()) {
    LOG(FATAL) << "Failed to read config files.";
  }
  // Needed for feature tracks visualization
  nav_cam_params_.reset(new camera::CameraParameters(&config, "nav_cam"));
}

void LocalizationGraphDisplay::onInitialize() { MFDClass::onInitialize(); }

void LocalizationGraphDisplay::reset() {
  MFDClass::reset();
  clearDisplay();
}

void LocalizationGraphDisplay::clearDisplay() {
  graph_pose_axes_.clear();
  imu_factor_arrows_.clear();
}

void LocalizationGraphDisplay::imageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
  img_buffer_.emplace(lc::TimeFromHeader(image_msg->header), image_msg);
}

void LocalizationGraphDisplay::addOpticalFlowVisual(const graph_localizer::FeatureTrackMap& feature_tracks,
                                                    const localization_common::Time latest_graph_time) {
  const auto img = getImage(latest_graph_time);
  if (!img) return;
  const auto feature_track_image = graph_bag::CreateFeatureTrackImage(img, feature_tracks, *nav_cam_params_);
  if (!feature_track_image) return;
  optical_flow_image_pub_.publish(*feature_track_image);
}

sensor_msgs::ImageConstPtr LocalizationGraphDisplay::getImage(const localization_common::Time time) {
  const auto img_it = img_buffer_.find(time);
  if (img_it == img_buffer_.end()) return nullptr;
  return img_it->second;
}

void LocalizationGraphDisplay::clearImageBuffer(const localization_common::Time oldest_graph_time) {
  const auto img_it = img_buffer_.find(oldest_graph_time);
  if (img_it == img_buffer_.end()) return;
  img_buffer_.erase(img_buffer_.begin(), img_it);
}

void LocalizationGraphDisplay::addSmartFactorProjectionVisual(const SmartFactor& smart_factor,
                                                              const graph_localizer::GraphValues& graph_values) {
  std::vector<cv::Mat> images;
  for (const auto& key : smart_factor.keys()) {
    const auto timestamp = graph_values.Timestamp(key);
    if (!timestamp) return;
    const auto image = getImage(*timestamp);
    if (!image) return;
    cv_bridge::CvImagePtr cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
      LOG(ERROR) << "cv_bridge exception: " << e.what();
      return;
    }

    images.emplace_back(cv_image->image);
  }

  if (images.empty()) return;
  const auto& image = images.front();
  cv_bridge::CvImage smart_factor_projection_image;
  // smart_factor_projection_image.header =
  smart_factor_projection_image.encoding = sensor_msgs::image_encodings::RGB8;
  smart_factor_projection_image.image = cv::Mat(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));

  const int num_images = images.size();
  int divisor;
  // Always create square images (2x2, 3x3, or 4x4)
  // Draw max 16 images
  if (num_images <= 4) {
    divisor = 2;
  } else if (num_images <= 9) {
    divisor = 3;
  } else {
    divisor = 4;
  }

  const int width = image.cols / divisor;
  const int height = image.rows / divisor;

  int destination_row = 0;
  int destination_column = 0;
  for (int i = 0; i < num_images && i < 16; ++i) {
    // TODO(rsoussan): projection image measurement point and map projection onto image!!!!!!
    cv::resize(images[i], images[i], cv::Size(width, height));
    images[i].copyTo(smart_factor_projection_image.image(cv::Rect(destination_column, destination_row, width, height)));
    destination_column += width;
    // Move down a row when a row of images in the destination image is filled.
    // Account for integer division (farthest column might not be equal to destination.cols)
    if (std::abs(destination_column - smart_factor_projection_image.image.cols) < 10) {
      destination_column = 0;
      destination_row += height;
    }
  }
  smart_factor_projection_image_pub_.publish(smart_factor_projection_image.toImageMsg());
}

void LocalizationGraphDisplay::addImuVisual(const graph_localizer::GraphLocalizer& graph_localizer,
                                            const gtsam::CombinedImuFactor* const imu_factor) {
  const auto pose = graph_localizer.graph_values().at<gtsam::Pose3>(imu_factor->key1());
  if (!pose) {
    LOG(ERROR) << "ProcessMessage: Failed to get pose.";
    return;
  }
  if (show_pose_axes_->getBool()) {
    const float scale = pose_axes_size_->getFloat();
    auto axis = axisFromPose(*pose, scale, context_->getSceneManager(), scene_node_);
    graph_pose_axes_.emplace_back(std::move(axis));
  }

  if (show_imu_factor_arrows_->getBool()) {
    const auto imu_predicted_combined_nav_state = pimPredict(graph_localizer, imu_factor);
    if (!imu_predicted_combined_nav_state) {
      LOG(ERROR) << "AddImuVisual: Failed to get pim predicted nav state.";
      return;
    }
    auto imu_factor_arrow = std::unique_ptr<rviz::Arrow>(new rviz::Arrow(context_->getSceneManager(), scene_node_));
    imu_factor_arrow->setPosition(ogrePosition(*pose));
    const auto orientation_and_length =
      getOrientationAndLength(pose->translation(), imu_predicted_combined_nav_state->pose().translation());
    imu_factor_arrow->setOrientation(orientation_and_length.first);
    const float diameter = imu_factor_arrows_diameter_->getFloat();
    imu_factor_arrow->set(3.0 * orientation_and_length.second / 4.0, 0.5 * diameter,
                          orientation_and_length.second / 4.0, diameter);
    imu_factor_arrow->setColor(1, 1, 0, 1);
    imu_factor_arrows_.emplace_back(std::move(imu_factor_arrow));
  }
}

void LocalizationGraphDisplay::processMessage(const ff_msgs::LocalizationGraph::ConstPtr& msg) {
  clearDisplay();
  graph_localizer::GraphLocalizer graph_localizer;
  gtsam::deserializeBinary(msg->serialized_graph, graph_localizer);
  SmartFactor* largest_error_smart_factor;
  double largest_smart_factor_error = -1;
  if (graph_localizer.graph_values().LatestTimestamp())
    addOpticalFlowVisual(graph_localizer.feature_tracks(), *(graph_localizer.graph_values().LatestTimestamp()));
  for (const auto factor : graph_localizer.factor_graph()) {
    const auto smart_factor = dynamic_cast<SmartFactor*>(factor.get());
    if (smart_factor) {
      const double smart_factor_error = smart_factor->error(graph_localizer.graph_values().values());
      if (smart_factor_error > largest_smart_factor_error) {
        largest_smart_factor_error = smart_factor_error;
        largest_error_smart_factor = smart_factor;
      }
    }
    const auto imu_factor = dynamic_cast<gtsam::CombinedImuFactor*>(factor.get());
    if (imu_factor) {
      addImuVisual(graph_localizer, imu_factor);
    }
  }
  if (largest_error_smart_factor) {
    addSmartFactorProjectionVisual(*largest_error_smart_factor, graph_localizer.graph_values());
  }

  const auto oldest_timestamp = graph_localizer.graph_values().OldestTimestamp();
  if (oldest_timestamp) clearImageBuffer(*oldest_timestamp);
}

}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::LocalizationGraphDisplay, rviz::Display)
