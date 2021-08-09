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
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <cv_bridge/cv_bridge.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include "depth_odometry_display.h"  // NOLINT
#include "utilities.h"               // NOLINT

namespace localization_rviz_plugins {
namespace lc = localization_common;

DepthOdometryDisplay::DepthOdometryDisplay() {
  correspondence_index_slider_.reset(new rviz::SliderProperty("Select Correspondence", 0, "Select Correspondence.",
                                                              this, SLOT(createCorrespondencesImage())));

  image_transport::ImageTransport image_transport(nh_);
  const std::string image_topic = /*static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                  static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                  static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_DEPTH_IMAGE);*/
    "/hw/depth_haz/extended/amplitude_int";
  image_sub_ = image_transport.subscribe(image_topic, 10, &DepthOdometryDisplay::imageCallback, this);
  const std::string point_cloud_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                        static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                        static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX);
  point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
    point_cloud_topic, 10, &DepthOdometryDisplay::pointCloudCallback, this, ros::TransportHints().tcpNoDelay());
  correspondence_image_pub_ = image_transport.advertise("/depth_odom/correspondence_image", 1);
  source_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("source_cloud_with_correspondence", 10);
  target_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("target_cloud_with_correspondence", 10);
}

void DepthOdometryDisplay::onInitialize() { MFDClass::onInitialize(); }

void DepthOdometryDisplay::reset() {
  MFDClass::reset();
  clearDisplay();
}

void DepthOdometryDisplay::clearDisplay() {}

void DepthOdometryDisplay::imageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
  img_buffer_.emplace(lc::TimeFromHeader(image_msg->header), image_msg);
}

void DepthOdometryDisplay::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
  point_cloud_buffer_.emplace(lc::TimeFromHeader(point_cloud_msg->header), point_cloud_msg);
}

sensor_msgs::ImageConstPtr DepthOdometryDisplay::getImage(const localization_common::Time time) {
  const auto img_it = img_buffer_.find(time);
  if (img_it == img_buffer_.end()) return nullptr;
  return img_it->second;
}

void DepthOdometryDisplay::clearImageBuffer(const localization_common::Time oldest_graph_time) {
  const auto img_it = img_buffer_.find(oldest_graph_time);
  if (img_it == img_buffer_.end()) return;
  img_buffer_.erase(img_buffer_.begin(), img_it);
}

sensor_msgs::PointCloud2ConstPtr DepthOdometryDisplay::getPointCloud(const localization_common::Time time) {
  const auto point_cloud_it = point_cloud_buffer_.find(time);
  if (point_cloud_it == point_cloud_buffer_.end()) return nullptr;
  return point_cloud_it->second;
}

void DepthOdometryDisplay::processMessage(const ff_msgs::DepthCorrespondences::ConstPtr& correspondences_msg) {
  latest_correspondences_msg_ = correspondences_msg;
  createCorrespondencesImage();
}

void DepthOdometryDisplay::createCorrespondencesImage() {
  clearDisplay();
  if (!latest_correspondences_msg_) return;
  const lc::Time previous_time = lc::TimeFromRosTime(latest_correspondences_msg_->previous_time);
  const lc::Time latest_time = lc::TimeFromRosTime(latest_correspondences_msg_->latest_time);
  const auto previous_image_msg = getImage(previous_time);
  const auto latest_image_msg = getImage(latest_time);
  if (!previous_image_msg || !latest_image_msg) return;
  clearImageBuffer(previous_time);

  // TODO(rsoussan): make function for this, unify with loc graph display
  cv_bridge::CvImagePtr previous_cv_image;
  try {
    previous_cv_image = cv_bridge::toCvCopy(previous_image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    LogError("cv_bridge exception: " << e.what());
    return;
  }
  auto& previous_image = previous_cv_image->image;

  cv_bridge::CvImagePtr latest_cv_image;
  try {
    latest_cv_image = cv_bridge::toCvCopy(latest_image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    LogError("cv_bridge exception: " << e.what());
    return;
  }
  auto& latest_image = latest_cv_image->image;

  // Create correspondence image
  // Draw previous image above latest image, add correspondences as points outlined
  // by rectangles to each image
  cv_bridge::CvImage correspondence_image;
  correspondence_image.encoding = sensor_msgs::image_encodings::RGB8;
  const int rows = latest_image.rows;
  const int cols = latest_image.cols;
  correspondence_image.image = cv::Mat(rows * 2, cols, CV_8UC3, cv::Scalar(0, 0, 0));

  correspondence_index_slider_->setMaximum(latest_correspondences_msg_->correspondences.size() - 1);
  const int correspondence_index = correspondence_index_slider_->getInt();
  const auto correspondence = latest_correspondences_msg_->correspondences[correspondence_index];
  const int previous_correspondence_index = correspondence.previous_image_index;
  const int previous_correspondence_row = previous_correspondence_index / cols;
  const int previous_correspondence_col = previous_correspondence_index - cols * previous_correspondence_row;
  const cv::Point previous_correspondence_image_point(previous_correspondence_col, previous_correspondence_row);
  const int latest_correspondence_index = correspondence.latest_image_index;
  const int latest_correspondence_row = latest_correspondence_index / cols;
  const int latest_correspondence_col = latest_correspondence_index - cols * latest_correspondence_row;
  const cv::Point latest_correspondence_image_point(latest_correspondence_col, latest_correspondence_row);
  const cv::Point rectangle_offset(40, 40);
  cv::circle(previous_image, previous_correspondence_image_point, 5 /* Radius*/, cv::Scalar(0, 255, 0), -1 /*Filled*/,
             8);
  cv::rectangle(previous_image, previous_correspondence_image_point - rectangle_offset,
                previous_correspondence_image_point + rectangle_offset, cv::Scalar(0, 255, 0), 8);
  cv::circle(latest_image, latest_correspondence_image_point, 5 /* Radius*/, cv::Scalar(0, 255, 0), -1 /*Filled*/, 8);
  cv::rectangle(latest_image, latest_correspondence_image_point - rectangle_offset,
                latest_correspondence_image_point + rectangle_offset, cv::Scalar(0, 255, 0), 8);
  previous_image.copyTo(correspondence_image.image(cv::Rect(0, 0, cols, rows)));
  latest_image.copyTo(correspondence_image.image(cv::Rect(0, rows, cols, rows)));
  correspondence_image_pub_.publish(correspondence_image.toImageMsg());
  publishPointClouds(correspondence, previous_time, latest_time);
}

void DepthOdometryDisplay::publishPointClouds(const ff_msgs::DepthCorrespondence& correspondence,
                                              const lc::Time previous_time, const lc::Time latest_time) {
  const auto previous_point_cloud = getPointCloud(previous_time);
  const auto latest_point_cloud = getPointCloud(latest_time);
  if (!previous_point_cloud || !latest_point_cloud) return;
  source_cloud_pub_.publish(*previous_point_cloud);
}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::DepthOdometryDisplay, rviz::Display)
