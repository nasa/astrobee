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
#include <QObject>
#include <ff_util/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <cv_bridge/cv_bridge.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include "depth_odometry_image_display.h"  // NOLINT
#include "utilities.h"                     // NOLINT

namespace localization_rviz_plugins {
namespace lc = localization_common;
namespace lm = localization_measurements;

DepthOdometryImageDisplay::DepthOdometryImageDisplay() {
  correspondence_index_slider_.reset(new rviz::SliderProperty("Select Correspondence", 0, "Select Correspondence.",
                                                              this, SLOT(createCorrespondencesImage())));

  image_transport::ImageTransport image_transport(nh_);
  const std::string image_topic = /*static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                  static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                  static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_DEPTH_IMAGE);*/
    "/hw/depth_haz/extended/amplitude_int";
  image_sub_ = image_transport.subscribe(image_topic, 10, &DepthOdometryImageDisplay::imageCallback, this);
  correspondence_image_pub_ = image_transport.advertise("/depth_odom/correspondence_image", 1);
}

void DepthOdometryImageDisplay::onInitialize() { MFDClass::onInitialize(); }

void DepthOdometryImageDisplay::reset() {
  MFDClass::reset();
  clearDisplay();
}

void DepthOdometryImageDisplay::clearDisplay() {}

void DepthOdometryImageDisplay::imageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
  img_buffer_.emplace(lc::TimeFromHeader(image_msg->header), image_msg);
}

sensor_msgs::ImageConstPtr DepthOdometryImageDisplay::getImage(const localization_common::Time time) {
  const auto img_it = img_buffer_.find(time);
  if (img_it == img_buffer_.end()) return nullptr;
  return img_it->second;
}

void DepthOdometryImageDisplay::clearImageBuffer(const localization_common::Time oldest_graph_time) {
  const auto img_it = img_buffer_.find(oldest_graph_time);
  if (img_it == img_buffer_.end()) return;
  img_buffer_.erase(img_buffer_.begin(), img_it);
}

void DepthOdometryImageDisplay::processMessage(const ff_msgs::ImageCorrespondences::ConstPtr& correspondences_msg) {
  latest_correspondences_msg_ = correspondences_msg;
  createCorrespondencesImage();
}

// TODO(rsoussan): Unify this with DepthOdometryPointCloudDisplay
void DepthOdometryImageDisplay::createCorrespondencesImage() {
  clearDisplay();
  if (!latest_correspondences_msg_) return;
  const lc::Time source_time = lc::TimeFromRosTime(latest_correspondences_msg_->source_time);
  const lc::Time target_time = lc::TimeFromRosTime(latest_correspondences_msg_->target_time);
  const auto source_image_msg = getImage(source_time);
  const auto target_image_msg = getImage(target_time);
  if (!source_image_msg || !target_image_msg) return;
  clearImageBuffer(source_time);

  auto source_image_measurement = lm::MakeImageMeasurement(source_image_msg, sensor_msgs::image_encodings::MONO16);
  auto target_image_measurement = lm::MakeImageMeasurement(target_image_msg, sensor_msgs::image_encodings::MONO16);
  if (!source_image_measurement || !target_image_measurement) return;
  auto& source_image = source_image_measurement->image;
  auto& target_image = target_image_measurement->image;

  // Create correspondence image
  // Draw source image above target image, add correspondences as points outlined
  // by rectangles to each image
  cv_bridge::CvImage correspondence_image;
  correspondence_image.encoding = sensor_msgs::image_encodings::RGB8;
  const int rows = target_image.rows;
  const int cols = target_image.cols;
  correspondence_image.image = cv::Mat(rows * 2, cols, CV_8UC3, cv::Scalar(0, 0, 0));

  correspondence_index_slider_->setMaximum(latest_correspondences_msg_->correspondences.size() - 1);
  const int correspondence_index = correspondence_index_slider_->getInt();
  const auto correspondence = latest_correspondences_msg_->correspondences[correspondence_index];
  const cv::Point source_correspondence_image_point(correspondence.source_point.x, correspondence.source_point.y);
  const cv::Point target_correspondence_image_point(correspondence.target_point.x, correspondence.target_point.y);
  const cv::Point rectangle_offset(40, 40);
  cv::circle(source_image, source_correspondence_image_point, 5 /* Radius*/, cv::Scalar(0, 255, 0), -1 /*Filled*/, 8);
  cv::rectangle(source_image, source_correspondence_image_point - rectangle_offset,
                source_correspondence_image_point + rectangle_offset, cv::Scalar(0, 255, 0), 8);
  cv::circle(target_image, target_correspondence_image_point, 5 /* Radius*/, cv::Scalar(0, 255, 0), -1 /*Filled*/, 8);
  cv::rectangle(target_image, target_correspondence_image_point - rectangle_offset,
                target_correspondence_image_point + rectangle_offset, cv::Scalar(0, 255, 0), 8);
  source_image.copyTo(correspondence_image.image(cv::Rect(0, 0, cols, rows)));
  target_image.copyTo(correspondence_image.image(cv::Rect(0, rows, cols, rows)));
  correspondence_image_pub_.publish(correspondence_image.toImageMsg());
}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::DepthOdometryImageDisplay, rviz::Display)
