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
#include <pcl_conversions/pcl_conversions.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include "depth_odometry_display.h"  // NOLINT
#include "utilities.h"               // NOLINT

namespace localization_rviz_plugins {
namespace lc = localization_common;
namespace lm = localization_measurements;

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
  source_correspondence_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("source_point_with_correspondence", 10);
  target_correspondence_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("target_point_with_correspondence", 10);
  source_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("source_cloud", 10);
  target_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("target_cloud", 10);
}

void DepthOdometryDisplay::onInitialize() { MFDClass::onInitialize(); }

void DepthOdometryDisplay::reset() {
  MFDClass::reset();
  clearDisplay();
}

void DepthOdometryDisplay::clearDisplay() {}

void DepthOdometryDisplay::imageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
  img_buffer_.AddMeasurement(lc::TimeFromHeader(image_msg->header), image_msg);
}

void DepthOdometryDisplay::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*point_cloud_msg, *point_cloud);
  point_cloud_buffer_.AddMeasurement(lc::TimeFromHeader(point_cloud_msg->header), point_cloud);
}

void DepthOdometryDisplay::processMessage(const ff_msgs::DepthImageCorrespondences::ConstPtr& correspondences_msg) {
  latest_correspondences_msg_ = correspondences_msg;
  createCorrespondencesImage();
}

void DepthOdometryDisplay::createCorrespondencesImage() {
  clearDisplay();
  if (!latest_correspondences_msg_) return;
  const lc::Time source_time = lc::TimeFromRosTime(latest_correspondences_msg_->source_time);
  const lc::Time target_time = lc::TimeFromRosTime(latest_correspondences_msg_->target_time);
  const auto source_image_msg = img_buffer_.GetMeasurement(source_time);
  const auto target_image_msg = img_buffer_.GetMeasurement(target_time);
  if (!source_image_msg || !target_image_msg) return;
  img_buffer_.EraseIncluding(source_time);

  // TODO(rsoussan): make function for this, unify with loc graph display
  cv_bridge::CvImagePtr source_cv_image;
  try {
    source_cv_image = cv_bridge::toCvCopy(*source_image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    LogError("cv_bridge exception: " << e.what());
    return;
  }
  auto& source_image = source_cv_image->image;

  cv_bridge::CvImagePtr target_cv_image;
  try {
    target_cv_image = cv_bridge::toCvCopy(*target_image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    LogError("cv_bridge exception: " << e.what());
    return;
  }
  auto& target_image = target_cv_image->image;

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
  const int source_correspondence_index = correspondence.source_index;
  const int source_correspondence_row = source_correspondence_index / cols;
  const int source_correspondence_col = source_correspondence_index - cols * source_correspondence_row;
  const cv::Point source_correspondence_image_point(source_correspondence_col, source_correspondence_row);
  const int target_correspondence_index = correspondence.target_index;
  const int target_correspondence_row = target_correspondence_index / cols;
  const int target_correspondence_col = target_correspondence_index - cols * target_correspondence_row;
  const cv::Point target_correspondence_image_point(target_correspondence_col, target_correspondence_row);
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
  publishCorrespondencePoints(correspondence, source_time, target_time);
}

void DepthOdometryDisplay::publishCorrespondencePoints(const ff_msgs::DepthImageCorrespondence& correspondence,
                                                       const lc::Time source_time, const lc::Time target_time) {
  const auto source_point_cloud = point_cloud_buffer_.GetNearbyMeasurement(source_time, 0.05);
  const auto target_point_cloud = point_cloud_buffer_.GetNearbyMeasurement(target_time, 0.05);
  if (!source_point_cloud || !target_point_cloud) return;
  geometry_msgs::PointStamped source_correspondence_point_msg;
  const auto source_correspondence_point = (*source_point_cloud)->points[correspondence.source_index];
  source_correspondence_point_msg.point.x = source_correspondence_point.x;
  source_correspondence_point_msg.point.y = source_correspondence_point.y;
  source_correspondence_point_msg.point.z = source_correspondence_point.z;
  source_correspondence_point_msg.header.stamp = ros::Time::now();
  source_correspondence_point_msg.header.frame_id = "haz_cam";
  source_correspondence_point_pub_.publish(source_correspondence_point_msg);

  geometry_msgs::PointStamped target_correspondence_point_msg;
  const auto target_correspondence_point = (*target_point_cloud)->points[correspondence.target_index];
  target_correspondence_point_msg.point.x = target_correspondence_point.x;
  target_correspondence_point_msg.point.y = target_correspondence_point.y;
  target_correspondence_point_msg.point.z = target_correspondence_point.z;
  target_correspondence_point_msg.header.stamp = ros::Time::now();
  target_correspondence_point_msg.header.frame_id = "haz_cam";
  target_correspondence_point_pub_.publish(target_correspondence_point_msg);

  {
    const auto source_cloud_msg =
      lm::MakePointCloudMsg(**source_point_cloud, lc::TimeFromRosTime(ros::Time::now()), "haz_cam");
    source_point_cloud_pub_.publish(source_cloud_msg);
    const auto target_cloud_msg =
      lm::MakePointCloudMsg(**target_point_cloud, lc::TimeFromRosTime(ros::Time::now()), "haz_cam");
    target_point_cloud_pub_.publish(target_cloud_msg);
  }

  point_cloud_buffer_.EraseIncluding(source_time);
}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::DepthOdometryDisplay, rviz::Display)
