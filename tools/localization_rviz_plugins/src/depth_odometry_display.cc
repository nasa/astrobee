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
#include <msg_conversions/msg_conversions.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/calib3d.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <string>
#include <vector>

#include "depth_odometry_display.h"  // NOLINT
#include "utilities.h"               // NOLINT

namespace localization_rviz_plugins {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

cv::Point2f CvPoint(const Eigen::Vector2d& point) { return cv::Point2f(point.x(), point.y()); }

DepthOdometryDisplay::DepthOdometryDisplay() {
  int ff_argc = 1;
  char argv[] = "depth_odometry_display";
  char* argv_ptr = &argv[0];
  char** argv_ptr_ptr = &argv_ptr;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv_ptr_ptr);
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");
  config.AddFile("transforms.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  camera_params_.reset(new camera::CameraParameters(&config, "haz_cam"));
  const auto focal_lengths = camera_params_->GetFocalVector();
  const auto distortion_params = camera_params_->GetDistortion();
  const auto principal_points = camera_params_->GetOpticalOffset();
  intrinsics_ = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
  intrinsics_.at<double>(0, 0) = focal_lengths[0];
  intrinsics_.at<double>(1, 1) = focal_lengths[1];
  intrinsics_.at<double>(0, 2) = principal_points[0];
  intrinsics_.at<double>(1, 2) = principal_points[1];
  intrinsics_.at<double>(2, 2) = 1;
  distortion_params_ = cv::Mat(4, 1, cv::DataType<double>::type);
  for (int i = 0; i < distortion_params.size(); ++i) {
    distortion_params_.at<double>(i, 0) = distortion_params[i];
  }

  correspondence_index_slider_.reset(new rviz::SliderProperty("Select Correspondence", 0, "Select Correspondence.",
                                                              this, SLOT(createSingleCorrespondenceDisplays())));

  image_transport::ImageTransport image_transport(nh_);
  const std::string image_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                  static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                  static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_EXTENDED) +
                                  static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_AMPLITUDE_IMAGE);
  image_sub_ = image_transport.subscribe(image_topic, 10, &DepthOdometryDisplay::imageCallback, this);
  const std::string point_cloud_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                        static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                        static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX);
  point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
    point_cloud_topic, 10, &DepthOdometryDisplay::pointCloudCallback, this, ros::TransportHints().tcpNoDelay());
  single_correspondence_image_pub_ = image_transport.advertise("/depth_odom/single_correspondence_image", 1);
  projection_and_optical_flow_image_pub_ =
    image_transport.advertise("/depth_odom/projection_and_optical_flow_image", 1);
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
  img_buffer_.Add(lc::TimeFromHeader(image_msg->header), image_msg);
}

void DepthOdometryDisplay::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*point_cloud_msg, *point_cloud);
  point_cloud_buffer_.Add(lc::TimeFromHeader(point_cloud_msg->header), point_cloud);
}

void DepthOdometryDisplay::processMessage(const ff_msgs::DepthOdometry::ConstPtr& depth_odometry_msg) {
  latest_depth_odometry_measurement_ = lm::MakeDepthOdometryMeasurement(*depth_odometry_msg);
  createSingleCorrespondenceDisplays();
  createProjectionAndOpticalFlowImage(*latest_depth_odometry_measurement_);
}

// TODO(rsoussan): Move this to vision_common?
cv::Point2f DepthOdometryDisplay::projectPoint(const Eigen::Vector3d& point_3d) {
  cv::Mat zero_r(cv::Mat::eye(3, 3, cv::DataType<double>::type));
  cv::Mat zero_t(cv::Mat::zeros(3, 1, cv::DataType<double>::type));
  std::vector<cv::Point2d> projected_points;
  std::vector<cv::Point3d> object_points;
  object_points.emplace_back(cv::Point3d(point_3d.x(), point_3d.y(), point_3d.z()));
  cv::projectPoints(object_points, zero_r, zero_t, intrinsics_, distortion_params_, projected_points);
  return projected_points[0];
}

void DepthOdometryDisplay::createProjectionAndOpticalFlowImage(const lm::DepthOdometryMeasurement& depth_odometry) {
  if (!depth_odometry.correspondences.valid_image_points) return;

  const lc::Time source_time = depth_odometry.odometry.source_time;
  const lc::Time target_time = depth_odometry.odometry.target_time;
  const auto source_image_msg = img_buffer_.Get(source_time);
  const auto target_image_msg = img_buffer_.Get(target_time);
  if (!source_image_msg || !target_image_msg) return;

  auto source_image_measurement = lm::MakeImageMeasurement(*source_image_msg, sensor_msgs::image_encodings::RGB8);
  if (!source_image_measurement) return;
  auto& source_image = source_image_measurement->image;

  auto target_image_measurement = lm::MakeImageMeasurement(*target_image_msg, sensor_msgs::image_encodings::RGB8);
  if (!target_image_measurement) return;
  auto& target_image = target_image_measurement->image;

  // Draw optical flow tracks in target image, add projected 3D points as well
  cv_bridge::CvImage projection_and_optical_flow_image;
  projection_and_optical_flow_image.encoding = sensor_msgs::image_encodings::RGB8;
  const int rows = target_image.rows;
  const int cols = target_image.cols;
  projection_and_optical_flow_image.image = cv::Mat(rows * 2, cols, CV_8UC3, cv::Scalar(0, 0, 0));

  const auto& correspondences = depth_odometry.correspondences;
  const cv::Point2f rectangle_offset(3, 3);
  for (int i = 0; i < correspondences.source_image_points.size(); ++i) {
    const cv::Point2f source_image_point = CvPoint(correspondences.source_image_points[i]);
    const Eigen::Vector3d& target_3d_point = correspondences.target_3d_points[i];
    const Eigen::Vector3d frame_changed_target_3d_point =
      depth_odometry.odometry.sensor_F_source_T_target.pose * target_3d_point;
    const auto projected_target_point = projectPoint(frame_changed_target_3d_point);
    const cv::Point2f target_image_point = CvPoint(correspondences.target_image_points[i]);
    // Draw source point and projected target point in source image
    cv::circle(source_image, source_image_point, 1 /* Radius*/, cv::Scalar(0, 255, 0), -1 /*Filled*/, 8);
    cv::rectangle(source_image, source_image_point - rectangle_offset, source_image_point + rectangle_offset,
                  cv::Scalar(0, 255, 0), 1);
    cv::circle(source_image, projected_target_point, 1 /* Radius*/, cv::Scalar(255, 0, 0), -1 /*Filled*/, 8);
    cv::line(source_image, source_image_point, projected_target_point, cv::Scalar(255, 0, 0), 2, 8, 0);
    // Draw target point and optical flow line in target image
    cv::circle(target_image, target_image_point, 1 /* Radius*/, cv::Scalar(0, 255, 0), -1 /*Filled*/, 8);
    cv::rectangle(target_image, target_image_point - rectangle_offset, target_image_point + rectangle_offset,
                  cv::Scalar(0, 255, 0), 1);
    cv::circle(target_image, source_image_point, 1 /* Radius*/, cv::Scalar(255, 0, 0), -1 /*Filled*/, 8);
    cv::line(target_image, target_image_point, source_image_point, cv::Scalar(255, 0, 0), 2, 8, 0);
  }
  source_image.copyTo(projection_and_optical_flow_image.image(cv::Rect(0, 0, cols, rows)));
  target_image.copyTo(projection_and_optical_flow_image.image(cv::Rect(0, rows, cols, rows)));
  projection_and_optical_flow_image_pub_.publish(projection_and_optical_flow_image.toImageMsg());
  // Account for time offset when erasing point cloud buffer
  point_cloud_buffer_.EraseUpTo(source_time - 0.05);
  img_buffer_.EraseUpTo(source_time);
}

void DepthOdometryDisplay::createSingleCorrespondenceDisplays() {
  if (!latest_depth_odometry_measurement_) return;
  const auto& depth_odometry = *latest_depth_odometry_measurement_;
  clearDisplay();
  // Publish 3d correspondences
  correspondence_index_slider_->setMaximum(depth_odometry.correspondences.target_3d_points.size() - 1);
  const int correspondence_index = correspondence_index_slider_->getInt();
  const lc::Time source_time = depth_odometry.odometry.source_time;
  const lc::Time target_time = depth_odometry.odometry.target_time;
  const Eigen::Vector3d& source_3d_point = depth_odometry.correspondences.source_3d_points[correspondence_index];
  const Eigen::Vector3d& target_3d_point = depth_odometry.correspondences.target_3d_points[correspondence_index];
  publishCorrespondencePoints(source_3d_point, target_3d_point, source_time, target_time);

  // Publish image space correspondences if available
  if (!depth_odometry.correspondences.valid_image_points) return;
  const auto source_image_msg = img_buffer_.Get(source_time);
  const auto target_image_msg = img_buffer_.Get(target_time);
  if (!source_image_msg || !target_image_msg) return;

  auto source_image_measurement = lm::MakeImageMeasurement(*source_image_msg, sensor_msgs::image_encodings::RGB8);
  if (!source_image_measurement) return;
  auto& source_image = source_image_measurement->image;
  auto target_image_measurement = lm::MakeImageMeasurement(*target_image_msg, sensor_msgs::image_encodings::RGB8);
  if (!target_image_measurement) return;
  auto& target_image = target_image_measurement->image;

  // Create correspondence image
  // Draw source image above target image, add correspondences as points outlined
  // by rectangles to each image
  cv_bridge::CvImage single_correspondence_image;
  single_correspondence_image.encoding = sensor_msgs::image_encodings::RGB8;
  const int rows = target_image.rows;
  const int cols = target_image.cols;
  single_correspondence_image.image = cv::Mat(rows * 2, cols, CV_8UC3, cv::Scalar(0, 0, 0));

  const cv::Point2f source_image_point =
    CvPoint(depth_odometry.correspondences.source_image_points[correspondence_index]);
  const Eigen::Vector3d frame_changed_target_3d_point =
    depth_odometry.odometry.sensor_F_source_T_target.pose * target_3d_point;
  const auto projected_target_point = projectPoint(frame_changed_target_3d_point);
  const cv::Point2f target_image_point =
    CvPoint(depth_odometry.correspondences.target_image_points[correspondence_index]);
  const cv::Point2f rectangle_offset(40, 40);
  cv::circle(source_image, source_image_point, 5 /* Radius*/, cv::Scalar(0, 255, 0), -1 /*Filled*/, 8);
  cv::rectangle(source_image, source_image_point - rectangle_offset, source_image_point + rectangle_offset,
                cv::Scalar(0, 255, 0), 8);
  cv::circle(source_image, projected_target_point, 3 /* Radius*/, cv::Scalar(255, 0, 0), -1 /*Filled*/, 8);
  cv::circle(target_image, target_image_point, 5 /* Radius*/, cv::Scalar(0, 255, 0), -1 /*Filled*/, 8);
  cv::rectangle(target_image, target_image_point - rectangle_offset, target_image_point + rectangle_offset,
                cv::Scalar(0, 255, 0), 8);
  source_image.copyTo(single_correspondence_image.image(cv::Rect(0, 0, cols, rows)));
  target_image.copyTo(single_correspondence_image.image(cv::Rect(0, rows, cols, rows)));
  single_correspondence_image_pub_.publish(single_correspondence_image.toImageMsg());
}

void DepthOdometryDisplay::publishCorrespondencePoints(const Eigen::Vector3d& source_3d_point,
                                                       const Eigen::Vector3d& target_3d_point,
                                                       const lc::Time source_time, const lc::Time target_time) {
  const auto source_point_cloud = point_cloud_buffer_.GetNearby(source_time, 0.05);
  const auto target_point_cloud = point_cloud_buffer_.GetNearby(target_time, 0.05);
  if (!source_point_cloud || !target_point_cloud) return;
  geometry_msgs::PointStamped source_3d_point_msg;
  mc::VectorToMsg(source_3d_point, source_3d_point_msg.point);
  source_3d_point_msg.header.stamp = ros::Time::now();
  source_3d_point_msg.header.frame_id = "haz_cam";
  source_correspondence_point_pub_.publish(source_3d_point_msg);

  geometry_msgs::PointStamped target_3d_point_msg;
  mc::VectorToMsg(target_3d_point, target_3d_point_msg.point);
  target_3d_point_msg.header.stamp = ros::Time::now();
  target_3d_point_msg.header.frame_id = "haz_cam";
  target_correspondence_point_pub_.publish(target_3d_point_msg);

  {
    const auto source_cloud_msg =
      lm::MakePointCloudMsg(**source_point_cloud, lc::TimeFromRosTime(ros::Time::now()), "haz_cam");
    source_point_cloud_pub_.publish(source_cloud_msg);
    const auto target_cloud_msg =
      lm::MakePointCloudMsg(**target_point_cloud, lc::TimeFromRosTime(ros::Time::now()), "haz_cam");
    target_point_cloud_pub_.publish(target_cloud_msg);
  }
}
}  // namespace localization_rviz_plugins

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(localization_rviz_plugins::DepthOdometryDisplay, rviz::Display)
