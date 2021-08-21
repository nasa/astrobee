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

// Header file must go in src directory for Qt/Rviz plugin
#ifndef LOCALIZATION_RVIZ_PLUGINS_DEPTH_ODOMETRY_DISPLAY_H_  // NOLINT
#define LOCALIZATION_RVIZ_PLUGINS_DEPTH_ODOMETRY_DISPLAY_H_  // NOLINT

// Required for Qt
#ifndef Q_MOC_RUN
#include <ff_msgs/DepthImageCorrespondences.h>
#include <localization_common/time.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <rviz/message_filter_display.h>
#include "slider_property.h"  // NOLINT
#endif

// Forward declarations for ogre and rviz
namespace Ogre {
class SceneNode;
}

namespace localization_rviz_plugins {
class DepthOdometryDisplay : public rviz::MessageFilterDisplay<ff_msgs::DepthImageCorrespondences> {
  Q_OBJECT    // NOLINT
    public :  // NOLINT
              DepthOdometryDisplay();
  ~DepthOdometryDisplay() = default;

  // private:
 protected:
  void onInitialize() final;
  void reset() final;

 private Q_SLOTS:  // NOLINT
  void createCorrespondencesImage();

 private:
  void processMessage(const ff_msgs::DepthImageCorrespondences::ConstPtr& correspondences_msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg);
  void publishCorrespondencePoints(const ff_msgs::DepthImageCorrespondence& correspondence,
                                   const localization_common::Time source_time,
                                   const localization_common::Time target_time);
  void clearImageBuffer(const localization_common::Time oldest_allowed_time);
  void clearDisplay();
  sensor_msgs::ImageConstPtr getImage(const localization_common::Time time);
  pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud(const localization_common::Time time);

  std::unique_ptr<rviz::SliderProperty> correspondence_index_slider_;
  ff_msgs::DepthImageCorrespondences::ConstPtr latest_correspondences_msg_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher source_correspondence_point_pub_, target_correspondence_point_pub_;
  ros::Publisher source_point_cloud_pub_, target_point_cloud_pub_;
  image_transport::Publisher correspondence_image_pub_;
  ros::NodeHandle nh_;
  // TODO(rsoussan): Create seperate class for image buffer, unify with loc graph display
  std::map<localization_common::Time, sensor_msgs::ImageConstPtr> img_buffer_;
  std::map<localization_common::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> point_cloud_buffer_;
};
}  // namespace localization_rviz_plugins
#endif  // LOCALIZATION_RVIZ_PLUGINS_DEPTH_ODOMETRY_DISPLAY_H_ NOLINT
