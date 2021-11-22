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
#ifndef DEPTH_ODOMETRY_DEPTH_ODOMETRY_WRAPPER_H_
#define DEPTH_ODOMETRY_DEPTH_ODOMETRY_WRAPPER_H_

#include <depth_odometry/depth_odometry.h>
#include <ff_msgs/Odometry.h>
#include <localization_common/measurement_buffer.h>
#include <localization_common/time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace depth_odometry {
class DepthOdometryWrapper {
 public:
  DepthOdometryWrapper();
  std::vector<ff_msgs::Odometry> PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg);
  std::vector<ff_msgs::Odometry> ImageCallback(const sensor_msgs::ImageConstPtr& depth_image_msg);
  bool depth_image_registration_enabled() const { return depth_odometry_->params().depth_image_registration_enabled; }
  bool depth_point_cloud_registration_enabled() const {
    return depth_odometry_->params().depth_point_cloud_registration_enabled;
  }

 private:
  std::vector<ff_msgs::Odometry> ProcessDepthImageIfAvailable();

  std::unique_ptr<DepthOdometry> depth_odometry_;
  localization_common::MeasurementBuffer<sensor_msgs::PointCloud2ConstPtr> point_cloud_buffer_;
  localization_common::MeasurementBuffer<sensor_msgs::ImageConstPtr> image_buffer_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_ODOMETRY_WRAPPER_H_
