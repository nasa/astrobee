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

#include <depth_odometry/depth_odometry.h>
#include <localization_common/logger.h>

namespace depth_odometry {
namespace lc = localization_common;

DepthOdometry::DepthOdometry() {}

void DepthOdometry::DepthCloudCallback(
  std::shared_ptr<std::pair<lc::Time, pcl::PointCloud<pcl::PointXYZ>>> depth_cloud) {
  if (!previous_depth_cloud_ && !latest_depth_cloud_) latest_depth_cloud_ = depth_cloud;
  if (depth_cloud->first < latest_depth_cloud_->first) {
    LogWarning("DepthCloudCallback: Out of order measurement received.");
    return;
  }
  previous_depth_cloud_ = latest_depth_cloud_;
  latest_depth_cloud_ = depth_cloud;
}
}  // namespace depth_odometry
