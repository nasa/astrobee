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
#ifndef DEPTH_ODOMETRY_DEPTH_ODOMETRY_H_
#define DEPTH_ODOMETRY_DEPTH_ODOMETRY_H_

#include <localization_common/time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace depth_odometry {
class DepthOdometry {
 public:
  DepthOdometry();
  void DepthCloudCallback(std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> depth_cloud);

 private:
  Eigen::Isometry3d Icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b) const;
  std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> previous_depth_cloud_;
  std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> latest_depth_cloud_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_ODOMETRY_H_
