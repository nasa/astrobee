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

#include <depth_odometry/depth_odometry_params.h>
#include <localization_common/time.h>

#include <boost/optional.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <sensor_msgs/PointCloud2.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace depth_odometry {
class DepthOdometry {
 public:
  DepthOdometry();
  boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> DepthCloudCallback(
    std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> depth_cloud);

 private:
  std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>> Icp(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b) const;
  Eigen::Matrix<double, 6, 6> ComputeCovarianceMatrix(
    const pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>& icp,
    const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_a,
    const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_a_transformed,
    const Eigen::Isometry3d& relative_transform) const;
  Eigen::Matrix<double, 1, 6> Jacobian(const pcl::PointNormal& source_point, const pcl::PointNormal& target_point,
                                       const Eigen::Isometry3d& relative_transform) const;

  std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> previous_depth_cloud_;
  std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> latest_depth_cloud_;
  DepthOdometryParams params_;
  ros::NodeHandle nh_;
  ros::Publisher pca_pub_, pcb_pub_, pct_pub_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_ODOMETRY_H_
