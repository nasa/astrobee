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
#include <msg_conversions/msg_conversions.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace depth_odometry {
namespace lc = localization_common;

DepthOdometry::DepthOdometry() {
  // TODO(rsoussan): remove this
  config_reader::ConfigReader config;
  config.AddFile("transforms.config");
  config.AddFile("geometry.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }

  body_T_haz_cam_ = msg_conversions::LoadEigenTransform(config, "haz_cam_transform");
  // body_T_haz_cam_.translation() = Eigen::Vector3d::Zero();
  LogError("body_T_haz_no_trans: " << body_T_haz_cam_.matrix());
  pca_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pca", 10);
  pcb_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pcb", 10);
  pct_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pct", 10);
}

boost::optional<Eigen::Isometry3d> DepthOdometry::DepthCloudCallback(
  std::pair<lc::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> depth_cloud) {
  if (!previous_depth_cloud_.second && !latest_depth_cloud_.second) latest_depth_cloud_ = depth_cloud;
  if (depth_cloud.first < latest_depth_cloud_.first) {
    LogWarning("DepthCloudCallback: Out of order measurement received.");
    return boost::none;
  }
  LogError("t: " << std::setprecision(15) << depth_cloud.first);
  previous_depth_cloud_ = latest_depth_cloud_;
  latest_depth_cloud_ = depth_cloud;

  return body_T_haz_cam_ * Icp(latest_depth_cloud_.second, previous_depth_cloud_.second) * body_T_haz_cam_.inverse();
}

Eigen::Isometry3d DepthOdometry::Icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b) const {
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_b_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_b_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud_b);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.03);
  ne.compute(*cloud_b_normals);
  pcl::concatenateFields(*cloud_b, *cloud_b_normals, *cloud_b_with_normals);
  /*for (auto it = cloud_a_with_normals->begin(); it != cloud_a_with_normals->end(); ++it){
  //for (auto it = cloud_a->begin(); it != cloud_a->end(); ++it){
    //LogError("p: " << it->x << ", " << it->y << ", " << it->z << ", " << it->normal);
    LogError("p: " << it->normal[0] << ", " << it->normal[1] << ", " << it->normal[2]);
 // LogError("p: " << it->x << ", " << it->y << ", " << it->z);
  }*/
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_a_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*cloud_a, *cloud_a_with_normals);
  pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
  icp.setInputSource(cloud_a_with_normals);
  icp.setInputTarget(cloud_b_with_normals);
  icp.setMaximumIterations(10);
  pcl::PointCloud<pcl::PointNormal> result;
  icp.align(result);

  {
    sensor_msgs::PointCloud2 ros_cloud_a;
    pcl::toROSMsg(*cloud_a, ros_cloud_a);
    ros_cloud_a.header.stamp = ros::Time::now();
    ros_cloud_a.header.frame_id = "haz_cam";
    pca_pub_.publish(ros_cloud_a);
    sensor_msgs::PointCloud2 ros_cloud_b;
    pcl::toROSMsg(*cloud_b, ros_cloud_b);
    ros_cloud_b.header.stamp = ros::Time::now();
    ros_cloud_b.header.frame_id = "haz_cam";
    pcb_pub_.publish(ros_cloud_b);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_a, *cloud_t, icp.getFinalTransformation());
    sensor_msgs::PointCloud2 ros_cloud_t;
    pcl::toROSMsg(*cloud_t, ros_cloud_t);
    ros_cloud_t.header.stamp = ros::Time::now();
    ros_cloud_t.header.frame_id = "haz_cam";
    pct_pub_.publish(ros_cloud_t);
  }
  if (icp.hasConverged())
    LogError("converged!!!");
  else
    LogError("failed to converge!!!!!!!");

  LogError("fit score: " << icp.getFitnessScore());

  // TODO(rsoussan): clean this up
  const Eigen::Isometry3d relative_transform(
    Eigen::Isometry3f(icp.getFinalTransformation().matrix()).cast<double>());  //.cast<double>();
  return relative_transform;
  // TODO(rsoussan): get covariance!!!
}
}  // namespace depth_odometry
