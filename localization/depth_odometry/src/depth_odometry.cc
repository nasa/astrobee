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
#include <depth_odometry/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <gtsam/geometry/Pose3.h>

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
  config.AddFile("localization/depth_odometry.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }

  LoadDepthOdometryParams(config, params_);

  if (params_.publish_point_clouds) {
    pca_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pca", 10);
    pcb_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pcb", 10);
    pct_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pct", 10);
  }
}

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> DepthOdometry::DepthCloudCallback(
  std::pair<lc::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> depth_cloud) {
  if (!previous_depth_cloud_.second && !latest_depth_cloud_.second) latest_depth_cloud_ = depth_cloud;
  if (depth_cloud.first < latest_depth_cloud_.first) {
    LogWarning("DepthCloudCallback: Out of order measurement received.");
    return boost::none;
  }
  LogError("t: " << std::setprecision(15) << depth_cloud.first);
  previous_depth_cloud_ = latest_depth_cloud_;
  latest_depth_cloud_ = depth_cloud;
  auto relative_transform = Icp(latest_depth_cloud_.second, previous_depth_cloud_.second);
  if (!relative_transform) {
    LogWarning("DepthCloudCallback: Failed to get relative transform.");
    return boost::none;
  }

  if (!CovarianceSane(relative_transform->second)) {
    LogWarning("DepthCloudCallback: Sanity check failed - invalid covariance.");
    return boost::none;
  }

  if (params_.frame_change_transform) {
    relative_transform->first = params_.body_T_haz_cam * relative_transform->first * params_.body_T_haz_cam.inverse();
    // TODO: rotate covariance matrix!!!! use exp map jacobian!!! sandwich withthis! (translation should be rotated by
    // rotation matrix)
  }

  LogError("cov: " << std::endl << relative_transform->second.matrix());

  return relative_transform;
}

bool DepthOdometry::CovarianceSane(const Eigen::Matrix<double, 6, 6>& covariance) const {
  const auto position_covariance_norm = covariance.block<3, 3>(0, 0).diagonal().norm();
  const auto orientation_covariance_norm = covariance.block<3, 3>(3, 3).diagonal().norm();
  LogError("pcov: " << position_covariance_norm << ", ocov: " << orientation_covariance_norm);
  return (position_covariance_norm <= params_.position_covariance_threshold &&
          orientation_covariance_norm <= params_.orientation_covariance_threshold);
}

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> DepthOdometry::Icp(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b) const {
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_b_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_b_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud_b);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(params_.search_radius);
  ne.compute(*cloud_b_normals);
  pcl::concatenateFields(*cloud_b, *cloud_b_normals, *cloud_b_with_normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_a_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*cloud_a, *cloud_a_with_normals);
  pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
  icp.setInputSource(cloud_a_with_normals);
  icp.setInputTarget(cloud_b_with_normals);
  icp.setMaximumIterations(10);
  pcl::PointCloud<pcl::PointNormal>::Ptr result(new pcl::PointCloud<pcl::PointNormal>);
  icp.align(*result);

  if (params_.publish_point_clouds) {
    PublishPointClouds(*cloud_a, *cloud_b, icp.getFinalTransformation());
  }

  if (!icp.hasConverged()) {
    LogWarning("Icp: Failed to converge.");
    return boost::none;
  }

  const double fitness_score = icp.getFitnessScore();
  if (fitness_score < params_.fitness_threshold) {
    LogWarning("Icp: Fitness score too low: " << fitness_score << ".");
    return boost::none;
  }

  // TODO(rsoussan): clean this up
  const Eigen::Isometry3d relative_transform(Eigen::Isometry3f(icp.getFinalTransformation().matrix()).cast<double>());
  const Eigen::Matrix<double, 6, 6> covariance =
    ComputeCovarianceMatrix(icp, cloud_a_with_normals, result, relative_transform);
  return std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>{relative_transform, covariance};
}

void DepthOdometry::PublishPointClouds(const pcl::PointCloud<pcl::PointXYZ>& cloud_a,
                                       const pcl::PointCloud<pcl::PointXYZ>& cloud_b,
                                       const Eigen::Matrix<float, 4, 4>& relative_transform) const {
  sensor_msgs::PointCloud2 ros_cloud_a;
  pcl::toROSMsg(cloud_a, ros_cloud_a);
  ros_cloud_a.header.stamp = ros::Time::now();
  ros_cloud_a.header.frame_id = "haz_cam";
  pca_pub_.publish(ros_cloud_a);
  sensor_msgs::PointCloud2 ros_cloud_b;
  pcl::toROSMsg(cloud_b, ros_cloud_b);
  ros_cloud_b.header.stamp = ros::Time::now();
  ros_cloud_b.header.frame_id = "haz_cam";
  pcb_pub_.publish(ros_cloud_b);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(cloud_a, *cloud_t, relative_transform);
  sensor_msgs::PointCloud2 ros_cloud_t;
  pcl::toROSMsg(*cloud_t, ros_cloud_t);
  ros_cloud_t.header.stamp = ros::Time::now();
  ros_cloud_t.header.frame_id = "haz_cam";
  pct_pub_.publish(ros_cloud_t);
}

Eigen::Matrix<double, 1, 6> DepthOdometry::Jacobian(const pcl::PointNormal& source_point,
                                                    const pcl::PointNormal& target_point,
                                                    const Eigen::Isometry3d& relative_transform) const {
  const gtsam::Pose3 gt_relative_transform = lc::GtPose(relative_transform);
  const gtsam::Point3 gt_point(source_point.x, source_point.y, source_point.z);
  const gtsam::Point3 gt_normal(target_point.normal[0], target_point.normal[1], target_point.normal[2]);
  return depth_odometry::Jacobian(gt_point, gt_normal, gt_relative_transform);
}

Eigen::Matrix<double, 6, 6> DepthOdometry::ComputeCovarianceMatrix(
  const pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>& icp,
  const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_a,
  const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_a_transformed, const Eigen::Isometry3d& relative_transform) const {
  icp.correspondence_estimation_->setInputSource(cloud_a_transformed);
  pcl::Correspondences correspondences;
  // Assumes normals for input source aren't needed and there are no correspondence rejectors added to ICP object
  icp.correspondence_estimation_->determineCorrespondences(correspondences, icp.corr_dist_threshold_);
  const int num_correspondences = correspondences.size();
  Eigen::MatrixXd full_jacobian(num_correspondences, 6);
  const auto& target_cloud = icp.target_;
  int index = 0;
  for (const auto correspondence : correspondences) {
    const auto& input_point = (*cloud_a)[correspondence.index_query];
    const auto& target_point = (*target_cloud)[correspondence.index_match];
    if (std::isnan(target_point.normal_x) || std::isnan(target_point.normal_y) || std::isnan(target_point.normal_z))
      continue;
    const Eigen::Matrix<double, 1, 6> jacobian = Jacobian(input_point, target_point, relative_transform);
    if (std::isnan(jacobian(0, 0)) || std::isnan(jacobian(0, 1)) || std::isnan(jacobian(0, 2)) ||
        std::isnan(jacobian(0, 3)) || std::isnan(jacobian(0, 4)) || std::isnan(jacobian(0, 5)))
      continue;
    full_jacobian.block(index++, 0, 1, 6) = jacobian;
  }
  const Eigen::Matrix<double, 6, 6> covariance = (full_jacobian.transpose() * full_jacobian).inverse();
  return covariance;
}
}  // namespace depth_odometry
