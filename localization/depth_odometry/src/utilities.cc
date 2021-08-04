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

#include <depth_odometry/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

namespace depth_odometry {
namespace mc = msg_conversions;

void LoadDepthOdometryNodeletParams(config_reader::ConfigReader& config, DepthOdometryNodeletParams& params) {
  params.publish_point_clouds = mc::LoadBool(config, "publish_point_clouds");
}

void LoadDepthOdometryParams(config_reader::ConfigReader& config, DepthOdometryParams& params) {
  params.position_covariance_threshold = mc::LoadDouble(config, "position_covariance_threshold");
  params.orientation_covariance_threshold = mc::LoadDouble(config, "orientation_covariance_threshold");
  params.fitness_threshold = mc::LoadDouble(config, "fitness_threshold");
  params.search_radius = mc::LoadDouble(config, "search_radius");
  params.max_iterations = mc::LoadInt(config, "max_iterations");
  params.symmetric_objective = mc::LoadBool(config, "symmetric_objective");
  params.enforce_same_direction_normals = mc::LoadBool(config, "enforce_same_direction_normals");
  params.inital_estimate_with_ransac_ia = mc::LoadBool(config, "inital_estimate_with_ransac_ia");
  params.correspondence_rejector_surface_normal = mc::LoadBool(config, "correspondence_rejector_surface_normal");
  params.correspondence_rejector_surface_normal_threshold =
    mc::LoadDouble(config, "correspondence_rejector_surface_normal_threshold");
  params.frame_change_transform = mc::LoadBool(config, "frame_change_transform");
  params.body_T_haz_cam = msg_conversions::LoadEigenTransform(config, "haz_cam_transform");
}

Eigen::Matrix<double, 1, 6> Jacobian(const gtsam::Point3& point, const gtsam::Vector3& normal,
                                     const gtsam::Pose3& relative_transform) {
  gtsam::Matrix H1;
  relative_transform.transformFrom(point, H1);
  return normal.transpose() * H1;
}

bool ValidPointNormal(const pcl::PointNormal& point) {
  bool valid_point = true;
  valid_point &= ValidPoint(point);
  const bool finite_normal =
    pcl_isfinite(point.normal_x) && pcl_isfinite(point.normal_y) && pcl_isfinite(point.normal_z);
  const bool nonzero_normal = point.normal_x != 0 || point.normal_y != 0 || point.normal_z != 0;
  valid_point &= finite_normal;
  valid_point &= nonzero_normal;
  return valid_point;
}

void RemoveNansAndZerosFromPointXYZs(pcl::PointCloud<pcl::PointXYZ>& cloud) {
  RemoveNansAndZerosFromPointTypes(ValidPoint<pcl::PointXYZ>, cloud);
}

void RemoveNansAndZerosFromPointNormals(pcl::PointCloud<pcl::PointNormal>& cloud) {
  RemoveNansAndZerosFromPointTypes(ValidPointNormal, cloud);
}
}  // namespace depth_odometry
