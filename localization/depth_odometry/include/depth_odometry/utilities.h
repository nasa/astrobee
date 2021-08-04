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
#ifndef DEPTH_ODOMETRY_UTILITIES_H_
#define DEPTH_ODOMETRY_UTILITIES_H_

#include <config_reader/config_reader.h>
#include <depth_odometry/depth_odometry_nodelet_params.h>
#include <depth_odometry/depth_odometry_params.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace depth_odometry {
void LoadDepthOdometryNodeletParams(config_reader::ConfigReader& config, DepthOdometryNodeletParams& params);
void LoadDepthOdometryParams(config_reader::ConfigReader& config, DepthOdometryParams& params);
Eigen::Matrix<double, 1, 6> Jacobian(const gtsam::Point3& point, const gtsam::Vector3& normal,
                                     const gtsam::Pose3& relative_transform);
bool ValidPointNormal(const pcl::PointNormal& point);
void RemoveNansAndZerosFromPointXYZs(pcl::PointCloud<pcl::PointXYZ>& cloud);
void RemoveNansAndZerosFromPointNormals(pcl::PointCloud<pcl::PointNormal>& cloud);

template <typename PointXYZType>
bool ValidPoint(const PointXYZType& point) {
  bool valid_point = true;
  const bool finite_point = pcl_isfinite(point.x) && pcl_isfinite(point.y) && pcl_isfinite(point.z);
  const bool nonzero_point = point.x != 0 || point.y != 0 || point.z != 0;
  valid_point &= finite_point;
  valid_point &= nonzero_point;
  return valid_point;
}

template <typename ValidatorFunction, typename PointType>
void RemoveNansAndZerosFromPointTypes(ValidatorFunction validator, pcl::PointCloud<PointType>& cloud) {
  size_t new_index = 0;
  for (const auto& point : cloud.points) {
    const bool valid_point = validator(point);
    if (!valid_point) continue;
    cloud.points[new_index++] = point;
  }
  if (new_index != cloud.points.size()) {
    cloud.points.resize(new_index);
  }

  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(new_index);
  cloud.is_dense = true;
}
}  // namespace depth_odometry
#endif  // DEPTH_ODOMETRY_UTILITIES_H_
