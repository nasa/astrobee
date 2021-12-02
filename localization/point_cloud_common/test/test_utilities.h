/* Copyright (c) 2017, United S/ates Government, as represented by the
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
#ifndef POINT_CLOUD_COMMON_TEST_UTILITIES_H_  // NOLINT
#define POINT_CLOUD_COMMON_TEST_UTILITIES_H_  // NOLINT

#include <ff_common/eigen_vectors.h>
#include <point_cloud_common/point_to_plane_icp_params.h>
#include <point_cloud_common/point_cloud_with_known_correspondences_aligner.h>

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <utility>
#include <vector>

namespace point_cloud_common {
std::vector<Eigen::Vector3d> RandomPoints(const int num_points);

// Assumes width and height vecs are normalized and define the plane attached to point
std::vector<Eigen::Vector3d> PlanePoints(const Eigen::Vector3d& point, const Eigen::Vector3d& width_vec,
                                         const Eigen::Vector3d& height_vec, const double width, const double height,
                                         const int num_width_points, const double num_height_points);

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> RandomPointsWithNormals(const int num_points);

// Returns points on three unqiue planes covering half of a cube
std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> CubicPoints();

pcl::PointXYZ PCLPoint(const Eigen::Vector3d& point);

pcl::PointNormal PCLPointNormal(const Eigen::Vector3d& point, const Eigen::Vector3d& normal);

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud(const std::vector<Eigen::Vector3d>& points);

pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudWithNormals(const std::vector<Eigen::Vector3d>& points,
                                                             const std::vector<Eigen::Vector3d>& normals);

PointToPlaneICPParams DefaultPointToPlaneICPParams();

PointCloudWithKnownCorrespondencesAlignerParams DefaultPointCloudWithKnownCorrespondencesAlignerParams();
}  // namespace point_cloud_common
#endif  // POINT_CLOUD_COMMON_TEST_UTILITIES_H_  // NOLINT
