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

#include <localization_common/test_utilities.h>
#include <point_cloud_common/test_utilities.h>

namespace point_cloud_common {
namespace lc = localization_common;

std::vector<Eigen::Vector3d> RandomPoints(const int num_points) {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < num_points; ++i) {
    points.emplace_back(lc::RandomVector());
  }
  return points;
}

std::vector<Eigen::Vector3d> PlanePoints(const Eigen::Vector3d& point, const Eigen::Vector3d& width_vec,
                                         const Eigen::Vector3d& height_vec, const double width, const double height,
                                         const int num_width_points, const double num_height_points) {
  std::vector<Eigen::Vector3d> plane_points;
  const Eigen::Vector3d top_left_point = point - width / 2.0 * width_vec - height / 2.0 * height_vec;
  const Eigen::Vector3d width_increment = width_vec * width / static_cast<double>(num_width_points);
  const Eigen::Vector3d height_increment = height_vec * height / static_cast<double>(num_height_points);
  for (int row = 0; row < num_height_points; ++row) {
    const Eigen::Vector3d height_plane_point = top_left_point + row * height_increment;
    for (int col = 0; col < num_width_points; ++col) {
      const Eigen::Vector3d plane_point = height_plane_point + col * width_increment;
      plane_points.emplace_back(plane_point);
    }
  }
  return plane_points;
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> RandomPointsWithNormals(const int num_points) {
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> normals;
  for (int i = 0; i < num_points; ++i) {
    points.emplace_back(lc::RandomVector());
    normals.emplace_back(lc::RandomVector().normalized());
  }
  return std::make_pair(points, normals);
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> CubicPoints() {
  std::vector<Eigen::Vector3d> cubic_points;
  std::vector<Eigen::Vector3d> normals;
  const Eigen::Vector3d origin(Eigen::Vector3d::Zero());
  const int num_width_points = 10;
  const int num_height_points = 10;
  const double width = 1.0;
  const double height = 1.0;
  const Eigen::Vector3d x_vec(1, 0, 0);
  const Eigen::Vector3d y_vec(0, 1, 0);
  const Eigen::Vector3d z_vec(0, 0, 1);
  const auto xy_plane_points = PlanePoints(origin, x_vec, y_vec, width, height, num_width_points, num_height_points);
  cubic_points.insert(cubic_points.end(), xy_plane_points.begin(), xy_plane_points.end());
  for (int i = 0; i < xy_plane_points.size(); ++i) normals.emplace_back(z_vec);
  const auto yz_plane_points = PlanePoints(origin, y_vec, z_vec, width, height, num_width_points, num_height_points);
  for (int i = 0; i < yz_plane_points.size(); ++i) normals.emplace_back(x_vec);
  cubic_points.insert(cubic_points.end(), yz_plane_points.begin(), yz_plane_points.end());
  const auto xz_plane_points = PlanePoints(origin, x_vec, z_vec, width, height, num_width_points, num_height_points);
  cubic_points.insert(cubic_points.end(), xz_plane_points.begin(), xz_plane_points.end());
  for (int i = 0; i < xz_plane_points.size(); ++i) normals.emplace_back(y_vec);
  return std::make_pair(cubic_points, normals);
}

pcl::PointXYZ PCLPoint(const Eigen::Vector3d& point) {
  pcl::PointXYZ pcl_point;
  pcl_point.x = point.x();
  pcl_point.y = point.y();
  pcl_point.z = point.z();
  return pcl_point;
}

pcl::PointNormal PCLPointNormal(const Eigen::Vector3d& point, const Eigen::Vector3d& normal) {
  pcl::PointNormal pcl_point;
  pcl_point.x = point.x();
  pcl_point.y = point.y();
  pcl_point.z = point.z();
  pcl_point.normal[0] = normal.x();
  pcl_point.normal[1] = normal.y();
  pcl_point.normal[2] = normal.z();
  return pcl_point;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud(const std::vector<Eigen::Vector3d>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& point : points) {
    cloud->points.emplace_back(PCLPoint(point));
  }
  return cloud;
}

pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudWithNormals(const std::vector<Eigen::Vector3d>& points,
                                                             const std::vector<Eigen::Vector3d>& normals) {
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
  for (int i = 0; i < points.size(); ++i) {
    cloud->points.emplace_back(PCLPointNormal(points[i], normals[i]));
  }
  return cloud;
}

PointToPlaneICPParams DefaultPointToPlaneICPParams() {
  PointToPlaneICPParams params;
  params.search_radius = 0.03;
  params.fitness_threshold = 1;
  params.max_iterations = 10;
  params.symmetric_objective = false;
  params.enforce_same_direction_normals = false;
  params.correspondence_rejector_surface_normal = false;
  params.correspondence_rejector_surface_normal_threshold = 0.75;
  params.downsample = false;
  params.downsample_leaf_size = 0.02;
  params.coarse_to_fine = false;
  params.num_coarse_to_fine_levels = 2;
  params.coarse_to_fine_final_leaf_size = 0.02;
  params.downsample_last_coarse_to_fine_iteration = true;
  return params;
}

PointCloudWithKnownCorrespondencesAlignerParams DefaultPointCloudWithKnownCorrespondencesAlignerParams() {
  PointCloudWithKnownCorrespondencesAlignerParams params;
  params.max_num_iterations = 100;
  params.function_tolerance = 1e-6;
  params.max_num_matches = 1000000;
  params.normal_search_radius = 0.03;
  params.use_umeyama_initial_guess = false;
  params.use_single_iteration_umeyama = false;
  params.use_point_to_plane_cost = false;
  params.use_symmetric_point_to_plane_cost = false;
  params.verbose_optimization = false;
  return params;
}
}  // namespace point_cloud_common
