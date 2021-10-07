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

#include <depth_odometry/point_cloud_with_known_correspondences_aligner.h>
#include <localization_common/logger.h>
#include <localization_common/timer.h>

namespace depth_odometry {
namespace lc = localization_common;

PointCloudWithKnownCorrespondencesAligner::PointCloudWithKnownCorrespondencesAligner(
  const PointCloudWithKnownCorrespondencesAlignerParams& params)
    : params_(params) {}

Eigen::Isometry3d PointCloudWithKnownCorrespondencesAligner::Align(const std::vector<Eigen::Vector3d>& source_points,
                                                                   const std::vector<Eigen::Vector3d>& target_points,
                                                                   const Eigen::Isometry3d& initial_guess) const {
  Eigen::Matrix<double, 6, 1> relative_transform = VectorFromIsometry3d(initial_guess);
  ceres::Problem problem;
  problem.AddParameterBlock(relative_transform.data(), 6);
  for (int i = 0; i < static_cast<int>(source_points.size()) && i < params_.max_num_matches; ++i) {
    AddCostFunction(source_points[i], target_points[i], relative_transform, problem);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = params_.max_num_iterations;
  options.function_tolerance = params_.function_tolerance;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  return Isometry3(relative_transform.data());
}

lc::PoseWithCovariance PointCloudWithKnownCorrespondencesAligner::ComputeRelativeTransform(
  const std::vector<Eigen::Vector3d>& source_points, const std::vector<Eigen::Vector3d>& target_points) const {
  Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
  // TODO(rsoussan): add option to use umeyama as initial guess
  const Eigen::Isometry3d relative_transform = Align(source_points, target_points, initial_guess);
  return lc::PoseWithCovariance(relative_transform, Eigen::Matrix<double, 6, 6>::Zero());
}

Eigen::Isometry3d PointCloudWithKnownCorrespondencesAligner::ComputeRelativeTransformUmeyama(
  const std::vector<Eigen::Vector3d>& source_points, const std::vector<Eigen::Vector3d>& target_points) const {
  // TODO(rsoussan): clean up naming
  const int npts = static_cast<int>(source_points.size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> cloud_src(3, npts);
  Eigen::Matrix<double, 3, Eigen::Dynamic> cloud_tgt(3, npts);
  for (int i = 0; i < npts; ++i) {
    const auto& source_point = source_points[i];
    cloud_src(0, i) = source_point.x();
    cloud_src(1, i) = source_point.y();
    cloud_src(2, i) = source_point.z();

    const auto& target_point = target_points[i];
    cloud_tgt(0, i) = target_point.x();
    cloud_tgt(1, i) = target_point.y();
    cloud_tgt(2, i) = target_point.z();
  }

  const Eigen::Matrix<double, 4, 4> relative_transform = Eigen::umeyama(cloud_src, cloud_tgt, false);
  return Eigen::Isometry3d(relative_transform.matrix());
}
}  // namespace depth_odometry
