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

#include <localization_common/logger.h>
#include <localization_common/timer.h>
#include <optimization_common/residuals.h>
#include <optimization_common/utilities.h>
#include <point_cloud_common/point_cloud_with_known_correspondences_aligner.h>
#include <point_cloud_common/utilities.h>

namespace point_cloud_common {
namespace lc = localization_common;
namespace oc = optimization_common;

PointCloudWithKnownCorrespondencesAligner::PointCloudWithKnownCorrespondencesAligner(
  const PointCloudWithKnownCorrespondencesAlignerParams& params)
    : params_(params) {}

Eigen::Isometry3d PointCloudWithKnownCorrespondencesAligner::Align(
  const std::vector<Eigen::Vector3d>& source_points, const std::vector<Eigen::Vector3d>& target_points,
  const Eigen::Isometry3d& initial_guess, const boost::optional<const std::vector<Eigen::Vector3d>&> source_normals,
  const boost::optional<const std::vector<Eigen::Vector3d>&> target_normals) const {
  Eigen::Matrix<double, 6, 1> relative_transform = oc::VectorFromIsometry3d(initial_guess);
  ceres::Problem problem;
  problem.AddParameterBlock(relative_transform.data(), 6);
  if (params_.use_symmetric_point_to_plane_cost) {
    if (!target_normals || !source_normals)
      LogFatal(
        "Align: Attempting to use symmetric point to plane cost without having valid source and target normals.");
    for (int i = 0; i < static_cast<int>(source_points.size()) && i < params_.max_num_matches; ++i) {
      oc::SymmetricPointToPlaneError::AddCostFunction(source_points[i], target_points[i], (*source_normals)[i],
                                                      (*target_normals)[i], relative_transform, problem);
    }
  } else if (params_.use_point_to_plane_cost) {
    if (!target_normals) LogFatal("Align: Attempting to use point to plane cost without having valid target normals.");
    for (int i = 0; i < static_cast<int>(source_points.size()) && i < params_.max_num_matches; ++i) {
      oc::PointToPlaneError::AddCostFunction(source_points[i], target_points[i], (*target_normals)[i],
                                             relative_transform, problem);
    }
  } else {
    for (int i = 0; i < static_cast<int>(source_points.size()) && i < params_.max_num_matches; ++i) {
      oc::PointToPointError::AddCostFunction(source_points[i], target_points[i], relative_transform, problem);
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = params_.max_num_iterations;
  options.function_tolerance = params_.function_tolerance;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (params_.verbose_optimization) {
    std::cout << summary.FullReport() << "\n";
    int residual_size;
    // TODO(rsousssan): Template on this? add somewhere else?
    if (params_.use_symmetric_point_to_plane_cost) {
      residual_size = 2;
    } else if (params_.use_point_to_plane_cost) {
      residual_size = 1;
    } else {
      residual_size = 3;
    }
    oc::CheckResiduals(residual_size, problem);
  }
  return oc::Isometry3d(relative_transform);
}

lc::PoseWithCovariance PointCloudWithKnownCorrespondencesAligner::ComputeRelativeTransform(
  const std::vector<Eigen::Vector3d>& source_points, const std::vector<Eigen::Vector3d>& target_points,
  const boost::optional<const std::vector<Eigen::Vector3d>&> source_normals,
  const boost::optional<const std::vector<Eigen::Vector3d>&> target_normals) const {
  if (params_.use_single_iteration_umeyama) {
    const Eigen::Isometry3d relative_transform = RelativeTransformUmeyama(source_points, target_points);
    const lc::PoseCovariance covariance = PointToPointCovariance(source_points, relative_transform);
    return lc::PoseWithCovariance(relative_transform, covariance);
  }

  const Eigen::Isometry3d initial_guess = params_.use_umeyama_initial_guess
                                            ? RelativeTransformUmeyama(source_points, target_points)
                                            : Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d relative_transform =
    Align(source_points, target_points, initial_guess, source_normals, target_normals);
  // TODO(rsoussan): Allow for covariances for point to plane and symmetric point to plane
  const lc::PoseCovariance covariance = PointToPointCovariance(source_points, relative_transform);
  return lc::PoseWithCovariance(relative_transform, covariance);
}
}  // namespace point_cloud_common
