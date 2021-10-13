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
  if (params_.use_point_to_plane_cost) {
    if (!target_normals_) LogFatal("Align: Attempting to use point to plane cost without having set target normals.");
    for (int i = 0; i < static_cast<int>(source_points.size()) && i < params_.max_num_matches; ++i) {
      AddPointToPlaneCostFunction(source_points[i], target_points[i], (*target_normals_)[i], relative_transform,
                                  problem);
    }
  } else {
    for (int i = 0; i < static_cast<int>(source_points.size()) && i < params_.max_num_matches; ++i) {
      AddPointToPointCostFunction(source_points[i], target_points[i], relative_transform, problem);
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = params_.max_num_iterations;
  options.function_tolerance = params_.function_tolerance;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  {
    // TODO:(rsoussan) clean this up
    static lc::Averager averager("err norm");
    double total_cost = 0.0;
    std::vector<double> err_norm;
    std::vector<double> residuals;
    ceres::Problem::EvaluateOptions eval_options;
    eval_options.num_threads = 1;
    eval_options.apply_loss_function = false;  // want raw residuals
    problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);
    double total_cost_huber = 0.0;
    std::vector<double> err_norm_huber;
    std::vector<double> residuals_huber;
    ceres::Problem::EvaluateOptions eval_options_huber;
    eval_options_huber.num_threads = 1;
    eval_options_huber.apply_loss_function = true;
    problem.Evaluate(eval_options_huber, &total_cost_huber, &residuals_huber, NULL, NULL);
    // size_t len = residuals.size()/2;
    size_t len = residuals.size();
    double res = 0.0;
    double res_huber = 0.0;
    for (size_t it = 0; it < len; it++) {
      // double norm = Eigen::Vector2d(residuals[2*it + 0], residuals[2*it + 1]).norm();
      double norm = residuals[it] * residuals[it];
      res += norm;
      err_norm.push_back(norm);

      averager.Update(norm);
      double norm_huber = residuals_huber[it] * residuals_huber[it];
      res_huber += norm_huber;
      err_norm_huber.push_back(norm_huber);
      if (norm_huber / norm < 0.99) {
        LogError("Huber effect! norm: " << norm << ", hnorm: " << norm_huber);
      }
    }
    res /= len;
    std::sort(err_norm.begin(), err_norm.end());
    averager.Log();
    std::cout << "Initial min, mean, median and max error: " << err_norm[0] << ' ' << res << ' '
              << err_norm[err_norm.size() / 2] << ' ' << err_norm.back() << std::endl;
  }

  return Isometry3(relative_transform.data());
}

lc::PoseWithCovariance PointCloudWithKnownCorrespondencesAligner::ComputeRelativeTransform(
  const std::vector<Eigen::Vector3d>& source_points, const std::vector<Eigen::Vector3d>& target_points) const {
  if (params_.use_single_iteration_umeyama) {
    const Eigen::Isometry3d relative_transform = ComputeRelativeTransformUmeyama(source_points, target_points);
    return lc::PoseWithCovariance(relative_transform, Eigen::Matrix<double, 6, 6>::Zero());
  }

  // Optimize if not using single iteration Umeyama
  const Eigen::Isometry3d initial_guess = params_.use_umeyama_initial_guess
                                            ? ComputeRelativeTransformUmeyama(source_points, target_points)
                                            : Eigen::Isometry3d::Identity();
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

void PointCloudWithKnownCorrespondencesAligner::SetTargetNormals(const std::vector<Eigen::Vector3d>& target_normals) {
  // TODO(rsoussan): Use std::move here?
  target_normals_ = target_normals;
}
}  // namespace depth_odometry
