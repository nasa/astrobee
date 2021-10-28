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
#ifndef CALIBRATION_CAMERA_UTILITIES_H_
#define CALIBRATION_CAMERA_UTILITIES_H_

#include <camera/camera_model.h>
#include <optimization_common/residuals.h>
#include <optimization_common/utilities.h>

#include <Eigen/Geometry>

#include <ceres/ceres.h>
#include <ceres/solver.h>

#include <opencv2/calib3d/calib3d.hpp>

#include <boost/optional.hpp>

#include <vector>

namespace calibration {
Eigen::Vector2d Project3dPointToImageSpace(const Eigen::Vector3d& cam_t_point, const Eigen::Matrix3d& intrinsics);

template <typename DISTORTION>
boost::optional<Eigen::Isometry3d> ReprojectionPoseEstimate(const std::vector<Eigen::Vector2d>& image_points,
                                                            const std::vector<Eigen::Vector3d>& points_3d,
                                                            const Eigen::Vector2d& focal_lengths,
                                                            const Eigen::Vector2d& principal_points,
                                                            const Eigen::VectorXd& distortion,
                                                            const int max_num_iterations = 100) {
  ceres::Problem problem;
  // TODO(rsoussan): Avoid all of these const casts?
  problem.AddParameterBlock(const_cast<double*>(focal_lengths.data()), 2);
  problem.SetParameterBlockConstant(const_cast<double*>(focal_lengths.data()));
  problem.AddParameterBlock(const_cast<double*>(principal_points.data()), 2);
  problem.SetParameterBlockConstant(const_cast<double*>(principal_points.data()));
  problem.AddParameterBlock(const_cast<double*>(distortion.data()), DISTORTION::kNumParams);
  problem.SetParameterBlockConstant(const_cast<double*>(distortion.data()));

  Eigen::Isometry3d pose_estimate(Eigen::Isometry3d::Identity());
  Eigen::Matrix<double, 6, 1> pose_estimate_vector = optimization_common::VectorFromIsometry3d(pose_estimate);
  problem.AddParameterBlock(pose_estimate_vector.data(), 6);
  const int num_matches = static_cast<int>(image_points.size());
  for (int i = 0; i < num_matches; ++i) {
    optimization_common::AddReprojectionCostFunction<DISTORTION>(
      image_points[i], points_3d[i], pose_estimate_vector, const_cast<Eigen::Vector2d&>(focal_lengths),
      const_cast<Eigen::Vector2d&>(principal_points), const_cast<Eigen::VectorXd&>(distortion), problem);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  // options.use_explicit_schur_complement = true;
  options.max_num_iterations = max_num_iterations;
  // options.function_tolerance = params_.function_tolerance;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << "\n";
  if (!summary.IsSolutionUsable()) return boost::none;
  return optimization_common::Isometry3(pose_estimate_vector.data());
}

template <typename DISTORTION>
boost::optional<Eigen::Isometry3d> ReprojectionPoseEstimate(const std::vector<Eigen::Vector2d>& image_points,
                                                            const std::vector<Eigen::Vector3d>& points_3d,
                                                            const Eigen::Matrix3d& intrinsics,
                                                            const Eigen::VectorXd& distortion,
                                                            const int max_num_iterations = 100) {
  const Eigen::Vector2d focal_lengths(intrinsics(0, 0), intrinsics(1, 1));
  const Eigen::Vector2d principal_points(intrinsics(0, 2), intrinsics(1, 2));
  return ReprojectionPoseEstimate<DISTORTION>(image_points, points_3d, focal_lengths, principal_points, distortion,
                                              max_num_iterations);
}

template <typename DISTORTION>
Eigen::Vector2d Project3dPointToImageSpaceWithDistortion(const Eigen::Vector3d& cam_t_point,
                                                         const Eigen::Matrix3d& intrinsics,
                                                         const Eigen::VectorXd& distortion_params) {
  const Eigen::Vector2d undistorted_image_point = Project3dPointToImageSpace(cam_t_point, intrinsics);
  const DISTORTION distortion;
  return distortion.Distort(distortion_params, intrinsics, undistorted_image_point);
}
}  // namespace calibration
#endif  // CALIBRATION_CAMERA_UTILITIES_H_
