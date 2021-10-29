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
#include <localization_common/logger.h>
#include <optimization_common/residuals.h>
#include <optimization_common/utilities.h>

#include <Eigen/Geometry>

#include <ceres/ceres.h>
#include <ceres/solver.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <boost/optional.hpp>

#include <utility>
#include <vector>

namespace calibration {
Eigen::Vector2d Project3dPointToImageSpace(const Eigen::Vector3d& cam_t_point, const Eigen::Matrix3d& intrinsics);

Eigen::Isometry3d Isometry3d(const cv::Mat& rodrigues_rotation_cv, const cv::Mat& translation_cv);

template <typename DISTORTER>
std::pair<Eigen::Isometry3d, Eigen::Vector2i> RansacPnP(const std::vector<Eigen::Vector2d>& image_points,
                                                        const std::vector<Eigen::Vector3d>& points_3d,
                                                        const Eigen::Matrix3d& intrinsics,
                                                        const Eigen::VectorXd& distortion,
                                                        const double min_inlier_threshold = 8.0,
                                                        const int max_num_iterations = 100) {
  // TODO(rsoussan): pass distorter as arg! same with project 3d point?
  const DISTORTER distorter;
  const std::vector<Eigen::Vector2d> undistorted_points = distorter.Undistort(image_points, intrinsics, distortion);
  // TODO(rsoussan): Avoid these looped conversions?
  std::vector<cv::Point2d> undistorted_points_cv;
  for (const auto& undistorted_point : undistorted_points) {
    undistorted_points_cv.emplace_back(cv::Point2d(undistorted_point.x(), undistorted_point.y()));
  }
  std::vector<cv::Point3d> points_3d_cv;
  for (const auto& point_3d : points_3d) {
    points_3d_cv.emplace_back(point_3d.x(), point_3d.y(), point_3d.z());
  }

  cv::Mat intrinsics_cv;
  cv::eigen2cv(intrinsics, intrinsics_cv);
  cv::Mat rodrigues_rotation_cv(3, 1, cv::DataType<double>::type, cv::Scalar(0));
  cv::Mat translation_cv(3, 1, cv::DataType<double>::type, cv::Scalar(0));
  cv::Mat zero_distortion(4, 1, cv::DataType<double>::type, cv::Scalar(0));
  cv::Mat inliers_cv;
  cv::solvePnPRansac(points_3d_cv, undistorted_points_cv, intrinsics_cv, zero_distortion, rodrigues_rotation_cv,
                     translation_cv, false, max_num_iterations, min_inlier_threshold, 0.99, inliers_cv,
                     cv::SOLVEPNP_EPNP);
  Eigen::Vector2i inliers;
  cv::cv2eigen(inliers_cv, inliers);
  const Eigen::Isometry3d pose_estimate = Isometry3d(rodrigues_rotation_cv, translation_cv);
  return std::make_pair(pose_estimate, inliers);
}

template <typename DISTORTER>
boost::optional<Eigen::Isometry3d> ReprojectionPoseEstimate(const std::vector<Eigen::Vector2d>& image_points,
                                                            const std::vector<Eigen::Vector3d>& points_3d,
                                                            const Eigen::Vector2d& focal_lengths,
                                                            const Eigen::Vector2d& principal_points,
                                                            const Eigen::VectorXd& distortion,
                                                            const int max_num_iterations = 100) {
  if (image_points.size() < 4) return boost::none;

  ceres::Problem problem;
  // TODO(rsoussan): Avoid all of these const casts?
  problem.AddParameterBlock(const_cast<double*>(focal_lengths.data()), 2);
  problem.SetParameterBlockConstant(const_cast<double*>(focal_lengths.data()));
  problem.AddParameterBlock(const_cast<double*>(principal_points.data()), 2);
  problem.SetParameterBlockConstant(const_cast<double*>(principal_points.data()));
  problem.AddParameterBlock(const_cast<double*>(distortion.data()), DISTORTER::kNumParams);
  problem.SetParameterBlockConstant(const_cast<double*>(distortion.data()));

  // TODO(rsoussan): pass max its and min thresh args!
  const Eigen::Matrix3d intrinsics = optimization_common::Intrinsics(focal_lengths, principal_points);
  // Use RansacPnP for initial estimate since using identity transform can lead to image projection issues
  // if any points_3d z values are 0.
  const auto initial_estimate = RansacPnP<DISTORTER>(image_points, points_3d, intrinsics, distortion);
  Eigen::Matrix<double, 6, 1> pose_estimate_vector = optimization_common::VectorFromIsometry3d(initial_estimate.first);
  problem.AddParameterBlock(pose_estimate_vector.data(), 6);
  const int num_matches = static_cast<int>(image_points.size());
  for (int i = 0; i < num_matches; ++i) {
    optimization_common::AddReprojectionCostFunction<DISTORTER>(
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
  if (!summary.IsSolutionUsable()) {
    LogError("ReprojectionPoseEstimate: Failed to find solution.");
    return boost::none;
  }
  return optimization_common::Isometry3(pose_estimate_vector.data());
}

template <typename DISTORTER>
boost::optional<Eigen::Isometry3d> ReprojectionPoseEstimate(const std::vector<Eigen::Vector2d>& image_points,
                                                            const std::vector<Eigen::Vector3d>& points_3d,
                                                            const Eigen::Matrix3d& intrinsics,
                                                            const Eigen::VectorXd& distortion,
                                                            const int max_num_iterations = 100) {
  const Eigen::Vector2d focal_lengths(intrinsics(0, 0), intrinsics(1, 1));
  const Eigen::Vector2d principal_points(intrinsics(0, 2), intrinsics(1, 2));
  return ReprojectionPoseEstimate<DISTORTER>(image_points, points_3d, focal_lengths, principal_points, distortion,
                                             max_num_iterations);
}

template <typename DISTORTER>
Eigen::Vector2d Project3dPointToImageSpaceWithDistortion(const Eigen::Vector3d& cam_t_point,
                                                         const Eigen::Matrix3d& intrinsics,
                                                         const Eigen::VectorXd& distortion_params) {
  const Eigen::Vector2d undistorted_image_point = Project3dPointToImageSpace(cam_t_point, intrinsics);
  const DISTORTER distorter;
  return distorter.Distort(distortion_params, intrinsics, undistorted_image_point);
}
}  // namespace calibration
#endif  // CALIBRATION_CAMERA_UTILITIES_H_
