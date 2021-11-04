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

#include <calibration/ransac_pnp_params.h>
#include <calibration/reprojection_pose_estimate_params.h>
#include <camera/camera_model.h>
#include <ff_common/eigen_vectors.h>
#include <localization_common/logger.h>
#include <optimization_common/residuals.h>
#include <optimization_common/utilities.h>

#include <Eigen/Geometry>

#include <ceres/ceres.h>
#include <ceres/solver.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <boost/optional.hpp>

#include <algorithm>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace calibration {
Eigen::Vector2d Project3dPointToImageSpace(const Eigen::Vector3d& cam_t_point, const Eigen::Matrix3d& intrinsics);

template <typename DISTORTER>
Eigen::Vector2d Project3dPointToImageSpaceWithDistortion(const Eigen::Vector3d& cam_t_point,
                                                         const Eigen::Matrix3d& intrinsics,
                                                         const Eigen::VectorXd& distortion_params) {
  const Eigen::Vector2d undistorted_image_point = Project3dPointToImageSpace(cam_t_point, intrinsics);
  const DISTORTER distorter;
  return distorter.Distort(distortion_params, intrinsics, undistorted_image_point);
}

template <typename DISTORTER>
void CreateReprojectionImage(const std::vector<Eigen::Vector2d>& image_points,
                             const std::vector<Eigen::Vector3d>& points_3d, const Eigen::VectorXi& indicies,
                             const Eigen::Matrix3d& intrinsics, const Eigen::VectorXd& distortion,
                             const Eigen::Isometry3d& pose, const std::string& name) {
  const double max_error_norm = 100;
  cv::Mat reprojection_image_grayscale(960, 1280, CV_8UC1, cv::Scalar(0));
  for (int i = 0; i < indicies.size(); ++i) {
    const int index = indicies[i];
    const Eigen::Vector2d& image_point = image_points[index];
    const Eigen::Vector3d& point_3d = points_3d[index];
    const Eigen::Vector3d camera_t_target_point = pose * point_3d;
    const Eigen::Vector2d projected_image_point =
      Project3dPointToImageSpaceWithDistortion<DISTORTER>(camera_t_target_point, intrinsics, distortion);
    const Eigen::Vector2d error = (image_point - projected_image_point);
    const double error_norm = error.norm();
    const cv::Point2i rounded_image_point(std::round(image_point.x()), std::round(image_point.y()));
    // Add 1 to each value so background pixels stay white and we can map these back to white
    // after applying colormap.
    // Only map up to 235 since darker reds that occur from 235-255 are hard to differentiate from
    // darker blues from 0 to 20 or so.
    const int error_color = std::round(std::min(error_norm, max_error_norm) / max_error_norm * 235.0) + 1;
    cv::circle(reprojection_image_grayscale, rounded_image_point, 4, cv::Scalar(235), -1);
    cv::circle(reprojection_image_grayscale,
               cv::Point2i(std::round(projected_image_point.x()), std::round(projected_image_point.y())), 4,
               cv::Scalar(error_color), -1);
  }
  cv::Mat reprojection_image_color;
  cv::applyColorMap(reprojection_image_grayscale, reprojection_image_color, cv::COLORMAP_JET);
  // Map white pixels back from lowest JET value (128, 0, 0) to white
  cv::Mat base_mask;
  cv::inRange(reprojection_image_color, cv::Scalar(128, 0, 0), cv::Scalar(128, 0, 0), base_mask);
  reprojection_image_color.setTo(cv::Scalar(255, 255, 255), base_mask);
  // cv::imshow(name, reprojection_image_color);
  // cv::waitKey(0);
}

Eigen::Isometry3d Isometry3d(const cv::Mat& rodrigues_rotation_cv, const cv::Mat& translation_cv);

void UndistortedPnP(const std::vector<cv::Point2d>& undistorted_image_points, const std::vector<cv::Point3d>& points_3d,
                    const cv::Mat& intrinsics, const int pnp_method, cv::Mat& rotation, cv::Mat& translation);

std::vector<int> RandomNIndices(const int num_possible_indices, const int num_sampled_indices);

template <typename DISTORTER>
int Inliers(const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& points_3d,
            const Eigen::Matrix3d& intrinsics, const Eigen::VectorXd& distortion,
            const Eigen::Isometry3d& pose_estimate, const double max_inlier_threshold,
            boost::optional<std::vector<int>&> inliers = boost::none) {
  int num_inliers = 0;
  for (int i = 0; i < static_cast<int>(image_points.size()); ++i) {
    const Eigen::Vector2d& image_point = image_points[i];
    const Eigen::Vector3d& point_3d = points_3d[i];
    const Eigen::Vector3d transformed_point_3d = pose_estimate * point_3d;
    const Eigen::Vector2d projected_image_point =
      Project3dPointToImageSpaceWithDistortion<DISTORTER>(transformed_point_3d, intrinsics, distortion);
    const Eigen::Vector2d error = (image_point - projected_image_point);
    const double error_norm = error.norm();
    if (error_norm <= max_inlier_threshold) {
      ++num_inliers;
      if (inliers) inliers->emplace_back(i);
    }
  }
  return num_inliers;
}

template <typename T>
std::vector<T> SampledValues(const std::vector<T>& values, const std::vector<int>& indices) {
  std::vector<T> sampled_values;
  for (const int i : indices) {
    sampled_values.emplace_back(values[i]);
  }
  return sampled_values;
}

template <typename DISTORTER>
boost::optional<std::pair<Eigen::Isometry3d, std::vector<int>>> RansacPnP(
  const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& points_3d,
  const Eigen::Matrix3d& intrinsics, const Eigen::VectorXd& distortion, const RansacPnPParams& params) {
  if (image_points.size() < 4) {
    LogError("RansacPnP: Too few matched points given.");
    return boost::none;
  }

  // TODO(rsoussan): pass distorter as arg! same with project 3d point?
  const DISTORTER distorter;
  const std::vector<Eigen::Vector2d> undistorted_image_points =
    distorter.Undistort(image_points, intrinsics, distortion);

  // TODO(rsoussan): make function to randomly populate these!!
  // TODO(rsoussan): Avoid these looped conversions?
  std::vector<cv::Point2d> undistorted_image_points_cv;
  for (const auto& undistorted_image_point : undistorted_image_points) {
    undistorted_image_points_cv.emplace_back(cv::Point2d(undistorted_image_point.x(), undistorted_image_point.y()));
  }
  std::vector<cv::Point3d> points_3d_cv;
  for (const auto& point_3d : points_3d) {
    points_3d_cv.emplace_back(point_3d.x(), point_3d.y(), point_3d.z());
  }

  cv::Mat intrinsics_cv;
  cv::eigen2cv(intrinsics, intrinsics_cv);
  cv::Mat rodrigues_rotation_cv(3, 1, cv::DataType<double>::type, cv::Scalar(0));
  cv::Mat translation_cv(3, 1, cv::DataType<double>::type, cv::Scalar(0));
  int max_num_inliers = 0;
  Eigen::Isometry3d best_pose_estimate;
  // Use 4 values for P3P
  // TODO(rsoussan): make this a param
  constexpr int kNumSampledValues = 4;
  for (int i = 0; i < params.num_iterations; ++i) {
    const auto sampled_indices = RandomNIndices(image_points.size(), kNumSampledValues);
    const auto sampled_undistorted_image_points = SampledValues(undistorted_image_points_cv, sampled_indices);
    const auto sampled_points_3d = SampledValues(points_3d_cv, sampled_indices);
    UndistortedPnP(sampled_undistorted_image_points, sampled_points_3d, intrinsics_cv, params.pnp_method,
                   rodrigues_rotation_cv, translation_cv);
    const Eigen::Isometry3d pose_estimate = Isometry3d(rodrigues_rotation_cv, translation_cv);
    const int num_inliers =
      Inliers<DISTORTER>(image_points, points_3d, intrinsics, distortion, pose_estimate, params.max_inlier_threshold);
    if (num_inliers > max_num_inliers) {
      best_pose_estimate = pose_estimate;
      max_num_inliers = num_inliers;
    }
  }

  if (max_num_inliers < params.min_num_inliers) {
    LogError("RansacPnP: Failed to find pose with enough inliers.");
    return boost::none;
  }

  std::vector<int> inliers;
  Inliers<DISTORTER>(image_points, points_3d, intrinsics, distortion, best_pose_estimate, params.max_inlier_threshold,
                     inliers);
  return std::make_pair(best_pose_estimate, inliers);
}

template <typename DISTORTER>
boost::optional<Eigen::Isometry3d> ReprojectionPoseEstimate(const std::vector<Eigen::Vector2d>& image_points,
                                                            const std::vector<Eigen::Vector3d>& points_3d,
                                                            const Eigen::Vector2d& focal_lengths,
                                                            const Eigen::Vector2d& principal_points,
                                                            const Eigen::VectorXd& distortion,
                                                            const ReprojectionPoseEstimateParams& params) {
  if (image_points.size() < 4) {
    LogError("ReprojectionPoseEstimate: Too few matched points given.");
    return boost::none;
  }

  ceres::Problem problem;
  // TODO(rsoussan): Avoid all of these const casts?
  problem.AddParameterBlock(const_cast<double*>(focal_lengths.data()), 2);
  problem.SetParameterBlockConstant(const_cast<double*>(focal_lengths.data()));
  problem.AddParameterBlock(const_cast<double*>(principal_points.data()), 2);
  problem.SetParameterBlockConstant(const_cast<double*>(principal_points.data()));
  problem.AddParameterBlock(const_cast<double*>(distortion.data()), DISTORTER::kNumParams);
  problem.SetParameterBlockConstant(const_cast<double*>(distortion.data()));

  const Eigen::Matrix3d intrinsics = optimization_common::Intrinsics(focal_lengths, principal_points);
  // Use RansacPnP for initial estimate since using identity transform can lead to image projection issues
  // if any points_3d z values are 0.
  const auto initial_estimate_and_inliers =
    RansacPnP<DISTORTER>(image_points, points_3d, intrinsics, distortion, params.ransac_pnp);
  if (!initial_estimate_and_inliers) {
    LogError("ReprojectionPoseEstimate: Failed to get initial estimate.");
    return boost::none;
  }

  Eigen::Matrix<double, 6, 1> pose_estimate_vector =
    optimization_common::VectorFromIsometry3d(initial_estimate_and_inliers->first);
  problem.AddParameterBlock(pose_estimate_vector.data(), 6);
  const int num_inliers = initial_estimate_and_inliers->second.size();
  if (num_inliers < params.ransac_pnp.min_num_inliers) {
    LogError("ReprojectionPoseEstimate: Too few inliers found. Need " << params.ransac_pnp.min_num_inliers << ", got "
                                                                      << num_inliers << ".");
    return boost::none;
  }

  for (int i = 0; i < num_inliers; ++i) {
    const int inlier_index = initial_estimate_and_inliers->second[i];
    optimization_common::AddReprojectionCostFunction<DISTORTER>(
      image_points[inlier_index], points_3d[inlier_index], pose_estimate_vector,
      const_cast<Eigen::Vector2d&>(focal_lengths), const_cast<Eigen::Vector2d&>(principal_points),
      const_cast<Eigen::VectorXd&>(distortion), problem, 1, params.optimization.huber_loss);
  }

  ceres::Solver::Summary summary;
  ceres::Solve(params.optimization.solver_options, &problem, &summary);
  // std::cout << summary.FullReport() << "\n";
  if (!summary.IsSolutionUsable()) {
    LogError("ReprojectionPoseEstimate: Failed to find solution.");
    return boost::none;
  }

  // CreateReprojectionImage<DISTORTER>(image_points, points_3d, initial_estimate_and_inliers->second, intrinsics,
  //                                 distortion, initial_estimate_and_inliers->first, "new_reprojposeestimate");
  return optimization_common::Isometry3(pose_estimate_vector.data());
}

template <typename DISTORTER>
boost::optional<Eigen::Isometry3d> ReprojectionPoseEstimate(const std::vector<Eigen::Vector2d>& image_points,
                                                            const std::vector<Eigen::Vector3d>& points_3d,
                                                            const Eigen::Matrix3d& intrinsics,
                                                            const Eigen::VectorXd& distortion,
                                                            const double min_inlier_threshold = 4.0,
                                                            const int ransac_num_iterations = 100) {
  const Eigen::Vector2d focal_lengths(intrinsics(0, 0), intrinsics(1, 1));
  const Eigen::Vector2d principal_points(intrinsics(0, 2), intrinsics(1, 2));
  return ReprojectionPoseEstimate<DISTORTER>(image_points, points_3d, focal_lengths, principal_points, distortion,
                                             ransac_num_iterations);
}

}  // namespace calibration
#endif  // CALIBRATION_CAMERA_UTILITIES_H_
