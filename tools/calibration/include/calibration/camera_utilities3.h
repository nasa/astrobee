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
#ifndef CALIBRATION_CAMERA_UTILITIES3_H_
#define CALIBRATION_CAMERA_UTILITIES3_H_

#include <calibration/ransac_pnp_params.h>
#include <localization_common/logger.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <boost/optional.hpp>

#include <unordered_set>
#include <utility>
#include <vector>

namespace calibration {
void UndistortedPnP(const std::vector<cv::Point2d>& undistorted_image_points, const std::vector<cv::Point3d>& points_3d,
                    const cv::Mat& intrinsics, const int pnp_method, cv::Mat& rotation, cv::Mat& translation);

std::vector<int> RandomNIndices(const int num_possible_indices, const int num_sampled_indices);

// Remove This
Eigen::Vector2d Project3dPointToImageSpace2(const Eigen::Vector3d& cam_t_point, const Eigen::Matrix3d& intrinsics);

// Remove this
Eigen::Isometry3d Isometry3d2(const cv::Mat& rodrigues_rotation_cv, const cv::Mat& translation_cv);

// TODO(rsoussan): Remove this
template <typename DISTORTER>
Eigen::Vector2d Project3dPointToImageSpaceWithDistortion2(const Eigen::Vector3d& cam_t_point,
                                                          const Eigen::Matrix3d& intrinsics,
                                                          const Eigen::VectorXd& distortion_params) {
  const Eigen::Vector2d undistorted_image_point = Project3dPointToImageSpace2(cam_t_point, intrinsics);
  const DISTORTER distorter;
  return distorter.Distort(distortion_params, intrinsics, undistorted_image_point);
}

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
      Project3dPointToImageSpaceWithDistortion2<DISTORTER>(transformed_point_3d, intrinsics, distortion);
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
boost::optional<std::pair<Eigen::Isometry3d, std::vector<int>>> RansacPnP2(
  const std::vector<Eigen::Vector2d>& image_points, const std::vector<Eigen::Vector3d>& points_3d,
  const Eigen::Matrix3d& intrinsics, const Eigen::VectorXd& distortion, const RansacPnPParams& params) {
  if (image_points.size() < 4) {
    LogError("RansacPnP2: Too few matched points given.");
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
  // Remove this
  std::vector<int> best_indices;
  // Use 4 values for P3P
  constexpr int kNumSampledValues = 4;
  for (int i = 0; i < params.num_iterations; ++i) {
    const auto sampled_indices = RandomNIndices(image_points.size(), kNumSampledValues);
    const auto sampled_undistorted_image_points = SampledValues(undistorted_image_points_cv, sampled_indices);
    const auto sampled_points_3d = SampledValues(points_3d_cv, sampled_indices);
    UndistortedPnP(sampled_undistorted_image_points, sampled_points_3d, intrinsics_cv, params.pnp_method,
                   rodrigues_rotation_cv, translation_cv);
    const Eigen::Isometry3d pose_estimate = Isometry3d2(rodrigues_rotation_cv, translation_cv);
    const int num_inliers =
      Inliers<DISTORTER>(image_points, points_3d, intrinsics, distortion, pose_estimate, params.max_inlier_threshold);
    if (num_inliers > max_num_inliers) {
      best_pose_estimate = pose_estimate;
      max_num_inliers = num_inliers;
      // Remove this
      best_indices = sampled_indices;
    }
  }

  if (max_num_inliers < params.min_num_inliers) {
    LogError("RansacPnP: Failed to find pose with enough inliers.");
    return boost::none;
  }

  LogError("New RansacPnP indices used: " << best_indices.size());
  for (int i = 0; i < static_cast<int>(best_indices.size()); ++i) {
    std::cout << best_indices[i] << " ";
  }
  std::cout << std::endl;

  std::vector<int> inliers;
  Inliers<DISTORTER>(image_points, points_3d, intrinsics, distortion, best_pose_estimate, params.max_inlier_threshold,
                     inliers);
  LogError("New RansacPnP num inliers: " << inliers.size());
  for (int i = 0; i < static_cast<int>(inliers.size()); ++i) {
    std::cout << inliers[i] << " ";
  }
  std::cout << std::endl;

  return std::make_pair(best_pose_estimate, inliers);
}
}  // namespace calibration
#endif  // CALIBRATION_CAMERA_UTILITIES3_H_
