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
#include <calibration/camera_utilities2.h>
#include <calibration/camera_utilities3.h>
#include <calibration/reprojection_pose_estimate_params.h>
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
  LogError("image points size in repojposeestimate: " << image_points.size());
  // test!!!!
  boost::optional<Eigen::Isometry3d> old_estimate;
  {
    LogError("Getting estimates!");
    Eigen::Isometry3d camera_T_target(Eigen::Isometry3d::Identity());
    // tODO: right order for image size????
    camera::CameraParameters camera(Eigen::Vector2i(1280, 960), focal_lengths, principal_points, distortion);
    camera::CameraModel cam_model(camera_T_target, camera);
    if (!RansacEstimateCameraWithDistortion(points_3d, image_points, params.ransac_pnp.num_iterations,
                                            params.ransac_pnp.max_inlier_threshold, params.ransac_pnp.min_num_inliers,
                                            &cam_model)) {
      LogError("Old RansacEstimteCamera failed to get pose!");
    } else {
      old_estimate = Eigen::Isometry3d(cam_model.GetTransform().matrix());
      LogError("Old RansacEstimateCamera pose: " << std::endl << old_estimate->matrix());
    }
  }
  ////

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
  const auto initial_estimate_and_inliers =
    RansacPnP2<DISTORTER>(image_points, points_3d, intrinsics, distortion, params.ransac_pnp.max_inlier_threshold,
                          params.ransac_pnp.num_iterations, params.ransac_pnp.min_num_inliers);
  if (!initial_estimate_and_inliers) {
    LogError("ReprojectionPoseEstimate: Failed to get initial estimate.");
    return boost::none;
  }

  if (!old_estimate) return boost::none;
  Eigen::Matrix<double, 6, 1> pose_estimate_vector =
    // optimization_common::VectorFromIsometry3d(initial_estimate_and_inliers->first);
    optimization_common::VectorFromIsometry3d(*old_estimate);
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

  if (old_estimate) LogError("Old estimate again: " << std::endl << old_estimate->matrix());
  LogError("New initial estimate: " << std::endl << initial_estimate_and_inliers->first.matrix());

  LogError("New ReprojectionPoseEstimate: " << std::endl
                                            << optimization_common::Isometry3(pose_estimate_vector.data()).matrix());
  // return optimization_common::Isometry3(pose_estimate_vector.data());
  // CreateReprojectionImage<DISTORTER>(image_points, points_3d, initial_estimate_and_inliers->second, intrinsics,
  //                                 distortion, initial_estimate_and_inliers->first, "new_reprojposeestimate");
  return old_estimate;
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
