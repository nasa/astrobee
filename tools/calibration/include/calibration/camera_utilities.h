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

#include <Eigen/Geometry>

#include <opencv2/calib3d/calib3d.hpp>

#include <vector>

namespace calibration {
size_t CountInliersWithDistortion(const std::vector<Eigen::Vector3d>& landmarks,
                                  const std::vector<Eigen::Vector2d>& observations, const camera::CameraModel& camera,
                                  int tolerance, std::vector<size_t>* inliers);
bool P3PIterWithDistortion(const std::vector<cv::Point3d>& landmarks, const std::vector<cv::Point2d>& observations,
                           const camera::CameraParameters& params, const cv::Mat& rvec, const cv::Mat& tvec,
                           Eigen::Vector3d* pos, Eigen::Matrix3d* rotation);
bool P3PWithDistortion(const std::vector<cv::Point3d>& landmarks, const std::vector<cv::Point2d>& observations,
                       const camera::CameraParameters& params, cv::Mat& rvec_result, cv::Mat& tvec_result,
                       Eigen::Vector3d* pos, Eigen::Matrix3d* rotation);
bool RansacEstimateCameraWithDistortion(const std::vector<Eigen::Vector3d>& landmarks,
                                        const std::vector<Eigen::Vector2d>& observations, int num_tries,
                                        int inlier_tolerance, int min_num_inliers, camera::CameraModel* camera_estimate,
                                        std::vector<Eigen::Vector3d>* inlier_landmarks_out = NULL,
                                        std::vector<Eigen::Vector2d>* inlier_observations_out = NULL,
                                        bool verbose = false);
void SelectRandomObservations(const std::vector<Eigen::Vector3d>& all_landmarks,
                              const std::vector<Eigen::Vector2d>& all_observations, size_t num_selected,
                              std::vector<cv::Point3d>* landmarks, std::vector<cv::Point2d>* observations);
// Random integer between min (inclusive) and max (exclusive)
int RandomInt(int min, int max);

Eigen::Vector2d Project3dPointToImageSpace(const Eigen::Vector3d& cam_t_point, const Eigen::Matrix3d& intrinsics);

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
