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

#include <calibration/camera_utilities.h>

#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <random>
#include <unordered_map>

namespace calibration {
bool P3PWithDistortion(const std::vector<cv::Point3d>& landmarks, const std::vector<cv::Point2d>& observations,
                       const camera::CameraParameters& params, cv::Mat& rvec_result, cv::Mat& tvec_result,
                       Eigen::Vector3d* pos, Eigen::Matrix3d* rotation) {
  cv::Mat camera_matrix(3, 3, cv::DataType<double>::type);
  cv::eigen2cv(params.GetIntrinsicMatrix<camera::DISTORTED>(), camera_matrix);
  cv::Mat rvec(3, 1, cv::DataType<double>::type, cv::Scalar(0));
  cv::Mat tvec(3, 1, cv::DataType<double>::type, cv::Scalar(0));
  const auto distortion_params = params.GetDistortion();
  cv::Mat distortion = cv::Mat::zeros(4, 1, cv::DataType<double>::type);
  for (int i = 0; i < distortion_params.size(); ++i) {
    distortion.at<double>(i, 0) = distortion_params[i];
  }

  /*for (int i = 0; i < landmarks.size(); ++i){
  cv::Mat zero_r(cv::Mat::eye(3, 3, cv::DataType<double>::type));
  cv::Mat zero_t(cv::Mat::zeros(3, 1, cv::DataType<double>::type));
  std::vector<cv::Point2d> projected_points;
  std::vector<cv::Point3d> object_points;
  const auto& point_3d = landmarks[i];
  object_points.emplace_back(point_3d);
  cv::projectPoints(object_points, zero_r, zero_t, camera_matrix, distortion, projected_points);
  std::cout << "obs: " << observations[i] << std::endl;
  std::cout << "pro: " << projected_points[0] << std::endl;
  }

  std::cout << "intrinsics: " << camera_matrix << std::endl;
  std::cout << "distortion: " << distortion << std::endl;*/

  bool result = cv::solvePnP(landmarks, observations, camera_matrix, distortion, rvec, tvec, false, cv::SOLVEPNP_P3P);
  if (!result) {
    return false;
  }
  /* result = cv::solvePnP(landmarks, observations, camera_matrix, distortion, rvec, tvec, true,
    cv::SOLVEPNP_ITERATIVE); if (!result) return false;*/

  cv::cv2eigen(tvec, *pos);
  camera::RodriguesToRotation(Eigen::Vector3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)), rotation);
  rvec_result = rvec.clone();
  tvec_result = tvec.clone();
  return true;
}

bool P3PIterWithDistortion(const std::vector<cv::Point3d>& landmarks, const std::vector<cv::Point2d>& observations,
                           const camera::CameraParameters& params, const cv::Mat& rvec, const cv::Mat& tvec,
                           Eigen::Vector3d* pos, Eigen::Matrix3d* rotation) {
  cv::Mat camera_matrix(3, 3, cv::DataType<double>::type);
  cv::eigen2cv(params.GetIntrinsicMatrix<camera::DISTORTED>(), camera_matrix);
  const auto distortion_params = params.GetDistortion();
  cv::Mat distortion(cv::Mat::zeros(4, 1, cv::DataType<double>::type));
  for (int i = 0; i < distortion_params.size(); ++i) {
    distortion.at<double>(i, 0) = distortion_params[i];
  }

  // std::cout << "intrinsics: " << camera_matrix << std::endl;
  // std::cout << "distortion: " << distortion << std::endl;

  const bool result =
    cv::solvePnP(landmarks, observations, camera_matrix, distortion, rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
  if (!result) return false;

  cv::cv2eigen(tvec, *pos);
  camera::RodriguesToRotation(Eigen::Vector3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)), rotation);
  return true;
}

size_t CountInliersWithDistortion(const std::vector<Eigen::Vector3d>& landmarks,
                                  const std::vector<Eigen::Vector2d>& observations, const camera::CameraModel& camera,
                                  int tolerance, std::vector<size_t>* inliers) {
  int num_inliers = 0;
  if (inliers) {
    // To save ourselves some allocation time. We'll prealloc for a 50% inlier
    // success rate
    inliers->reserve(observations.size() / 2);
  }

  double tolerance_sq = tolerance * tolerance;

  for (size_t i = 0; i < landmarks.size(); i++) {
    Eigen::Vector2d pos = camera.DistortedImageCoordinates(landmarks[i]);
    if ((observations[i] - pos).squaredNorm() <= tolerance_sq) {
      /*std::cout << "pos: " << pos.x() << ", " << pos.y() << std::endl;
      std::cout << "obs: " << observations[i].x() << ", " << observations[i].y() << std::endl;*/
      const double norm = (observations[i] - pos).squaredNorm();
      // std::cout << "norm: " << norm << std::endl;*/
      if (norm > 9) {
        std::cout << "max norm: " << tolerance_sq << std::endl;
        std::cout << "repo large norm inlier!!" << std::endl;
      }

      num_inliers++;
      if (inliers) inliers->push_back(i);
    }
  }
  return num_inliers;
}

bool RansacEstimateCameraWithDistortion(const std::vector<Eigen::Vector3d>& landmarks,
                                        const std::vector<Eigen::Vector2d>& observations, int num_tries,
                                        int inlier_tolerance, int min_num_inliers, camera::CameraModel* camera_estimate,
                                        std::vector<Eigen::Vector3d>* inlier_landmarks_out,
                                        std::vector<Eigen::Vector2d>* inlier_observations_out, bool verbose) {
  size_t best_inliers = 0;
  camera::CameraParameters params = camera_estimate->GetParameters();

  // Need the minimum number of observations
  if (observations.size() < 4) {
    return false;
  }

  // RANSAC to find the best camera with P3P
  std::vector<cv::Point3d> subset_landmarks;
  std::vector<cv::Point2d> subset_observations;
  Eigen::Vector3d best_pos;
  Eigen::Matrix3d best_rotation;
  cv::Mat best_rvec;
  cv::Mat best_tvec;

  for (int i = 0; i < num_tries; i++) {
    subset_landmarks.clear();
    subset_observations.clear();
    SelectRandomObservations(landmarks, observations, 4, &subset_landmarks, &subset_observations);

    Eigen::Vector3d pos;
    Eigen::Matrix3d rotation;
    bool result =
      P3PWithDistortion(subset_landmarks, subset_observations, params, best_rvec, best_tvec, &pos, &rotation);
    if (!result) continue;
    Eigen::Affine3d cam_t_global;
    cam_t_global.setIdentity();
    cam_t_global.translate(pos);
    cam_t_global.rotate(rotation);
    camera::CameraModel guess(cam_t_global, camera_estimate->GetParameters());

    size_t inliers = CountInliersWithDistortion(landmarks, observations, guess, inlier_tolerance, NULL);
    if (inliers > best_inliers) {
      best_inliers = inliers;
      *camera_estimate = guess;
      best_pos = pos;
      best_rotation = rotation;
    }
  }

  if (verbose) std::cout << observations.size() << " Ransac observations " << best_inliers << " inliers\n";

  // TODO(bcoltin): Return some sort of confidence?
  if (best_inliers < min_num_inliers) {
    return false;
  }

  std::vector<size_t> inliers;
  CountInliersWithDistortion(landmarks, observations, *camera_estimate, inlier_tolerance, &inliers);
  std::vector<Eigen::Vector3d> inlier_landmarks;
  std::vector<Eigen::Vector2d> inlier_observations;
  std::vector<cv::Point3d> cv_inlier_landmarks;
  std::vector<cv::Point2d> cv_inlier_observations;

  inlier_landmarks.reserve(inliers.size());
  inlier_observations.reserve(inliers.size());
  for (size_t idx : inliers) {
    inlier_landmarks.push_back(landmarks[idx]);
    inlier_observations.push_back(observations[idx]);
    cv_inlier_landmarks.push_back(cv::Point3d(landmarks[idx][0], landmarks[idx][1], landmarks[idx][2]));
    cv_inlier_observations.push_back(cv::Point2d(observations[idx][0], observations[idx][1]));
  }

  /*{
     bool result = P3PIterWithDistortion(cv_inlier_landmarks, cv_inlier_observations, params, best_rvec, best_tvec,
   &best_pos, &best_rotation); if (!result) return false; Eigen::Affine3d cam_t_global; cam_t_global.setIdentity();
     cam_t_global.translate(best_pos);
     cam_t_global.rotate(best_rotation);
     camera::CameraModel guess(cam_t_global, camera_estimate->GetParameters());
     *camera_estimate = guess;
   }*/
  /*ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = 1;  // it is no slower with only one thread
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  // improve estimate with CERES solver
  EstimateCamera(camera_estimate, &inlier_landmarks, inlier_observations, options, &summary);*/

  // find inliers again with refined estimate
  inliers.clear();
  best_inliers = CountInliersWithDistortion(landmarks, observations, *camera_estimate, inlier_tolerance, &inliers);

  if (verbose) std::cout << "Number of inliers with refined camera: " << best_inliers << "\n";

  if (best_inliers < min_num_inliers) {
    return false;
  }

  inlier_landmarks.clear();
  inlier_observations.clear();
  inlier_landmarks.reserve(inliers.size());
  inlier_observations.reserve(inliers.size());
  for (size_t idx : inliers) {
    inlier_landmarks.push_back(landmarks[idx]);
    inlier_observations.push_back(observations[idx]);
  }
  if (inlier_landmarks_out) {
    inlier_landmarks_out->reserve(inliers.size());
    std::copy(inlier_landmarks.begin(), inlier_landmarks.end(), std::back_inserter(*inlier_landmarks_out));
  }
  if (inlier_observations_out) {
    inlier_observations_out->reserve(inliers.size());
    std::copy(inlier_observations.begin(), inlier_observations.end(), std::back_inserter(*inlier_observations_out));
  }

  for (size_t i = 0; i < inlier_landmarks.size(); i++) {
    Eigen::Vector2d pos = camera_estimate->DistortedImageCoordinates(inlier_landmarks[i]);
    /*std::cout << "pos: " << pos.x() << ", " << pos.y() << std::endl;
    std::cout << "obs: " << observations[i].x() << ", " << observations[i].y() << std::endl;*/
    const double norm = (inlier_observations[i] - pos).squaredNorm();
    // std::cout << "norm: " << norm << std::endl;*/
    if (norm > 9) {
      std::cout << "result large norm inlier!!" << std::endl;
    }
  }
  return true;
}
void SelectRandomObservations(const std::vector<Eigen::Vector3d>& all_landmarks,
                              const std::vector<Eigen::Vector2d>& all_observations, size_t num_selected,
                              std::vector<cv::Point3d>* landmarks, std::vector<cv::Point2d>* observations) {
  std::unordered_map<int, int> used;
  // not enough observations
  if (all_observations.size() < num_selected) return;
  // Reserve space in the output so we don't have to keep reallocating on
  // push_back().
  landmarks->reserve(num_selected);
  observations->reserve(num_selected);
  while (observations->size() < num_selected) {
    int id = RandomInt(0, all_observations.size());
    if (used.count(id) > 0) continue;
    Eigen::Vector3d p = all_landmarks[id];
    landmarks->push_back(cv::Point3d(p[0], p[1], p[2]));
    observations->push_back(cv::Point2d(all_observations[id][0], all_observations[id][1]));
    used[id] = 1;
  }
}
// random intger in [min, max)
int RandomInt(int min, int max) {
  static std::mt19937 generator;  // should be thread_local for thread safe, gcc 4.6 doesn't support
  std::uniform_int_distribution<int> random_item(min, max - 1);
  return random_item(generator);
}

Eigen::Vector2d Project3dPointToImageSpace(const Eigen::Vector3d& cam_t_point, const Eigen::Matrix3d& intrinsics) {
  return (intrinsics * cam_t_point).hnormalized();
}
}  // namespace calibration
