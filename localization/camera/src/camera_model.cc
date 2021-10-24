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

#include <camera/camera_model.h>

#include <opencv2/calib3d.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>


namespace camera {

CameraModel::CameraModel(const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation,
                         double fov_x, int x_res, int y_res) :
  params_(Eigen::Vector2i(x_res, y_res),
          Eigen::Vector2d::Constant(1.0 / (tan(fov_x / 2) / (x_res / 2.0))),
          Eigen::Vector2d(x_res, y_res) / 2) {
  InitTransform(position, rotation);
}

CameraModel::CameraModel(const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation,
        const camera::CameraParameters & params) : params_(params) {
  InitTransform(position, rotation);
}

CameraModel::CameraModel(const camera::CameraParameters & params) : params_(params) {
  InitTransform(Eigen::Vector3d(0, 0, 0), Eigen::Matrix3d::Identity());
}

CameraModel::CameraModel(const Eigen::Affine3d & cam_t_global,
                         const camera::CameraParameters & params) :
  cam_t_global_(cam_t_global), params_(params) {}

void CameraModel::InitTransform(const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation) {
  cam_t_global_.setIdentity();
  cam_t_global_.translate(-(rotation * position));
  cam_t_global_.rotate(rotation);
}

CameraModel::~CameraModel() {}

Eigen::Vector3d CameraModel::GetPosition() const {
  return -cam_t_global_.rotation().inverse() * cam_t_global_.translation();
}

Eigen::Matrix3d CameraModel::GetRotation() const {
  return cam_t_global_.rotation();
}

const Eigen::Affine3d& CameraModel::GetTransform() const {
  return cam_t_global_;
}

void CameraModel::SetTransform(const Eigen::Affine3d & cam_t_global) {
  cam_t_global_ = cam_t_global;
}

double CameraModel::GetFovX(void) const {
  // This is an approximation since it doesn't take in account lens distortion
  return atan(1.0 / (params_.GetFocalVector()[0] * params_.GetDistortedHalfSize()[0])) * 2;
}

double CameraModel::GetFovY(void) const {
  // This is an approximation since it doesn't take in account lens distortion
  return atan(1.0 / (params_.GetFocalVector()[1] * params_.GetDistortedHalfSize()[1])) * 2;
}

const camera::CameraParameters& CameraModel::GetParameters() const {
  return params_;
}

Eigen::Vector2d CameraModel::ImageCoordinates(double x, double y, double z) const {
  return ImageCoordinates(Eigen::Vector3d(x, y, z));
}

Eigen::Vector2d CameraModel::ImageCoordinates(const Eigen::Vector3d & p) const {
  // This returns undistorted pixels relative to the center of the undistorted image.
  return params_.GetFocalVector().cwiseProduct((cam_t_global_ * p).hnormalized());
}

Eigen::Vector2d CameraModel::DistortedImageCoordinates(const Eigen::Vector3d & p) const {
  // TODO(rsoussan): clean this up!
  const auto focal_lengths = params_.GetFocalVector();
  const auto distortion_params = params_.GetDistortion();
  const auto principal_points = params_.GetOpticalOffset();
  cv::Mat intrinsics = (cv::Mat::zeros(3, 3, cv::DataType<double>::type));
  intrinsics.at<double>(0, 0) = focal_lengths[0];
  intrinsics.at<double>(1, 1) = focal_lengths[1];
  intrinsics.at<double>(0, 2) = principal_points[0];
  intrinsics.at<double>(1, 2) = principal_points[1];
  intrinsics.at<double>(2, 2) = 1;
  cv::Mat distortion = (cv::Mat::zeros(4, 1, cv::DataType<double>::type));
  for (int i = 0; i < distortion_params.size(); ++i) {
    distortion.at<double>(i, 0) = distortion_params[i];
  }

  cv::Mat zero_r(cv::Mat::eye(3, 3, cv::DataType<double>::type));
  cv::Mat zero_t(cv::Mat::zeros(3, 1, cv::DataType<double>::type));
  std::vector<cv::Point2d> projected_points;
  std::vector<cv::Point3d> object_points;
  const auto& frame_changed_pt = cam_t_global_ * p;
  object_points.emplace_back(cv::Point3d(frame_changed_pt.x(), frame_changed_pt.y(), frame_changed_pt.z()));
  cv::projectPoints(object_points, zero_r, zero_t, intrinsics, distortion, projected_points);
  const auto& projected_point = projected_points[0];
  return Eigen::Vector2d(projected_point.x, projected_point.y);
}

Eigen::Vector3d CameraModel::Ray(int x, int y) const {
  return cam_t_global_.rotation().inverse() * Eigen::Vector3d(x / params_.GetFocalVector()[0],
      y / params_.GetFocalVector()[1], 1.0).normalized();
}

Eigen::Vector3d CameraModel::CameraCoordinates(double x, double y, double z) const {
  return CameraCoordinates(Eigen::Vector3d(x, y, z));
}

Eigen::Vector3d CameraModel::CameraCoordinates(const Eigen::Vector3d & p) const {
  return cam_t_global_ * p;
}

// TODO(oalexan1): This looks buggy. Because ImageCoordinates()
// returns an undistorted pixel, it must compare to GetUndistortedHalfSize().
bool CameraModel::IsInFov(const Eigen::Vector3d & p) const {
  Eigen::Vector3d t = cam_t_global_ * p;
  if (t.z() <= 0.0)
    return false;
  Eigen::Vector2d camera = ImageCoordinates(p);
  if (camera.x() < -params_.GetDistortedHalfSize()[0] ||
      camera.x() >= params_.GetDistortedHalfSize()[0])
    return false;
  if (camera.y() < -params_.GetDistortedHalfSize()[1] ||
      camera.y() >= params_.GetDistortedHalfSize()[1])
    return false;
  return true;
}

bool CameraModel::IsInFov(double x, double y, double z) const {
  return IsInFov(Eigen::Vector3d(x, y, z));
}

// Rodrigues is a collapsed Angle Axis Representation
void RotationToRodrigues(Eigen::Matrix3d const& rotation,
                         Eigen::Vector3d * vector) {
  Eigen::AngleAxisd aa(rotation);
  *vector = aa.axis();
  vector->normalize();
  *vector *= aa.angle();
}

void RodriguesToRotation(Eigen::Vector3d const& vector,
                         Eigen::Matrix3d * rotation) {
  double angle = vector.norm();
  Eigen::AngleAxisd aa(angle, vector / angle);
  *rotation = aa.matrix();
}

}  // namespace camera
