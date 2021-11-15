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
