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
#ifndef CAMERA_CAMERA_MODEL_H_
#define CAMERA_CAMERA_MODEL_H_

#include <camera/camera_params.h>

#include <Eigen/Geometry>
#include <string>

namespace camera {

/**
 * A model of a camera, with transformation matrix and camera parameters.
 **/
class CameraModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CameraModel(const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation,
      double fov_x, int x_res, int y_res);
  CameraModel(const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation,
      const camera::CameraParameters & params);
  CameraModel(const Eigen::Affine3d & transform,
      const camera::CameraParameters & params);
  explicit CameraModel(const camera::CameraParameters & params);
  ~CameraModel();

  // Takes world coordinates and output image coordinates that are
  // undistorted and relative to the center of the undistorted image.
  Eigen::Vector2d ImageCoordinates(const Eigen::Vector3d & p) const;
  Eigen::Vector2d ImageCoordinates(double x, double y, double z) const;

  // outputs 3D coordinates in camera frame
  Eigen::Vector3d CameraCoordinates(const Eigen::Vector3d & p) const;
  Eigen::Vector3d CameraCoordinates(double x, double y, double z) const;

  // outputs 3D ray unit vector from image coordinates
  Eigen::Vector3d Ray(int x, int y) const;

  // TODO(oalexan1): This looks buggy. Because ImageCoordinates()
  // returns an undistorted pixel, it must compare to GetUndistortedHalfSize().
  bool IsInFov(const Eigen::Vector3d & p) const;
  bool IsInFov(double x, double y, double z) const;

  double GetFovX(void) const;
  double GetFovY(void) const;

  const camera::CameraParameters& GetParameters() const;
  Eigen::Vector3d GetPosition() const;
  Eigen::Matrix3d GetRotation() const;
  const Eigen::Affine3d& GetTransform() const;
  void SetTransform(const Eigen::Affine3d & cam_t_global);

 private:
  void InitTransform(const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation);
  // The transform cam_t_global_ goes from the world to the camera.
  Eigen::Affine3d cam_t_global_;
  camera::CameraParameters params_;
};

// Rodrigues is a collapsed Angle Axis Representation
void RotationToRodrigues(Eigen::Matrix3d const& rotation, Eigen::Vector3d * vector);
void RodriguesToRotation(Eigen::Vector3d const& vector, Eigen::Matrix3d * rotation);

}  // namespace camera

#endif  // CAMERA_CAMERA_MODEL_H_
