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
#ifndef GRAPH_LOCALIZER_GRAPH_LOCALIZER_PARAMS_H_
#define GRAPH_LOCALIZER_GRAPH_LOCALIZER_PARAMS_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace graph_localizer {
class GraphLocalizerParams {
 public:
  void SetBiases(const Eigen::Vector3d &accelerometer_bias, const Eigen::Vector3d &gyro_bias) {
    accelerometer_bias_ = accelerometer_bias;
    gyro_bias_ = gyro_bias;
  }

  void SetStartPose(const Eigen::Isometry3d &global_T_body_start, const double timestamp) {
    global_T_body_start_ = global_T_body_start;
    start_time_ = timestamp;
  }

  // TODO(rsoussan): rename this?
  void SetCalibration(const Eigen::Isometry3d &body_T_imu, const Eigen::Isometry3d &body_T_nav_cam,
                      const Eigen::Matrix3d &nav_cam_intrinsics, const Eigen::Isometry3d &body_T_dock_cam,
                      const Eigen::Matrix3d &dock_cam_intrinsics, const Eigen::Isometry3d &world_T_dock,
                      const Eigen::Vector3d &gravity) {
    body_T_imu_ = body_T_imu;
    body_T_nav_cam_ = body_T_nav_cam;
    nav_cam_focal_lengths_ = Eigen::Vector2d(nav_cam_intrinsics(0, 0), nav_cam_intrinsics(1, 1));
    nav_cam_principal_point_ = Eigen::Vector2d(nav_cam_intrinsics(0, 2), nav_cam_intrinsics(1, 2));
    body_T_dock_cam_ = body_T_dock_cam;
    dock_cam_focal_lengths_ = Eigen::Vector2d(dock_cam_intrinsics(0, 0), dock_cam_intrinsics(1, 1));
    dock_cam_principal_point_ = Eigen::Vector2d(dock_cam_intrinsics(0, 2), dock_cam_intrinsics(1, 2));
    world_T_dock_ = world_T_dock;
    gravity_ = gravity;
  }

  void SetParams(const double sliding_window_duration, const int min_num_sliding_window_states,
                 const double min_of_avg_distance_from_mean) {
    sliding_window_duration_ = sliding_window_duration;
    min_num_sliding_window_states_ = min_num_sliding_window_states;
    min_of_avg_distance_from_mean_ = min_of_avg_distance_from_mean;
  }

  const double start_time() const { return start_time_; }

  const Eigen::Isometry3d &body_T_imu() const { return body_T_imu_; }
  const Eigen::Isometry3d &body_T_nav_cam() const { return body_T_nav_cam_; }
  const Eigen::Isometry3d &body_T_dock_cam() const { return body_T_dock_cam_; }
  const Eigen::Isometry3d &global_T_body_start() const { return global_T_body_start_; }
  const Eigen::Isometry3d &world_T_dock() const { return world_T_dock_; }
  const Eigen::Vector3d &accelerometer_bias() const { return accelerometer_bias_; }
  const Eigen::Vector3d &gyro_bias() const { return gyro_bias_; }
  const Eigen::Vector2d &nav_cam_focal_lengths() const { return nav_cam_focal_lengths_; }
  const Eigen::Vector2d &nav_cam_principal_point() const { return nav_cam_principal_point_; }
  const Eigen::Vector2d &dock_cam_focal_lengths() const { return dock_cam_focal_lengths_; }
  const Eigen::Vector2d &dock_cam_principal_point() const { return dock_cam_principal_point_; }
  const Eigen::Vector3d &gravity() const { return gravity_; }
  const double sliding_window_duration() const { return sliding_window_duration_; }
  const int min_num_sliding_window_states() const { return min_num_sliding_window_states_; }
  const double min_of_avg_distance_from_mean() const { return min_of_avg_distance_from_mean_; }

 private:
  double start_time_;
  Eigen::Isometry3d body_T_imu_;
  Eigen::Isometry3d body_T_nav_cam_;
  Eigen::Isometry3d body_T_dock_cam_;
  Eigen::Isometry3d world_T_dock_;
  Eigen::Vector3d accelerometer_bias_;
  Eigen::Vector3d gyro_bias_;
  Eigen::Isometry3d global_T_body_start_;
  Eigen::Vector2d nav_cam_focal_lengths_;
  Eigen::Vector2d nav_cam_principal_point_;
  Eigen::Vector2d dock_cam_focal_lengths_;
  Eigen::Vector2d dock_cam_principal_point_;
  Eigen::Vector3d gravity_;
  double sliding_window_duration_;
  int min_num_sliding_window_states_;
  double min_of_avg_distance_from_mean_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_PARAMS_H_
