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

#ifndef VISION_COMMON_INVERSE_DEPTH_MEASUREMENT_H_
#define VISION_COMMON_INVERSE_DEPTH_MEASUREMENT_H_

namespace vision_common {

/**
 * Optimizable inverse depth parameterization for a landmark point using a (u,v)
 * image space measurement and the associated camera pose for the measurement
 * along with the camera intrinsics matrix to be able to backproject the inverse depth
 * point to a 3d point.
 */

class InverseDepthMeasurement {
  InverseDepthMeasurement(const double inverse_depth, const Eigen::Vector2d& image_coordinates,
                          const Eigen::Matrix3d& intrinsics, const Pose3& body_T_sensor)
      : inverse_depth_(inverse_depth),
        image_coordinates_(image_coordinates),
        intrinsics_(intrinsics),
        body_T_sensor_(body_T_sensor) {}

  Eigen::Vector3d Backproject(boost::optional<Matrix&> d_backprojected_point_d_inverse_depth = boost::none) const {
    return vision_common::Backproject(image_coordinates_, intrinsics_, depth(), d_projected_point_d_world_T_source);
  }

  boost::optional<Eigen::Vector2d> Project(const gtsam::Pose3& world_T_source, const gtsam::Pose3& world_T_target,
                                           boost::optional<Matrix&> d_projected_point_d_world_T_source = boost::none,
                                           boost::optional<Matrix&> d_projected_point_d_world_T_target = boost::none,
                                           boost::optional<Matrix&> d_projected_point_d_inverse_depth = boost::none) {
    if (d_projected_point_d_world_T_source || d_projected_point_d_world_T_target || d_projected_point_d_inverse_depth) {
      // Jacobian Calculations:
      // projected_point = (project(target_T_source*backproject(inverse_depth)))
      // call backproject(inverse_depth) = source_t_point
      // call target_T_source* source_t_point = target_t_point
      // Pose Jacobians:
      // d_projected_point_d_world_T_source = d_project_d_target_t_point * d_target_t_point_d_world_T_source
      // d_projected_point_d_world_T_target = d_project_d_target_t_point * d_target_t_point_d_world_T_target
      // where
      // d_target_t_point_d_world_T_source = d_target_t_point_d_target_T_source * d_target_T_source_d_world_T_source
      // d_target_t_point_d_world_T_target = d_target_t_point_d_target_T_source * d_target_T_source_d_world_T_target
      // d_target_T_source_d_world_T_target = d_target_T_source_d_target_T_world * d_target_T_world_d_world_T_target
      // Inverse Depth Jacobian:
      // d_projected_point_d_inverse_depth = d_project_d_target_t_point * d_target_t_point_d_inverse_depth
      // where
      // d_target_t_point_d_inverse_depth = d_target_t_point_d_source_t_point * d_source_t_point_d_inverse_depth

      // Intermediate Jacobians
      Matrix d_source_t_point_d_inverse_depth;
      Matrix d_target_T_world_d_world_T_target;
      Matrix d_target_T_source_d_world_T_source;
      Matrix d_target_T_source_d_target_T_world;
      Matrix d_target_t_point_d_target_T_source;
      Matrix d_target_t_point_d_source_t_point;
      Matrix d_project_d_target_t_point;
      const auto projeced_point =
        Project(world_T_source, world_T_target, d_source_t_point_d_inverse_depth, d_target_T_source_d_target_T_world,
                d_target_T_source_d_world_T_source, d_target_t_point_d_target_T_source,
                d_target_t_point_d_source_t_point, d_project_d_target_t_point);
      // Final pose Jacobians
      const Matrix d_target_T_source_d_world_T_target =
        d_target_T_source_d_target_T_world * d_target_T_world_d_world_T_target;
      const Matrix d_target_t_point_d_world_T_source =
        d_target_t_point_d_target_T_source * d_target_T_source_d_world_T_source;
      const Matrix d_target_t_point_d_world_T_target =
        d_target_t_point_d_target_T_source * d_target_T_source_d_world_T_target;
      if (d_projected_point_d_world_T_source)
        d_projected_point_d_world_T_source = d_project_d_target_t_point * d_target_t_point_d_world_T_source;
      if (d_projected_point_d_world_T_target)
        d_projected_point_d_world_T_target = d_project_d_target_t_point * d_target_t_point_d_world_T_target;
      // Final inverse depth Jacobian
      const Matrix d_target_t_point_d_inverse_depth =
        d_target_t_point_d_source_t_point * d_source_t_point_d_inverse_depth;
      if (d_projected_point_d_inverse_depth)
        d_projected_point_d_inverse_depth = d_project_d_target_t_point * d_target_t_point_d_inverse_depth;
      return projection;
    }

    // Jacobians not required
    return Project(world_T_source, world_T_target);
  }

  double depth() const { return 1.0 / inverse_depth(); }

  double inverse_depth() const { return inverse_depth_; }

 private:
  // Intermediate call to optionally fill required jacobians.  Allows for code reuse whether the jacobians are need or
  // not.
  boost::optional<Eigen::Vector2d> Project(const gtsam::Pose3& world_T_source, const gtsam::Pose3& world_T_target,
                                           boost::optional<Matrix&> d_source_t_point_d_inverse_depth = boost::none,
                                           boost::optional<Matrix&> d_target_T_source_d_target_T_world = boost::none,
                                           boost::optional<Matrix&> d_target_T_source_d_world_T_source = boost::none,
                                           boost::optional<Matrix&> d_target_t_point_d_target_T_source = boost::none,
                                           boost::optional<Matrix&> d_target_t_point_d_source_t_point = boost::none,
                                           boost::optional<Matrix&> d_project_d_target_t_point = boost::none) {
    const Eigen::Vector3d source_t_point = Backproject(d_source_t_point_d_inverse_depth);
    const gtsam::Pose3 target_T_world = world_T_target.inverse(d_target_T_world_d_world_T_target);
    const gtsam::Pose3 target_T_source =
      target_T_world.compose(world_T_source, d_target_T_source_d_target_T_world, d_target_T_source_d_world_T_source);
    const Eigen::Vector3d target_t_point =
      target_T_source.compose(source_t_point, d_target_t_point_d_target_T_source, d_target_t_point_d_source_t_point);
    if (target_t_point.z() < 0) return boost::none;
    return vision_common::Project(target_t_point, intrinsics_, d_project_d_target_t_point);
  }

  Eigen::Vector2d image_coordinates_;
  Eigen::Matrix3d intrinsics_;
  Pose3 body_T_sensor_;
  double inverse_depth_;
}

}  // namespace vision_common

#endif  // VISION_COMMON_INVERSE_DEPTH_MEASUREMENT_H_
