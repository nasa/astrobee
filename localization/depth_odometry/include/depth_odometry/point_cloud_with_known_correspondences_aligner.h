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
#ifndef DEPTH_ODOMETRY_POINT_CLOUD_WITH_KNOWN_CORRESPONDENCES_ALIGNER_H_
#define DEPTH_ODOMETRY_POINT_CLOUD_WITH_KNOWN_CORRESPONDENCES_ALIGNER_H_

#include <depth_odometry/point_cloud_with_known_correspondences_aligner_params.h>
#include <depth_odometry/optimization_utilities.h>
#include <localization_common/pose_with_covariance.h>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/optional.hpp>

namespace depth_odometry {
class PointCloudWithKnownCorrespondencesAligner {
 public:
  PointCloudWithKnownCorrespondencesAligner(const PointCloudWithKnownCorrespondencesAlignerParams& params);

  Eigen::Isometry3d Align(const std::vector<Eigen::Vector3d>& source_points,
                          const std::vector<Eigen::Vector3d>& target_points,
                          const Eigen::Isometry3d& initial_guess) const;

  localization_common::PoseWithCovariance ComputeRelativeTransform(
    const std::vector<Eigen::Vector3d>& source_points, const std::vector<Eigen::Vector3d>& target_points) const;

  Eigen::Isometry3d ComputeRelativeTransformUmeyama(const std::vector<Eigen::Vector3d>& source_points,
                                                    const std::vector<Eigen::Vector3d>& target_points) const;

  void SetSourceNormals(const std::vector<Eigen::Vector3d>& source_normals);

  void SetTargetNormals(const std::vector<Eigen::Vector3d>& target_normals);

 private:
  PointCloudWithKnownCorrespondencesAlignerParams params_;
  boost::optional<std::vector<Eigen::Vector3d>> source_normals_;
  boost::optional<std::vector<Eigen::Vector3d>> target_normals_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_POINT_CLOUD_WITH_KNOWN_CORRESPONDENCES_ALIGNER_H_
