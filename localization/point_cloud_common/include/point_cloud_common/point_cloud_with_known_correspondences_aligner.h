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
#ifndef POINT_CLOUD_COMMON_POINT_CLOUD_WITH_KNOWN_CORRESPONDENCES_ALIGNER_H_
#define POINT_CLOUD_COMMON_POINT_CLOUD_WITH_KNOWN_CORRESPONDENCES_ALIGNER_H_

#include <localization_common/pose_with_covariance.h>
#include <point_cloud_common/point_cloud_with_known_correspondences_aligner_params.h>

#include <Eigen/Core>

#include <boost/optional.hpp>

#include <vector>

namespace point_cloud_common {
class PointCloudWithKnownCorrespondencesAligner {
 public:
  explicit PointCloudWithKnownCorrespondencesAligner(const PointCloudWithKnownCorrespondencesAlignerParams& params);

  Eigen::Isometry3d Align(
    const std::vector<Eigen::Vector3d>& source_points, const std::vector<Eigen::Vector3d>& target_points,
    const Eigen::Isometry3d& initial_source_T_target_estimate,
    const boost::optional<const std::vector<Eigen::Vector3d>&> source_normals = boost::none,
    const boost::optional<const std::vector<Eigen::Vector3d>&> target_normals = boost::none) const;

  boost::optional<localization_common::PoseWithCovariance> ComputeRelativeTransform(
    const std::vector<Eigen::Vector3d>& source_points, const std::vector<Eigen::Vector3d>& target_points,
    const boost::optional<const std::vector<Eigen::Vector3d>&> source_normals = boost::none,
    const boost::optional<const std::vector<Eigen::Vector3d>&> target_normals = boost::none,
    const Eigen::Isometry3d& initial_source_T_target_estimate = Eigen::Isometry3d::Identity()) const;

  boost::optional<localization_common::PoseWithCovariance> ComputeRelativeTransform(
    const std::vector<Eigen::Vector3d>& source_points, const std::vector<Eigen::Vector3d>& target_points,
    const Eigen::Isometry3d& initial_source_T_target_estimate = Eigen::Isometry3d::Identity()) const;

 private:
  PointCloudWithKnownCorrespondencesAlignerParams params_;
};
}  // namespace point_cloud_common

#endif  // POINT_CLOUD_COMMON_POINT_CLOUD_WITH_KNOWN_CORRESPONDENCES_ALIGNER_H_
