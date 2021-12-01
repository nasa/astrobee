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
#ifndef POINT_CLOUD_COMMON_POINT_TO_PLANE_ICP_H_
#define POINT_CLOUD_COMMON_POINT_TO_PLANE_ICP_H_

#include <localization_common/logger.h>
#include <localization_common/pose_with_covariance.h>
#include <localization_common/time.h>
#include <localization_common/utilities.h>
#include <point_cloud_common/icp_correspondences.h>
#include <point_cloud_common/point_to_plane_icp_params.h>
#include <point_cloud_common/transformation_estimation_symmetric_point_to_plane_lls.h>
#include <point_cloud_common/utilities.h>

#include <boost/optional.hpp>

// TODO(rsoussan): Switch back to this when PCL bug is fixed
// #include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <point_cloud_common/correspondence_rejection_surface_normal2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <vector>

namespace point_cloud_common {
template <typename PointType>
class PointToPlaneICP {
 public:
  explicit PointToPlaneICP(const PointToPlaneICPParams& params);
  boost::optional<localization_common::PoseWithCovariance> ComputeRelativeTransform(
    const typename pcl::PointCloud<PointType>::Ptr source_cloud_with_normals,
    const typename pcl::PointCloud<PointType>::Ptr target_cloud_with_normals,
    const Eigen::Isometry3d& initial_estimate = Eigen::Isometry3d::Identity());
  const boost::optional<ICPCorrespondences>& correspondences() const;

 private:
  boost::optional<localization_common::PoseWithCovariance> RunPointToPlaneICP(
    const typename pcl::PointCloud<PointType>::Ptr source_cloud_with_normals,
    const typename pcl::PointCloud<PointType>::Ptr target_cloud_with_normals,
    const Eigen::Isometry3d& initial_estimate = Eigen::Isometry3d::Identity());
  boost::optional<localization_common::PoseWithCovariance> RunCoarseToFinePointToPlaneICP(
    const typename pcl::PointCloud<PointType>::Ptr source_cloud_with_normals,
    const typename pcl::PointCloud<PointType>::Ptr target_cloud_with_normals,
    const Eigen::Isometry3d& initial_estimate = Eigen::Isometry3d::Identity());
  boost::optional<localization_common::PoseWithCovariance> RunDownsampledPointToPlaneICP(
    const typename pcl::PointCloud<PointType>::Ptr source_cloud_with_normals,
    const typename pcl::PointCloud<PointType>::Ptr target_cloud_with_normals, const double leaf_size,
    const Eigen::Isometry3d& initial_estimate = Eigen::Isometry3d::Identity());
  void SaveCorrespondences(const pcl::IterativeClosestPointWithNormals<PointType, PointType>& icp,
                           const typename pcl::PointCloud<PointType>::Ptr source_cloud,
                           const typename pcl::PointCloud<PointType>::Ptr source_cloud_transformed);
  boost::optional<ICPCorrespondences> correspondences_;
  PointToPlaneICPParams params_;
};

// Implemenation
template <typename PointType>
PointToPlaneICP<PointType>::PointToPlaneICP(const PointToPlaneICPParams& params) : params_(params) {}

template <typename PointType>
boost::optional<localization_common::PoseWithCovariance> PointToPlaneICP<PointType>::ComputeRelativeTransform(
  const typename pcl::PointCloud<PointType>::Ptr source_cloud_with_normals,
  const typename pcl::PointCloud<PointType>::Ptr target_cloud_with_normals, const Eigen::Isometry3d& initial_estimate) {
  if (params_.coarse_to_fine) {
    return RunCoarseToFinePointToPlaneICP(source_cloud_with_normals, target_cloud_with_normals, initial_estimate);
  } else if (params_.downsample) {
    return RunDownsampledPointToPlaneICP(source_cloud_with_normals, target_cloud_with_normals,
                                         params_.downsample_leaf_size, initial_estimate);
  } else {
    return RunPointToPlaneICP(source_cloud_with_normals, target_cloud_with_normals, initial_estimate);
  }
}

template <typename PointType>
boost::optional<localization_common::PoseWithCovariance> PointToPlaneICP<PointType>::RunPointToPlaneICP(
  const typename pcl::PointCloud<PointType>::Ptr source_cloud_with_normals,
  const typename pcl::PointCloud<PointType>::Ptr target_cloud_with_normals, const Eigen::Isometry3d& initial_estimate) {
  pcl::IterativeClosestPointWithNormals<PointType, PointType> icp;

  if (params_.symmetric_objective) {
    auto symmetric_transformation_estimation =
      boost::make_shared<pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointType, PointType>>();
    symmetric_transformation_estimation->setEnforceSameDirectionNormals(params_.enforce_same_direction_normals);
    icp.transformation_estimation_ = symmetric_transformation_estimation;
  }

  if (params_.correspondence_rejector_surface_normal) {
    typename pcl::registration::CorrespondenceRejectorSurfaceNormal2<PointType>::Ptr
      correspondence_rejector_surface_normal(new pcl::registration::CorrespondenceRejectorSurfaceNormal2<PointType>());
    correspondence_rejector_surface_normal->initializeDataContainer();
    correspondence_rejector_surface_normal->setThreshold(params_.correspondence_rejector_surface_normal_threshold);
    correspondence_rejector_surface_normal->setInputNormals(source_cloud_with_normals);
    correspondence_rejector_surface_normal->setTargetNormals(target_cloud_with_normals);
    icp.addCorrespondenceRejector(correspondence_rejector_surface_normal);
  }

  icp.setInputSource(source_cloud_with_normals);
  icp.setInputTarget(target_cloud_with_normals);
  icp.setMaximumIterations(params_.max_iterations);
  typename pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>);
  icp.align(*result, initial_estimate.matrix().cast<float>());

  if (!icp.hasConverged()) {
    LogError("Icp: Failed to converge.");
    return boost::none;
  }

  const double fitness_score = icp.getFitnessScore();
  if (fitness_score > params_.fitness_threshold) {
    LogError("Icp: Fitness score too large: " << fitness_score << ".");
    return boost::none;
  }

  const Eigen::Isometry3d relative_transform(
    (Eigen::Isometry3f(icp.getFinalTransformation().matrix()).cast<double>()).inverse());
  SaveCorrespondences(icp, source_cloud_with_normals, result);
  const auto covariance =
    PointToPlaneCovariance(correspondences_->source_points, correspondences_->target_normals, relative_transform);
  if (!covariance) {
    LogError("Icp: Failed to get covariance.");
    return boost::none;
  }
  return localization_common::PoseWithCovariance(relative_transform, *covariance);
}

template <typename PointType>
boost::optional<localization_common::PoseWithCovariance> PointToPlaneICP<PointType>::RunDownsampledPointToPlaneICP(
  const typename pcl::PointCloud<PointType>::Ptr source_cloud_with_normals,
  const typename pcl::PointCloud<PointType>::Ptr target_cloud_with_normals, const double leaf_size,
  const Eigen::Isometry3d& initial_estimate) {
  // TODO(rsoussan): Why does template deduction fail without this?
  typename pcl::PointCloud<PointType>::Ptr downsampled_source_cloud_with_normals =
    DownsamplePointCloud<PointType>(source_cloud_with_normals, leaf_size);
  typename pcl::PointCloud<PointType>::Ptr downsampled_target_cloud_with_normals =
    DownsamplePointCloud<PointType>(target_cloud_with_normals, leaf_size);
  return RunPointToPlaneICP(downsampled_source_cloud_with_normals, downsampled_target_cloud_with_normals,
                            initial_estimate);
}

template <typename PointType>
boost::optional<localization_common::PoseWithCovariance> PointToPlaneICP<PointType>::RunCoarseToFinePointToPlaneICP(
  const typename pcl::PointCloud<PointType>::Ptr source_cloud_with_normals,
  const typename pcl::PointCloud<PointType>::Ptr target_cloud_with_normals, const Eigen::Isometry3d& initial_estimate) {
  boost::optional<localization_common::PoseWithCovariance> coarse_to_fine_relative_transform =
    localization_common::PoseWithCovariance(initial_estimate, Eigen::Matrix<double, 6, 6>());
  for (int i = 0; i < params_.num_coarse_to_fine_levels; ++i) {
    // Final iteration and no downsampling enabled for last iteration
    if (i == params_.num_coarse_to_fine_levels - 1 && !params_.downsample_last_coarse_to_fine_iteration) {
      coarse_to_fine_relative_transform = RunPointToPlaneICP(source_cloud_with_normals, target_cloud_with_normals,
                                                             coarse_to_fine_relative_transform->pose);
    } else {  // Downsampled for other iterations
      const double leaf_size =
        static_cast<double>(params_.num_coarse_to_fine_levels - i) * params_.coarse_to_fine_final_leaf_size;
      coarse_to_fine_relative_transform = RunDownsampledPointToPlaneICP(
        source_cloud_with_normals, target_cloud_with_normals, leaf_size, coarse_to_fine_relative_transform->pose);
    }
    if (!coarse_to_fine_relative_transform) {
      LogWarning("RunCoarseToFinePointToPlaneICP: Failed to get relative transform.");
      return boost::none;
    }
  }
  return coarse_to_fine_relative_transform;
}

template <typename PointType>
void PointToPlaneICP<PointType>::SaveCorrespondences(
  const pcl::IterativeClosestPointWithNormals<PointType, PointType>& icp,
  const typename pcl::PointCloud<PointType>::Ptr source_cloud,
  const typename pcl::PointCloud<PointType>::Ptr source_cloud_transformed) {
  icp.correspondence_estimation_->setInputSource(source_cloud_transformed);
  pcl::Correspondences pcl_correspondences;
  // Assumes normals for input source aren't needed and there are no correspondence rejectors added to ICP object
  icp.correspondence_estimation_->determineCorrespondences(pcl_correspondences, icp.corr_dist_threshold_);
  const auto& target_cloud = icp.target_;
  FilterCorrespondences(*source_cloud, *target_cloud, pcl_correspondences);
  std::vector<Eigen::Vector3d> source_points;
  std::vector<Eigen::Vector3d> target_points;
  std::vector<Eigen::Vector3d> target_normals;
  for (const auto& correspondence : pcl_correspondences) {
    const auto& pcl_source_point = (*source_cloud)[correspondence.index_query];
    const auto& pcl_target_point = (*target_cloud)[correspondence.index_match];
    source_points.emplace_back(Vector3d(pcl_source_point));
    target_points.emplace_back(Vector3d(pcl_target_point));
    target_normals.emplace_back(NormalVector3d(pcl_target_point));
  }
  correspondences_ = ICPCorrespondences(source_points, target_points, target_normals);
}

template <typename PointType>
const boost::optional<ICPCorrespondences>& PointToPlaneICP<PointType>::correspondences() const {
  return correspondences_;
}
}  // namespace point_cloud_common

#endif  // POINT_CLOUD_COMMON_POINT_TO_PLANE_ICP_H_
