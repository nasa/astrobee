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

#include <localization_common/logger.h>
#include <localization_common/timer.h>
#include <localization_common/utilities.h>
#include <point_cloud_common/point_to_plane_icp.h>
#include <point_cloud_common/transformation_estimation_symmetric_point_to_plane_lls.h>
#include <point_cloud_common/utilities.h>

#include <gtsam/geometry/Pose3.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
// TODO(rsoussan): Switch back to this when PCL bug is fixed
// #include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <point_cloud_common/correspondence_rejection_surface_normal2.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace point_cloud_common {
namespace lc = localization_common;

PointToPlaneICP::PointToPlaneICP(const PointToPlaneICPParams& params) : params_(params) {}

boost::optional<lc::PoseWithCovariance> PointToPlaneICP::ComputeRelativeTransform(
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud_with_normals,
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud_with_normals,
  const Eigen::Isometry3d& initial_estimate) {
  if (params_.coarse_to_fine) {
    return RunCoarseToFinePointToPlaneICP(source_cloud_with_normals, target_cloud_with_normals, initial_estimate);
  } else {
    return RunPointToPlaneICP(source_cloud_with_normals, target_cloud_with_normals, initial_estimate);
  }
}

boost::optional<lc::PoseWithCovariance> PointToPlaneICP::RunPointToPlaneICP(
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud_with_normals,
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud_with_normals,
  const Eigen::Isometry3d& initial_estimate) {
  static lc::Timer icp_timer("PointToPlaneICP");
  icp_timer.Start();
  pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;

  if (params_.symmetric_objective) {
    auto symmetric_transformation_estimation =
      boost::make_shared<pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<pcl::PointXYZINormal,
                                                                                             pcl::PointXYZINormal>>();
    symmetric_transformation_estimation->setEnforceSameDirectionNormals(params_.enforce_same_direction_normals);
    icp.transformation_estimation_ = symmetric_transformation_estimation;
  }

  if (params_.correspondence_rejector_surface_normal) {
    pcl::registration::CorrespondenceRejectorSurfaceNormal2::Ptr correspondence_rejector_surface_normal(
      new pcl::registration::CorrespondenceRejectorSurfaceNormal2());
    correspondence_rejector_surface_normal->initializeDataContainer<pcl::PointXYZI, pcl::PointXYZINormal>();
    correspondence_rejector_surface_normal->setThreshold(params_.correspondence_rejector_surface_normal_threshold);
    correspondence_rejector_surface_normal->setInputNormals<pcl::PointXYZI, pcl::PointXYZINormal>(
      source_cloud_with_normals);
    correspondence_rejector_surface_normal->setTargetNormals<pcl::PointXYZI, pcl::PointXYZINormal>(
      target_cloud_with_normals);
    icp.addCorrespondenceRejector(correspondence_rejector_surface_normal);
  }

  icp.setInputSource(source_cloud_with_normals);
  icp.setInputTarget(target_cloud_with_normals);
  icp.setMaximumIterations(params_.max_iterations);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZINormal>);
  icp.align(*result, initial_estimate.matrix().cast<float>());

  if (!icp.hasConverged()) {
    LogError("Icp: Failed to converge.");
    icp_timer.Stop();
    return boost::none;
  }

  const double fitness_score = icp.getFitnessScore();
  if (fitness_score > params_.fitness_threshold) {
    LogError("Icp: Fitness score too large: " << fitness_score << ".");
    icp_timer.Stop();
    return boost::none;
  }

  // TODO(rsoussan): clean this up
  // TODO(rsoussan): don't take inverse??
  const Eigen::Isometry3d relative_transform(
    (Eigen::Isometry3f(icp.getFinalTransformation().matrix()).cast<double>()).inverse());
  const Eigen::Matrix<double, 6, 6> covariance =
    ComputeCovarianceMatrix(icp, source_cloud_with_normals, result, relative_transform);
  icp_timer.StopAndLog();
  return lc::PoseWithCovariance(relative_transform, covariance);
}

boost::optional<lc::PoseWithCovariance> PointToPlaneICP::RunCoarseToFinePointToPlaneICP(
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud_with_normals,
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud_with_normals,
  const Eigen::Isometry3d& initial_estimate) {
  static lc::Timer coarse_to_fine_icp_timer("Coarse To Fine PointToPlaneICP");
  coarse_to_fine_icp_timer.Start();
  boost::optional<lc::PoseWithCovariance> latest_relative_transform =
    lc::PoseWithCovariance(initial_estimate, Eigen::Matrix<double, 6, 6>());
  for (int i = 0; i < params_.num_coarse_to_fine_levels; ++i) {
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr downsampled_source_cloud_with_normals;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr downsampled_target_cloud_with_normals;
    if (i == params_.num_coarse_to_fine_levels - 1 && !params_.downsample_last_coarse_to_fine_iteration) {
      downsampled_source_cloud_with_normals = source_cloud_with_normals;
      downsampled_target_cloud_with_normals = target_cloud_with_normals;
    } else {
      const double leaf_size_ratio =
        static_cast<double>(params_.num_coarse_to_fine_levels) / static_cast<double>(i + 1.0);
      const double leaf_size = leaf_size_ratio * params_.coarse_to_fine_final_leaf_size;
      // TODO(rsoussan): Why does template deduction fail without this?
      downsampled_source_cloud_with_normals =
        DownsamplePointCloud<pcl::PointXYZINormal>(source_cloud_with_normals, leaf_size);
      downsampled_target_cloud_with_normals =
        DownsamplePointCloud<pcl::PointXYZINormal>(target_cloud_with_normals, leaf_size);
    }
    latest_relative_transform = RunPointToPlaneICP(
      downsampled_source_cloud_with_normals, downsampled_target_cloud_with_normals, latest_relative_transform->pose);
    if (!latest_relative_transform) {
      LogWarning("RunCoarseToFinePointToPlaneICP: Failed to get relative transform.");
      coarse_to_fine_icp_timer.StopAndLog();
      return boost::none;
    }
  }
  coarse_to_fine_icp_timer.StopAndLog();
  return latest_relative_transform;
}

// TODO(rsoussan): Move this to utilities!!
Eigen::Matrix<double, 6, 6> PointToPlaneICP::ComputeCovarianceMatrix(
  const pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>& icp,
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud,
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud_transformed,
  const Eigen::Isometry3d& relative_transform) {
  icp.correspondence_estimation_->setInputSource(source_cloud_transformed);
  correspondences_ = pcl::Correspondences();
  // Assumes normals for input source aren't needed and there are no correspondence rejectors added to ICP object
  icp.correspondence_estimation_->determineCorrespondences(*correspondences_, icp.corr_dist_threshold_);
  const auto& target_cloud = icp.target_;
  FilterCorrespondences(*source_cloud, *target_cloud, *correspondences_);
  const int num_correspondences = correspondences_->size();
  Eigen::MatrixXd full_jacobian(num_correspondences, 6);
  int index = 0;
  for (const auto correspondence : *correspondences_) {
    const auto& input_point = (*source_cloud)[correspondence.index_query];
    const auto& target_point = (*target_cloud)[correspondence.index_match];
    const Eigen::Matrix<double, 1, 6> jacobian = PointToPlaneJacobian(input_point, target_point, relative_transform);
    if (std::isnan(jacobian(0, 0)) || std::isnan(jacobian(0, 1)) || std::isnan(jacobian(0, 2)) ||
        std::isnan(jacobian(0, 3)) || std::isnan(jacobian(0, 4)) || std::isnan(jacobian(0, 5)))
      continue;
    full_jacobian.block(index++, 0, 1, 6) = jacobian;
  }
  const Eigen::Matrix<double, 6, 6> covariance = (full_jacobian.transpose() * full_jacobian).inverse();
  return covariance;
}
}  // namespace point_cloud_common
