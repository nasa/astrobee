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
#include <point_cloud_common/icp.h>
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

ICP::ICP(const ICPParams& params) : params_(params) {}

boost::optional<lc::PoseWithCovariance> ICP::ComputeRelativeTransform(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,
  const Eigen::Isometry3d& initial_estimate) {
  if (params_.coarse_to_fine) {
    return RunCoarseToFineICP(source_cloud, target_cloud, initial_estimate);
  } else {
    return RunICP(source_cloud, target_cloud, initial_estimate);
  }
}

boost::optional<lc::PoseWithCovariance> ICP::RunICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud,
                                                    const pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,
                                                    const Eigen::Isometry3d& initial_estimate) {
  static lc::Timer icp_timer("ICP");
  icp_timer.Start();
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
  EstimateNormals(target_cloud, params_.search_radius, *target_cloud_with_normals);

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
  if (params_.symmetric_objective) {
    EstimateNormals(source_cloud, params_.search_radius, *source_cloud_with_normals);
  } else {
    pcl::copyPointCloud(*source_cloud, *source_cloud_with_normals);
  }

  RemoveNansAndZerosFromPoints(*source_cloud_with_normals);
  RemoveNansAndZerosFromPoints(*target_cloud_with_normals);

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

boost::optional<lc::PoseWithCovariance> ICP::RunCoarseToFineICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud,
                                                                const pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,
                                                                const Eigen::Isometry3d& initial_estimate) {
  static lc::Timer coarse_to_fine_icp_timer("Coarse To Fine ICP");
  coarse_to_fine_icp_timer.Start();
  boost::optional<lc::PoseWithCovariance> latest_relative_transform =
    lc::PoseWithCovariance(initial_estimate, Eigen::Matrix<double, 6, 6>());
  for (int i = 0; i < params_.num_coarse_to_fine_levels; ++i) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr icp_source_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr icp_target_cloud;
    if (i == params_.num_coarse_to_fine_levels - 1 && !params_.downsample_last_coarse_to_fine_iteration) {
      icp_source_cloud = source_cloud;
      icp_target_cloud = target_cloud;
    } else {
      const double leaf_size_ratio =
        static_cast<double>(params_.num_coarse_to_fine_levels) / static_cast<double>(i + 1.0);
      const double leaf_size = leaf_size_ratio * params_.coarse_to_fine_final_leaf_size;
      // TODO(rsoussan): Why does template deduction fail without this?
      icp_source_cloud = DownsamplePointCloud<pcl::PointXYZI>(source_cloud, leaf_size);
      icp_target_cloud = DownsamplePointCloud<pcl::PointXYZI>(target_cloud, leaf_size);
    }
    latest_relative_transform = RunICP(icp_source_cloud, icp_target_cloud, latest_relative_transform->pose);
    if (!latest_relative_transform) {
      LogWarning("RunCoarseToFineICP: Failed to get relative transform.");
      coarse_to_fine_icp_timer.StopAndLog();
      return boost::none;
    }
  }
  coarse_to_fine_icp_timer.StopAndLog();
  return latest_relative_transform;
}

Eigen::Matrix<double, 1, 6> ICP::Jacobian(const pcl::PointXYZINormal& source_point,
                                          const pcl::PointXYZINormal& target_point,
                                          const Eigen::Isometry3d& relative_transform) const {
  const gtsam::Pose3 gt_relative_transform = lc::GtPose(relative_transform);
  const gtsam::Point3 gt_point(source_point.x, source_point.y, source_point.z);
  const gtsam::Point3 gt_normal(target_point.normal[0], target_point.normal[1], target_point.normal[2]);
  return point_cloud_common::Jacobian(gt_point, gt_normal, gt_relative_transform);
}

void ICP::FilterCorrespondences(const pcl::PointCloud<pcl::PointXYZINormal>& input_cloud,
                                const pcl::PointCloud<pcl::PointXYZINormal>& target_cloud,
                                pcl::Correspondences& correspondences) const {
  for (auto correspondence_it = correspondences.begin(); correspondence_it != correspondences.end();) {
    const auto& input_point = (input_cloud)[correspondence_it->index_query];
    const auto& target_point = (target_cloud)[correspondence_it->index_match];
    const bool invalid_correspondence =
      std::isnan(input_point.x) || std::isnan(input_point.y) || std::isnan(input_point.z) ||
      std::isnan(target_point.x) || std::isnan(target_point.y) || std::isnan(target_point.z) ||
      std::isnan(target_point.normal_x) || std::isnan(target_point.normal_y) || std::isnan(target_point.normal_z);
    if (invalid_correspondence) {
      correspondence_it = correspondences.erase(correspondence_it);
      continue;
    }
    ++correspondence_it;
  }
}

Eigen::Matrix<double, 6, 6> ICP::ComputeCovarianceMatrix(
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
    const Eigen::Matrix<double, 1, 6> jacobian = Jacobian(input_point, target_point, relative_transform);
    if (std::isnan(jacobian(0, 0)) || std::isnan(jacobian(0, 1)) || std::isnan(jacobian(0, 2)) ||
        std::isnan(jacobian(0, 3)) || std::isnan(jacobian(0, 4)) || std::isnan(jacobian(0, 5)))
      continue;
    full_jacobian.block(index++, 0, 1, 6) = jacobian;
  }
  const Eigen::Matrix<double, 6, 6> covariance = (full_jacobian.transpose() * full_jacobian).inverse();
  return covariance;
}
}  // namespace point_cloud_common
