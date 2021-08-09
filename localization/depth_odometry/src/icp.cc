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

#include <depth_odometry/icp.h>
#include <depth_odometry/point_cloud_utilities.h>
#include <depth_odometry/transformation_estimation_symmetric_point_to_plane_lls.h>
#include <localization_common/logger.h>
#include <localization_common/timer.h>
#include <localization_common/utilities.h>

#include <gtsam/geometry/Pose3.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
// TODO(rsoussan): Switch back to this when PCL bug is fixed
//#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <depth_odometry/correspondence_rejection_surface_normal2.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace depth_odometry {
namespace lc = localization_common;

ICP::ICP(const ICPParams& params) : params_(params) {}

const pcl::Correspondences& ICP::correspondences() const { return correspondences_; }

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> ICP::ComputeRelativeTransform(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
  const Eigen::Isometry3d& initial_estimate) {
  if (params_.coarse_to_fine) {
    return RunCoarseToFineICP(source_cloud, target_cloud, initial_estimate);
  } else {
    return RunICP(source_cloud, target_cloud, initial_estimate);
  }
}

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> ICP::RunICP(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
  const Eigen::Isometry3d& initial_estimate) {
  static lc::Timer icp_timer("ICP");
  icp_timer.Start();
  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  EstimateNormals(target_cloud, params_.search_radius, *target_cloud_with_normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  if (params_.symmetric_objective) {
    EstimateNormals(source_cloud, params_.search_radius, *source_cloud_with_normals);
  } else {
    pcl::copyPointCloud(*source_cloud, *source_cloud_with_normals);
  }

  RemoveNansAndZerosFromPointNormals(*source_cloud_with_normals);
  RemoveNansAndZerosFromPointNormals(*target_cloud_with_normals);

  pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
  if (params_.symmetric_objective) {
    auto symmetric_transformation_estimation = boost::make_shared<
      pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>>();
    symmetric_transformation_estimation->setEnforceSameDirectionNormals(params_.enforce_same_direction_normals);
    icp.transformation_estimation_ = symmetric_transformation_estimation;
  }

  if (params_.correspondence_rejector_surface_normal) {
    pcl::registration::CorrespondenceRejectorSurfaceNormal2::Ptr correspondence_rejector_surface_normal(
      new pcl::registration::CorrespondenceRejectorSurfaceNormal2());
    correspondence_rejector_surface_normal->initializeDataContainer<pcl::PointXYZ, pcl::PointNormal>();
    correspondence_rejector_surface_normal->setThreshold(params_.correspondence_rejector_surface_normal_threshold);
    correspondence_rejector_surface_normal->setInputNormals<pcl::PointXYZ, pcl::PointNormal>(source_cloud_with_normals);
    correspondence_rejector_surface_normal->setTargetNormals<pcl::PointXYZ, pcl::PointNormal>(
      target_cloud_with_normals);
    icp.addCorrespondenceRejector(correspondence_rejector_surface_normal);
  }

  icp.setInputSource(source_cloud_with_normals);
  icp.setInputTarget(target_cloud_with_normals);
  icp.setMaximumIterations(params_.max_iterations);
  pcl::PointCloud<pcl::PointNormal>::Ptr result(new pcl::PointCloud<pcl::PointNormal>);
  icp.align(*result, initial_estimate.matrix().cast<float>());

  if (!icp.hasConverged()) {
    LogError("Icp: Failed to converge.");
    icp_timer.Stop();
    return boost::none;
  }

  const double fitness_score = icp.getFitnessScore();
  LogError("fitness: " << fitness_score);
  if (fitness_score > params_.fitness_threshold) {
    LogError("Icp: Fitness score too large: " << fitness_score << ".");
    icp_timer.Stop();
    return boost::none;
  }
  LogError("its: " << icp.nr_iterations_);

  // TODO(rsoussan): clean this up
  // TODO(rsoussan): don't take inverse??
  const Eigen::Isometry3d relative_transform(
    (Eigen::Isometry3f(icp.getFinalTransformation().matrix()).cast<double>()).inverse());
  const Eigen::Matrix<double, 6, 6> covariance =
    ComputeCovarianceMatrix(icp, source_cloud_with_normals, result, relative_transform);
  icp_timer.StopAndLog();
  return std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>{relative_transform, covariance};
}

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> ICP::RunCoarseToFineICP(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
  const Eigen::Isometry3d& initial_estimate) {
  // TODO: add functions to downsample cloud to point cloud utilities!
  // downsample according to ratios!
  // feed new initial estimate to next icp iteration
}

Eigen::Matrix<double, 1, 6> ICP::Jacobian(const pcl::PointNormal& source_point, const pcl::PointNormal& target_point,
                                          const Eigen::Isometry3d& relative_transform) const {
  const gtsam::Pose3 gt_relative_transform = lc::GtPose(relative_transform);
  const gtsam::Point3 gt_point(source_point.x, source_point.y, source_point.z);
  const gtsam::Point3 gt_normal(target_point.normal[0], target_point.normal[1], target_point.normal[2]);
  return depth_odometry::Jacobian(gt_point, gt_normal, gt_relative_transform);
}

void ICP::FilterCorrespondences(const pcl::PointCloud<pcl::PointNormal>& input_cloud,
                                const pcl::PointCloud<pcl::PointNormal>& target_cloud,
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
  const pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>& icp,
  const pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
  const pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud_transformed, const Eigen::Isometry3d& relative_transform) {
  icp.correspondence_estimation_->setInputSource(source_cloud_transformed);
  correspondences_.clear();
  // Assumes normals for input source aren't needed and there are no correspondence rejectors added to ICP object
  icp.correspondence_estimation_->determineCorrespondences(correspondences_, icp.corr_dist_threshold_);
  const auto& target_cloud = icp.target_;
  FilterCorrespondences(*source_cloud, *target_cloud, correspondences_);
  const int num_correspondences = correspondences_.size();
  LogError("a size: " << source_cloud->size());
  LogError("b size: " << target_cloud->size());
  LogError("num correspondences: " << num_correspondences);
  Eigen::MatrixXd full_jacobian(num_correspondences, 6);
  int index = 0;
  for (const auto correspondence : correspondences_) {
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
}  // namespace depth_odometry
