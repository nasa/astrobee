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

#include <depth_odometry/depth_odometry.h>
#include <depth_odometry/transformation_estimation_symmetric_point_to_plane_lls.h>
#include <depth_odometry/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/timer.h>
#include <localization_common/utilities.h>

#include <gtsam/geometry/Pose3.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
// TODO(rsoussan): Switch back to this when PCL bug is fixed
//#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <depth_odometry/correspondence_rejection_surface_normal2.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace depth_odometry {
namespace lc = localization_common;

DepthOdometry::DepthOdometry() {
  // TODO(rsoussan): remove this
  config_reader::ConfigReader config;
  config.AddFile("transforms.config");
  config.AddFile("geometry.config");
  config.AddFile("localization/depth_odometry.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }

  LoadDepthOdometryParams(config, params_);
}

std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> DepthOdometry::previous_depth_cloud() const {
  return previous_depth_cloud_;
}
std::pair<localization_common::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> DepthOdometry::latest_depth_cloud() const {
  return latest_depth_cloud_;
}

Eigen::Isometry3d DepthOdometry::latest_relative_transform() const { return latest_relative_transform_; }

const pcl::Correspondences& DepthOdometry::correspondences() const { return correspondences_; }

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> DepthOdometry::DepthCloudCallback(
  std::pair<lc::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> depth_cloud) {
  RemoveNansAndZerosFromPointXYZs(*(depth_cloud.second));
  if (!previous_depth_cloud_.second && !latest_depth_cloud_.second) latest_depth_cloud_ = depth_cloud;
  if (depth_cloud.first < latest_depth_cloud_.first) {
    LogWarning("DepthCloudCallback: Out of order measurement received.");
    return boost::none;
  }
  LogError("t: " << std::setprecision(15) << depth_cloud.first);
  previous_depth_cloud_ = latest_depth_cloud_;
  latest_depth_cloud_ = depth_cloud;
  auto relative_transform = Icp(previous_depth_cloud_.second, latest_depth_cloud_.second);
  if (!relative_transform) {
    LogWarning("DepthCloudCallback: Failed to get relative transform.");
    return boost::none;
  }

  if (!CovarianceSane(relative_transform->second)) {
    LogWarning("DepthCloudCallback: Sanity check failed - invalid covariance.");
    return boost::none;
  }

  if (params_.frame_change_transform) {
    relative_transform->first = params_.body_T_haz_cam * relative_transform->first * params_.body_T_haz_cam.inverse();
    // TODO: rotate covariance matrix!!!! use exp map jacobian!!! sandwich withthis! (translation should be rotated by
    // rotation matrix)
  }

  // LogError("cov: " << std::endl << relative_transform->second.matrix());
  if (relative_transform->first.translation().norm() > 0.5) LogError("large position jump!!");
  latest_relative_transform_ = relative_transform->first;
  return relative_transform;
}

bool DepthOdometry::CovarianceSane(const Eigen::Matrix<double, 6, 6>& covariance) const {
  const auto position_covariance_norm = covariance.block<3, 3>(0, 0).diagonal().norm();
  const auto orientation_covariance_norm = covariance.block<3, 3>(3, 3).diagonal().norm();
  LogError("pcov: " << position_covariance_norm << ", ocov: " << orientation_covariance_norm);
  return (position_covariance_norm <= params_.position_covariance_threshold &&
          orientation_covariance_norm <= params_.orientation_covariance_threshold);
}

void DepthOdometry::EstimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    pcl::PointCloud<pcl::PointNormal>& cloud_with_normals) const {
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(params_.search_radius);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*cloud_normals);
  pcl::concatenateFields(*cloud, *cloud_normals, cloud_with_normals);
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr DepthOdometry::EstimateHistogramFeatures(
  const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals) const {
  pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> feature_estimator;
  feature_estimator.setInputCloud(cloud_with_normals);
  feature_estimator.setInputNormals(cloud_with_normals);
  // TODO(rsoussan): Pass in kd tree from normal estimation?
  pcl::search::KdTree<pcl::PointNormal>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointNormal>);
  feature_estimator.setSearchMethod(kd_tree);
  // pcl: IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  feature_estimator.setRadiusSearch(0.05);  // 0.2??
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>());
  feature_estimator.compute(*features);
  return features;
}

Eigen::Matrix4f DepthOdometry::RansacIA(const pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                                        const pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud) const {
  const auto source_features = EstimateHistogramFeatures(source_cloud);
  const auto target_features = EstimateHistogramFeatures(target_cloud);

  pcl::SampleConsensusInitialAlignment<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> sac_ia_aligner;
  sac_ia_aligner.setInputSource(source_cloud);
  sac_ia_aligner.setInputTarget(target_cloud);
  sac_ia_aligner.setSourceFeatures(source_features);
  sac_ia_aligner.setTargetFeatures(target_features);
  sac_ia_aligner.setMaximumIterations(10);
  sac_ia_aligner.setMinSampleDistance(0);
  sac_ia_aligner.setNumberOfSamples(100);
  sac_ia_aligner.setCorrespondenceRandomness(1);
  pcl::PointCloud<pcl::PointNormal>::Ptr result(new pcl::PointCloud<pcl::PointNormal>);
  sac_ia_aligner.align(*result);
  // std::cout  <<"sac has converged:"<<scia.hasConverged()<<"  score: "<<scia.getFitnessScore()<<endl;
  // TODO(rsoussan): make boost optional, set thresholds for ransacia fitness and make sure it converged!
  // TODO(rsoussan): invert this???
  // const Matrix<double, 4, 4> final_transform = sac_ia_aligner.getFinalTransform();
  return sac_ia_aligner.getFinalTransformation();
}

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> DepthOdometry::Icp(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud) {
  static lc::Timer icp_timer("ICP");
  icp_timer.Start();
  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  EstimateNormals(target_cloud, *target_cloud_with_normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  if (params_.symmetric_objective) {
    EstimateNormals(source_cloud, *source_cloud_with_normals);
  } else {
    pcl::copyPointCloud(*source_cloud, *source_cloud_with_normals);
  }

  RemoveNansAndZerosFromPointNormals(*source_cloud_with_normals);
  RemoveNansAndZerosFromPointNormals(*target_cloud_with_normals);

  pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
  Eigen::Matrix4f initial_estimate = Eigen::Matrix4f::Identity();
  if (params_.inital_estimate_with_ransac_ia) {
    initial_estimate = RansacIA(source_cloud_with_normals, target_cloud_with_normals);
  }

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
  icp.align(*result, initial_estimate);

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

Eigen::Matrix<double, 1, 6> DepthOdometry::Jacobian(const pcl::PointNormal& source_point,
                                                    const pcl::PointNormal& target_point,
                                                    const Eigen::Isometry3d& relative_transform) const {
  const gtsam::Pose3 gt_relative_transform = lc::GtPose(relative_transform);
  const gtsam::Point3 gt_point(source_point.x, source_point.y, source_point.z);
  const gtsam::Point3 gt_normal(target_point.normal[0], target_point.normal[1], target_point.normal[2]);
  return depth_odometry::Jacobian(gt_point, gt_normal, gt_relative_transform);
}

void DepthOdometry::FilterCorrespondences(const pcl::PointCloud<pcl::PointNormal>& input_cloud,
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

Eigen::Matrix<double, 6, 6> DepthOdometry::ComputeCovarianceMatrix(
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
