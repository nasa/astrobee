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
#include <localization_common/utilities.h>
#include <point_cloud_common/utilities.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/ia_ransac.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/impl/kdtree.hpp>

namespace point_cloud_common {
namespace lc = localization_common;

void EstimateNormals(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const double search_radius,
                     pcl::PointCloud<pcl::PointXYZINormal>& cloud_with_normals) {
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(search_radius);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*cloud_normals);
  pcl::concatenateFields(*cloud, *cloud_normals, cloud_with_normals);
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr EstimateHistogramFeatures(
  const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals) {
  pcl::FPFHEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::FPFHSignature33> feature_estimator;
  feature_estimator.setInputCloud(cloud_with_normals);
  feature_estimator.setInputNormals(cloud_with_normals);
  // TODO(rsoussan): Pass in kd tree from normal estimation?
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZINormal>);
  feature_estimator.setSearchMethod(kd_tree);
  // pcl: IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  feature_estimator.setRadiusSearch(0.05);  // 0.2??
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>());
  feature_estimator.compute(*features);
  return features;
}

Eigen::Matrix4f RansacIA(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_cloud,
                         const pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud) {
  const auto source_features = EstimateHistogramFeatures(source_cloud);
  const auto target_features = EstimateHistogramFeatures(target_cloud);

  pcl::SampleConsensusInitialAlignment<pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::FPFHSignature33> sac_ia_aligner;
  sac_ia_aligner.setInputSource(source_cloud);
  sac_ia_aligner.setInputTarget(target_cloud);
  sac_ia_aligner.setSourceFeatures(source_features);
  sac_ia_aligner.setTargetFeatures(target_features);
  sac_ia_aligner.setMaximumIterations(10);
  sac_ia_aligner.setMinSampleDistance(0);
  sac_ia_aligner.setNumberOfSamples(100);
  sac_ia_aligner.setCorrespondenceRandomness(1);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZINormal>);
  sac_ia_aligner.align(*result);
  // std::cout  <<"sac has converged:"<<scia.hasConverged()<<"  score: "<<scia.getFitnessScore()<<endl;
  // TODO(rsoussan): make boost optional, set thresholds for ransacia fitness and make sure it converged!
  // TODO(rsoussan): invert this???
  // const Matrix<double, 4, 4> final_transform = sac_ia_aligner.getFinalTransform();
  return sac_ia_aligner.getFinalTransformation();
}

Eigen::Matrix<double, 1, 6> Jacobian(const gtsam::Point3& point, const gtsam::Vector3& normal,
                                     const gtsam::Pose3& relative_transform) {
  gtsam::Matrix H1;
  relative_transform.transformFrom(point, H1);
  return normal.transpose() * H1;
}

Eigen::Matrix<double, 1, 6> PointToPointJacobian(const gtsam::Point3& source_point,
                                                 const gtsam::Pose3& relative_transform) {
  gtsam::Matrix H1;
  relative_transform.transformFrom(source_point, H1);
  return H1;
}

Eigen::Isometry3d ComputeRelativeTransformUmeyama(const std::vector<Eigen::Vector3d>& source_points,
                                                  const std::vector<Eigen::Vector3d>& target_points) {
  const int num_points = static_cast<int>(source_points.size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> source_cloud_matrix(3, num_points);
  Eigen::Matrix<double, 3, Eigen::Dynamic> target_cloud_matrix(3, num_points);
  for (int i = 0; i < num_points; ++i) {
    const auto& source_point = source_points[i];
    source_cloud_matrix(0, i) = source_point.x();
    source_cloud_matrix(1, i) = source_point.y();
    source_cloud_matrix(2, i) = source_point.z();

    const auto& target_point = target_points[i];
    target_cloud_matrix(0, i) = target_point.x();
    target_cloud_matrix(1, i) = target_point.y();
    target_cloud_matrix(2, i) = target_point.z();
  }

  const Eigen::Matrix<double, 4, 4> relative_transform =
    Eigen::umeyama(source_cloud_matrix, target_cloud_matrix, false);
  return Eigen::Isometry3d(relative_transform.matrix());
}

boost::optional<Eigen::Vector3d> GetNormal(const Eigen::Vector3d& point, const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                           const pcl::search::KdTree<pcl::PointXYZI>& kdtree,
                                           const double search_radius) {
  // Adapted from pcl code
  pcl::PointXYZI pcl_point;
  pcl_point.x = point.x();
  pcl_point.y = point.y();
  pcl_point.z = point.z();

  std::vector<int> nn_indices;
  std::vector<float> distances;
  if (kdtree.radiusSearch(pcl_point, search_radius, nn_indices, distances, 0) < 3) {
    LogError("GetNormal: Failed to get enough neighboring points for query point.");
    return boost::none;
  }

  float normal_x;
  float normal_y;
  float normal_z;
  float curvature;
  if (!computePointNormal(cloud, nn_indices, normal_x, normal_y, normal_z, curvature)) {
    LogError("GetNormal: Failed to compute point normal.");
    return boost::none;
  }

  const double vpx = cloud.sensor_origin_.coeff(0);
  const double vpy = cloud.sensor_origin_.coeff(1);
  const double vpz = cloud.sensor_origin_.coeff(2);
  flipNormalTowardsViewpoint(pcl_point, vpx, vpy, vpz, normal_x, normal_y, normal_z);
  return Eigen::Vector3d(normal_x, normal_y, normal_z);
}

bool computePointNormal(const pcl::PointCloud<pcl::PointXYZI>& cloud, const std::vector<int>& indices, float& normal_x,
                        float& normal_y, float& normal_z, float& curvature) {
  // Adapted from pcl::common::centroid.h
  /** \brief Placeholder for the 3x3 covariance matrix at each surface patch. */
  static EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix_;
  /** \brief 16-bytes aligned placeholder for the XYZ centroid of a surface patch. */
  static Eigen::Vector4f xyz_centroid_;
  if (indices.size() < 3 ||
      pcl::computeMeanAndCovarianceMatrix(cloud, indices, covariance_matrix_, xyz_centroid_) == 0) {
    return false;
  }

  // Get the plane normal and surface curvature
  pcl::solvePlaneParameters(covariance_matrix_, normal_x, normal_y, normal_z, curvature);
  return true;
}

void flipNormalTowardsViewpoint(const pcl::PointXYZI& point, float vp_x, float vp_y, float vp_z, float& nx, float& ny,
                                float& nz) {
  // Adapted from pcl code
  // See if we need to flip any plane normals
  vp_x -= point.x;
  vp_y -= point.y;
  vp_z -= point.z;

  // Dot product between the (viewpoint - point) and the plane normal
  float cos_theta = (vp_x * nx + vp_y * ny + vp_z * nz);

  // Flip the plane normal
  if (cos_theta < 0) {
    nx *= -1;
    ny *= -1;
    nz *= -1;
  }
}

bool ValidVector6d(const Eigen::Matrix<double, 1, 6>& vector) {
  if (std::isnan(vector(0, 0)) || std::isnan(vector(0, 1)) || std::isnan(vector(0, 2)) || std::isnan(vector(0, 3)) ||
      std::isnan(vector(0, 4)) || std::isnan(vector(0, 5)))
    return false;
  return true;
}

Eigen::Matrix<double, 6, 6> ComputePointToPointCovarianceMatrix(const std::vector<Eigen::Vector3d>& source_points,
                                                                const Eigen::Isometry3d& relative_transform) {
  const int num_correspondences = static_cast<int>(source_points.size());
  Eigen::MatrixXd full_jacobian(num_correspondences, 6);
  for (int i = 0; i < num_correspondences; ++i) {
    const Eigen::Matrix<double, 1, 6> jacobian = PointToPointJacobian(source_points[i], lc::GtPose(relative_transform));
    if (!ValidVector6d(jacobian)) continue;
    full_jacobian.block(i++, 0, 1, 6) = jacobian;
  }
  const Eigen::Matrix<double, 6, 6> covariance = (full_jacobian.transpose() * full_jacobian).inverse();
  return covariance;
}

pcl::PointXYZI Interpolate(const double alpha, const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b) {
  const double beta = 1.0 - alpha;
  pcl::PointXYZI interpolated_point;
  interpolated_point.x = beta * point_a.x + alpha * point_b.x;
  interpolated_point.y = beta * point_a.y + alpha * point_b.y;
  interpolated_point.z = beta * point_a.z + alpha * point_b.z;
  interpolated_point.intensity = beta * point_a.intensity + alpha * point_b.intensity;
  return interpolated_point;
}

template <>
bool ValidPoint<pcl::PointXYZ>(const pcl::PointXYZ& point) {
  return ValidPointXYZ(point);
}

template <>
bool ValidPoint<pcl::PointXYZI>(const pcl::PointXYZI& point) {
  return ValidPointXYZ(point) && ValidIntensity(point);
}

template <>
bool ValidPoint<pcl::PointNormal>(const pcl::PointNormal& point) {
  return ValidPointXYZ(point) && ValidNormal(point);
}

template <>
bool ValidPoint<pcl::PointXYZINormal>(const pcl::PointXYZINormal& point) {
  return ValidPointXYZ(point) && ValidNormal(point) && ValidIntensity(point);
}
}  // namespace point_cloud_common
