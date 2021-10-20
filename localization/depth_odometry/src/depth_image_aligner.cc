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

#include <depth_odometry/brisk_feature_detector_and_matcher.h>
#include <depth_odometry/depth_image_aligner.h>
#include <depth_odometry/lk_optical_flow_feature_detector_and_matcher.h>
#include <depth_odometry/point_cloud_utilities.h>
#include <depth_odometry/surf_feature_detector_and_matcher.h>
#include <localization_common/logger.h>
#include <localization_common/timer.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;

DepthImageAligner::DepthImageAligner(const DepthImageAlignerParams& params)
    : params_(params),
      point_cloud_aligner_(params_.point_cloud_with_known_correspondences_aligner),
      cam_(*(params_.camera_params)) {
  if (params_.detector == "brisk") {
    feature_detector_and_matcher_.reset(new BriskFeatureDetectorAndMatcher(params_.brisk_feature_detector_and_matcher));
  } else if (params_.detector == "lk_optical_flow") {
    feature_detector_and_matcher_.reset(
      new LKOpticalFlowFeatureDetectorAndMatcher(params_.lk_optical_flow_feature_detector_and_matcher));
  } else if (params_.detector == "surf") {
    feature_detector_and_matcher_.reset(new SurfFeatureDetectorAndMatcher(params_.surf_feature_detector_and_matcher));
  } else {
    LogFatal("DepthImageAligner: Invalid feature detector and matcher.");
  }
  clahe_ = cv::createCLAHE(params_.clahe_clip_limit, cv::Size(params_.clahe_grid_length, params_.clahe_grid_length));
}

bool DepthImageAligner::ValidImagePoint(const Eigen::Vector2d& image_point) const {
  const int cols = latest_feature_depth_image_->cols();
  const int rows = latest_feature_depth_image_->rows();
  const double x_distance_to_border = std::min(image_point.x(), cols - image_point.x());
  const double y_distance_to_border = std::min(image_point.y(), rows - image_point.y());
  return (x_distance_to_border >= params_.min_x_distance_to_border &&
          y_distance_to_border >= params_.min_y_distance_to_border);
}

bool DepthImageAligner::Valid3dPoint(const boost::optional<pcl::PointXYZI>& point) const {
  return point && ValidPoint(*point) && point->z >= 0;
}

void DepthImageAligner::InitializeKdTree(const pcl::PointCloud<pcl::PointXYZI>& point_cloud,
                                         pcl::search::KdTree<pcl::PointXYZI>::Ptr& kdtree,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_point_cloud) const {
  kdtree = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>());
  filtered_point_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>(point_cloud));
  RemoveNansAndZerosFromPoints(*filtered_point_cloud);
  kdtree->setInputCloud(filtered_point_cloud);
}

void DepthImageAligner::InitializeRequiredKdTrees(
  pcl::search::KdTree<pcl::PointXYZI>::Ptr& source_kdtree,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& source_filtered_point_cloud,
  pcl::search::KdTree<pcl::PointXYZI>::Ptr& target_kdtree,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& target_filtered_point_cloud) const {
  if (params_.point_cloud_with_known_correspondences_aligner.use_point_to_plane_cost ||
      params_.point_cloud_with_known_correspondences_aligner.use_symmetric_point_to_plane_cost) {
    InitializeKdTree(*(latest_feature_depth_image_->point_cloud), target_kdtree, target_filtered_point_cloud);
  }

  // TODO(rsoussan): make kdtree on depth image measurement creation to avoid recreating if using symmetric cost
  if (params_.point_cloud_with_known_correspondences_aligner.use_symmetric_point_to_plane_cost) {
    InitializeKdTree(*(previous_feature_depth_image_->point_cloud), source_kdtree, source_filtered_point_cloud);
  }
}

bool DepthImageAligner::GetRequiredNormals(const Eigen::Vector3d& source_landmark,
                                           const pcl::search::KdTree<pcl::PointXYZI>& source_kdtree,
                                           const pcl::PointCloud<pcl::PointXYZI>& source_filtered_point_cloud,
                                           const Eigen::Vector3d& target_landmark,
                                           const pcl::search::KdTree<pcl::PointXYZI>& target_kdtree,
                                           const pcl::PointCloud<pcl::PointXYZI>& target_filtered_point_cloud,
                                           std::vector<Eigen::Vector3d>& source_normals,
                                           std::vector<Eigen::Vector3d>& target_normals) const {
  if (params_.point_cloud_with_known_correspondences_aligner.use_point_to_plane_cost ||
      params_.point_cloud_with_known_correspondences_aligner.use_symmetric_point_to_plane_cost) {
    const auto target_normal = GetNormal(target_landmark, target_filtered_point_cloud, target_kdtree,
                                         params_.point_cloud_with_known_correspondences_aligner.normal_search_radius);
    if (!target_normal) return false;
    if (params_.point_cloud_with_known_correspondences_aligner.use_symmetric_point_to_plane_cost) {
      const auto source_normal = GetNormal(source_landmark, source_filtered_point_cloud, source_kdtree,
                                           params_.point_cloud_with_known_correspondences_aligner.normal_search_radius);
      if (!source_normal) return false;
      source_normals.emplace_back(*source_normal);
    }
    target_normals.emplace_back(*target_normal);
  }
  return true;
}

boost::optional<lc::PoseWithCovariance> DepthImageAligner::ComputeRelativeTransform() {
  if (!previous_feature_depth_image_ || !latest_feature_depth_image_) return boost::none;
  const auto& matches =
    feature_detector_and_matcher_->Match(*previous_feature_depth_image_, *latest_feature_depth_image_);

  std::vector<Eigen::Vector3d> source_landmarks;
  std::vector<Eigen::Vector3d> target_landmarks;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr source_kdtree;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_filtered_point_cloud;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr target_kdtree;
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_filtered_point_cloud;
  InitializeRequiredKdTrees(source_kdtree, source_filtered_point_cloud, target_kdtree, target_filtered_point_cloud);

  // Store valid matches
  std::vector<Eigen::Vector2d> source_image_points;
  std::vector<Eigen::Vector2d> target_image_points;
  std::vector<Eigen::Vector3d> source_normals;
  std::vector<Eigen::Vector3d> target_normals;
  for (int i = 0; i < static_cast<int>(matches.size()); ++i) {
    const auto& match = matches[i];
    const auto& source_image_point = match.source_point;
    const auto& target_image_point = match.target_point;
    if (!ValidImagePoint(source_image_point) || !ValidImagePoint(target_image_point)) continue;
    const auto source_point_3d =
      previous_feature_depth_image_->InterpolatePoint3D(source_image_point.x(), source_image_point.y());
    const auto target_point_3d =
      latest_feature_depth_image_->InterpolatePoint3D(target_image_point.x(), target_image_point.y());
    if (!Valid3dPoint(source_point_3d) || !Valid3dPoint(target_point_3d)) continue;
    const Eigen::Vector3d source_landmark(source_point_3d->x, source_point_3d->y, source_point_3d->z);
    const Eigen::Vector3d target_landmark(target_point_3d->x, target_point_3d->y, target_point_3d->z);
    if (!GetRequiredNormals(source_landmark, *source_kdtree, *source_filtered_point_cloud, target_landmark,
                            *target_kdtree, *target_filtered_point_cloud, source_normals, target_normals))
      continue;

    source_image_points.emplace_back(source_image_point);
    target_image_points.emplace_back(target_image_point);
    source_landmarks.emplace_back(source_landmark);
    target_landmarks.emplace_back(target_landmark);
  }

  if (target_landmarks.size() < 4) {
    LogError("ComputeRelativeTransform: Too few points provided, need 4 but given " << target_landmarks.size() << ".");
    return boost::none;
  }

  matches_ = DepthMatches(source_image_points, target_image_points, source_landmarks, target_landmarks,
                          previous_feature_depth_image_->timestamp, latest_feature_depth_image_->timestamp);

  point_cloud_aligner_.SetTargetNormals(std::move(target_normals));
  point_cloud_aligner_.SetSourceNormals(std::move(source_normals));
  if (target_landmarks.size() < 4) {
    LogError("ComputeRelativeTransform: Not enough points with valid normals, need 4 but given "
             << target_landmarks.size() << ".");
    return boost::none;
  }

  const auto relative_transform = point_cloud_aligner_.ComputeRelativeTransform(source_landmarks, target_landmarks);
  // TODO(rsoussan): make this a param?? is this already a param in depth odometry? (B)
  if (relative_transform.pose.translation().norm() > 10) {
    LogError("large norm!!!");
    return boost::none;
  }

  return lc::PoseWithCovariance(relative_transform.pose.inverse(), relative_transform.covariance);
}

void DepthImageAligner::AddLatestDepthImage(const lm::DepthImageMeasurement& latest_depth_image) {
  previous_feature_depth_image_ = std::move(latest_feature_depth_image_);
  if (params_.use_clahe) {
    lm::DepthImageMeasurement clahe_depth_image = latest_depth_image;
    clahe_->apply(latest_depth_image.image, clahe_depth_image.image);
    latest_feature_depth_image_.reset(
      new lm::FeatureDepthImageMeasurement(clahe_depth_image, feature_detector_and_matcher_->detector()));
  } else
    latest_feature_depth_image_.reset(
      new lm::FeatureDepthImageMeasurement(latest_depth_image, feature_detector_and_matcher_->detector()));
}
}  // namespace depth_odometry
