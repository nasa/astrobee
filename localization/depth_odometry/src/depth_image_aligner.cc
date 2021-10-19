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

#include <camera/camera_params.h>
#include <camera/camera_model.h>
#include <depth_odometry/brisk_feature_detector_and_matcher.h>
#include <depth_odometry/depth_image_aligner.h>
#include <depth_odometry/lk_optical_flow_feature_detector_and_matcher.h>
#include <depth_odometry/point_cloud_with_known_correspondences_aligner.h>
#include <depth_odometry/point_cloud_utilities.h>
#include <depth_odometry/surf_feature_detector_and_matcher.h>
#include <localization_common/logger.h>
#include <localization_common/timer.h>
#include <sparse_mapping/reprojection.h>

#include <opencv2/imgproc.hpp>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/kdtree.hpp>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;

DepthImageAligner::DepthImageAligner(const DepthImageAlignerParams& params)
    : params_(params), cam_(*(params_.camera_params)) {
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
  // TODO(rsoussan): Get these from somewhere else
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

boost::optional<lc::PoseWithCovariance> DepthImageAligner::ComputeRelativeTransform() {
  if (!previous_feature_depth_image_ || !latest_feature_depth_image_) return boost::none;
  const auto& matches =
    feature_detector_and_matcher_->Match(*previous_feature_depth_image_, *latest_feature_depth_image_);

  std::vector<Eigen::Vector3d> source_landmarks;
  std::vector<Eigen::Vector3d> target_landmarks;
  // TODO: make this optional?
  std::vector<Eigen::Vector3d> target_normals;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr target_kdtree;
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_filtered_point_cloud;
  if (params_.point_cloud_with_known_correspondences_aligner.use_point_to_plane_cost ||
      params_.point_cloud_with_known_correspondences_aligner.use_symmetric_point_to_plane_cost) {
    target_kdtree = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>());
    target_filtered_point_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
      new pcl::PointCloud<pcl::PointXYZI>(*(latest_feature_depth_image_->point_cloud)));
    RemoveNansAndZerosFromPoints(*target_filtered_point_cloud);
    target_kdtree->setInputCloud(target_filtered_point_cloud);
  }

  // TODO(rsoussan): make kdtree on depth image measurement creation to avoid recreating if using symmetric cost
  std::vector<Eigen::Vector3d> source_normals;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr source_kdtree;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_filtered_point_cloud;
  if (params_.point_cloud_with_known_correspondences_aligner.use_symmetric_point_to_plane_cost) {
    source_kdtree = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>());
    source_filtered_point_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
      new pcl::PointCloud<pcl::PointXYZI>(*(previous_feature_depth_image_->point_cloud)));
    RemoveNansAndZerosFromPoints(*source_filtered_point_cloud);
    source_kdtree->setInputCloud(source_filtered_point_cloud);
  }

  // Get 3D points for image features
  std::vector<Eigen::Vector2d> source_image_points;
  std::vector<Eigen::Vector2d> target_image_points;
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
    if (params_.point_cloud_with_known_correspondences_aligner.use_point_to_plane_cost ||
        params_.point_cloud_with_known_correspondences_aligner.use_symmetric_point_to_plane_cost) {
      const auto target_normal = GetNormal(target_landmark, *target_filtered_point_cloud, *target_kdtree);
      if (!target_normal) continue;
      if (params_.point_cloud_with_known_correspondences_aligner.use_symmetric_point_to_plane_cost) {
        const auto source_normal = GetNormal(source_landmark, *source_filtered_point_cloud, *source_kdtree);
        if (!source_normal) continue;
        source_normals.emplace_back(*source_normal);
      }
      target_normals.emplace_back(*target_normal);
    }

    source_image_points.emplace_back(source_image_point);
    target_image_points.emplace_back(target_image_point);
    source_landmarks.emplace_back(source_landmark);
    target_landmarks.emplace_back(target_landmark);
  }

  if (target_landmarks.size() < 4) {
    LogError("ComputeRelativeTransform: Too few points provided, need 4 but given " << target_landmarks.size() << ".");
    return boost::none;
  }

  // TODO: save image points!
  matches_ = DepthMatches(source_image_points, target_image_points, source_landmarks, target_landmarks,
                          previous_feature_depth_image_->timestamp, latest_feature_depth_image_->timestamp);

  // TODO: make this a member var!
  PointCloudWithKnownCorrespondencesAligner point_cloud_aligner(params_.point_cloud_with_known_correspondences_aligner);
  // TODO(rsoussan): make these conditional on using point to plane/symmetric costs!!!
  point_cloud_aligner.SetTargetNormals(std::move(target_normals));
  point_cloud_aligner.SetSourceNormals(std::move(source_normals));
  if (target_landmarks.size() < 4) {
    LogError("ComputeRelativeTransform: Not enough points with valid normals, need 4 but given "
             << target_landmarks.size() << ".");
    return boost::none;
  }

  // static lc::Timer pc_timer("pc_aligner");
  // pc_timer.Start();
  const auto relative_transform = point_cloud_aligner.ComputeRelativeTransform(source_landmarks, target_landmarks);
  // pc_timer.StopAndLog();
  // TODO: get cov!!
  /*LogError("rel trafo trans: " << relative_transform.pose.translation().matrix());
  LogError("rel trafo trans norm: " << relative_transform.pose.translation().norm());*/
  // TODO(rsoussan): make this a param??
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
