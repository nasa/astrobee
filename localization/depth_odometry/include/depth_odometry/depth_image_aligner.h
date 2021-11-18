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
#ifndef DEPTH_ODOMETRY_DEPTH_IMAGE_ALIGNER_H_
#define DEPTH_ODOMETRY_DEPTH_IMAGE_ALIGNER_H_

#include <camera/camera_model.h>
#include <depth_odometry/depth_image_aligner_params.h>
#include <depth_odometry/depth_matches.h>
#include <depth_odometry/feature_detector_and_matcher.h>
#include <localization_common/pose_with_covariance.h>
#include <localization_measurements/depth_image_measurement.h>
#include <localization_measurements/feature_depth_image_measurement.h>
#include <point_cloud_common/point_cloud_with_known_correspondences_aligner.h>

#include <boost/optional.hpp>

#include <opencv2/imgproc.hpp>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/kdtree.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <vector>

namespace depth_odometry {
class DepthImageAligner {
 public:
  explicit DepthImageAligner(const DepthImageAlignerParams& params);
  boost::optional<localization_common::PoseWithCovariance> ComputeRelativeTransform();
  void AddLatestDepthImage(const localization_measurements::DepthImageMeasurement& latest_depth_image);
  const boost::optional<DepthMatches>& matches() const { return matches_; }

 private:
  bool ValidImagePoint(const Eigen::Vector2d& image_point) const;
  bool Valid3dPoint(const boost::optional<pcl::PointXYZI>& point) const;
  void InitializeKdTree(const pcl::PointCloud<pcl::PointXYZI>& point_cloud,
                        pcl::search::KdTree<pcl::PointXYZI>::Ptr& kdtree,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_point_cloud) const;
  void InitializeRequiredKdTrees(pcl::search::KdTree<pcl::PointXYZI>::Ptr& source_kdtree,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr& source_filtered_point_cloud,
                                 pcl::search::KdTree<pcl::PointXYZI>::Ptr& target_kdtree,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr& target_filtered_point_cloud) const;
  bool GetRequiredNormals(const Eigen::Vector3d& source_landmark,
                          const pcl::search::KdTree<pcl::PointXYZI>& source_kdtree,
                          const pcl::PointCloud<pcl::PointXYZI>& source_filtered_point_cloud,
                          const Eigen::Vector3d& target_landmark,
                          const pcl::search::KdTree<pcl::PointXYZI>& target_kdtree,
                          const pcl::PointCloud<pcl::PointXYZI>& target_filtered_point_cloud,
                          std::vector<Eigen::Vector3d>& source_normals,
                          std::vector<Eigen::Vector3d>& target_normals) const;
  Eigen::Isometry3d ComputeRelativeTransform(const std::vector<Eigen::Vector3d>& source_landmarks,
                                             const std::vector<Eigen::Vector3d>& target_landmarks) const;

  DepthImageAlignerParams params_;
  point_cloud_common::PointCloudWithKnownCorrespondencesAligner point_cloud_aligner_;
  std::unique_ptr<localization_measurements::FeatureDepthImageMeasurement> previous_feature_depth_image_;
  std::unique_ptr<localization_measurements::FeatureDepthImageMeasurement> latest_feature_depth_image_;
  std::unique_ptr<FeatureDetectorAndMatcher> feature_detector_and_matcher_;
  boost::optional<DepthMatches> matches_;
  cv::Ptr<cv::CLAHE> clahe_;
  camera::CameraModel cam_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_IMAGE_ALIGNER_H_
