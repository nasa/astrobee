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

#include <localization_measurements/depth_image.h>
#include <point_cloud_common/utilities.h>
#include <vision_common/feature_image.h>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/kdtree.hpp>

#ifndef DEPTH_ODOMETRY_DEPTH_IMAGE_FEATURES_AND_POINTS_H_
#define DEPTH_ODOMETRY_DEPTH_IMAGE_FEATURES_AND_POINTS_H_

namespace depth_odometry {
class DepthImageFeaturesAndPoints {
 public:
  // TODO(rsoussan): Pass clahe as boost optional ref! add default as boost none!
  DepthImageFeaturesAndPoints(const localization_measurements::DepthImage& depth_image, cv::Feature2D& feature_detector,
                              const cv::Ptr<cv::CLAHE> clahe, const bool normals_required = false)
      : depth_image_(depth_image) {
    if (clahe) {
      cv::Mat clahe_image;
      clahe->apply(depth_image_.image(), clahe_image);
      feature_image_.reset(new vision_common::FeatureImage(clahe_image, feature_detector));
    } else {
      feature_image_.reset(new vision_common::FeatureImage(depth_image_.image(), feature_detector));
    }

    if (normals_required) {
      kdtree_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
      filtered_point_cloud_ =
        point_cloud_common::FilteredPointCloud<pcl::PointXYZI>(depth_image_.unfiltered_point_cloud());
      kdtree_->setInputCloud(filtered_point_cloud_);
    }
  }

  boost::optional<Eigen::Vector3d> Normal(const Eigen::Vector3d& point_3d, const double search_radius) const {
    return point_cloud_common::GetNormal(point_3d, *filtered_point_cloud_, *kdtree_, search_radius);
  }

  const vision_common::FeatureImage& feature_image() const { return *feature_image_; }

  const localization_measurements::DepthImage& depth_image() const { return depth_image_; }

 private:
  localization_measurements::DepthImage depth_image_;
  std::unique_ptr<vision_common::FeatureImage> feature_image_;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_point_cloud_;
};
}  // namespace depth_odometry
#endif  // DEPTH_ODOMETRY_DEPTH_IMAGE_FEATURES_AND_POINTS_H_
