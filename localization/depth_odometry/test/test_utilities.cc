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

#include "test_utilities.h"  // NOLINT
#include <depth_odometry/point_to_plane_icp_depth_odometry_params.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>
#include <point_cloud_common/test_utilities.h>
#include <vision_common/test_utilities.h>

#include <pcl/common/transforms.h>

#include <cv_bridge/cv_bridge.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace pc = point_cloud_common;
namespace vc = vision_common;

lm::DepthImageMeasurement DefaultDepthImageMeasurement(const lc::Time timestamp) {
  const auto cubic_points = pc::CubicPoints();
  const auto point_cloud = pc::PointCloud<pcl::PointXYZI>(cubic_points.first);
  return lm::DepthImageMeasurement(cv::Mat(), point_cloud, timestamp);
}

lm::DepthImageMeasurement ImageFeatureDepthImageMeasurement(const lc::Time timestamp, const cv::Point2i& offset) {
  int num_markers_added;
  const auto image = vc::MarkerImage(33, 33, num_markers_added, offset);
  const int num_points = image.cols * image.rows;
  const auto points = pc::RandomPoints(num_points);
  const auto point_cloud = pc::PointCloud<pcl::PointXYZI>(points);
  return lm::DepthImageMeasurement(image, point_cloud, timestamp);
}

lm::DepthImageMeasurement TransformDepthImageMeasurement(const lm::DepthImageMeasurement& depth_image_measurement,
                                                         const lc::Time timestamp,
                                                         const Eigen::Isometry3d& target_T_source) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*(depth_image_measurement.depth_image.unfiltered_point_cloud()), *transformed_cloud,
                           Eigen::Affine3d(target_T_source.matrix()));
  return lm::DepthImageMeasurement(depth_image_measurement.depth_image.image(), transformed_cloud, timestamp);
}

lm::DepthImageMeasurement OffsetImageFeatureDepthImageMeasurement(
  const lc::Time timestamp, const lm::DepthImageMeasurement& depth_image_measurement, const cv::Point2i& offset,
  const Eigen::Isometry3d& target_T_source) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*(depth_image_measurement.depth_image.unfiltered_point_cloud()), *transformed_cloud,
                           Eigen::Affine3d(target_T_source.matrix()));
  int num_markers_added;
  const auto offset_image = vc::MarkerImage(33, 33, num_markers_added, offset);
  const int rows = offset_image.rows;
  const int cols = offset_image.cols;
  pcl::PointCloud<pcl::PointXYZI>::Ptr offset_and_transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  const int num_points = transformed_cloud->points.size();
  // Points are in row order
  for (int i = 0; i < num_points; ++i) {
    const int row = i / cols;
    const int col = i - row * cols;
    const int new_row = row + offset.y;
    const int new_col = col + offset.x;
    // std::cout << "i: " << i << ", row: " << row << ", col: " << col << std::endl;
    if (new_row >= rows || new_col >= cols) {
      pcl::PointXYZI zero_point;
      zero_point.x = 0;
      zero_point.y = 0;
      zero_point.z = 0;
      zero_point.intensity = 0;
      offset_and_transformed_cloud->points.emplace_back(zero_point);
    } else {
      const int new_point_index = new_row * cols + new_col;
      // std::cout << "new row: " << new_row << ", new col: " << new_col << ", new index: " << new_point_index <<
      // std::endl;
      offset_and_transformed_cloud->points.emplace_back(transformed_cloud->points[new_point_index]);
    }
  }
  return lm::DepthImageMeasurement(offset_image, offset_and_transformed_cloud, timestamp);
}

sensor_msgs::PointCloud2ConstPtr CubicPointsMsg(const lc::Time timestamp) {
  const auto cubic_points = pc::CubicPoints();
  auto point_cloud = pc::PointCloud<pcl::PointXYZ>(cubic_points.first);
  // DepthImageMeasurement expects a point cloud with a width and height to correlate with an intenisty image
  // TODO(rsoussan): Replace with resize when pcl version is updated
  // point_cloud->resize(20, 15);
  pcl::PointCloud<pcl::PointXYZ> resized_point_cloud(20, 15);
  int i = 0;
  for (const auto& point : point_cloud->points) {
    resized_point_cloud.points[i++] = point;
  }
  sensor_msgs::PointCloud2 msg;
  lc::TimeToHeader(timestamp, msg.header);
  pcl::toROSMsg(resized_point_cloud, msg);
  return sensor_msgs::PointCloud2ConstPtr(new sensor_msgs::PointCloud2(msg));
}

sensor_msgs::PointCloud2ConstPtr TransformPointsMsg(const lc::Time timestamp,
                                                    const sensor_msgs::PointCloud2ConstPtr old_msg,
                                                    const Eigen::Isometry3d& new_T_old) {
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::fromROSMsg(*old_msg, point_cloud);
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud(point_cloud, transformed_cloud, Eigen::Affine3d(new_T_old.matrix()));
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(transformed_cloud, msg);
  lc::TimeToHeader(timestamp, msg.header);
  return sensor_msgs::PointCloud2ConstPtr(new sensor_msgs::PointCloud2(msg));
}

sensor_msgs::ImageConstPtr ImageMsg(const lc::Time timestamp) {
  cv_bridge::CvImage msg_bridge;
  msg_bridge.encoding = sensor_msgs::image_encodings::MONO8;
  msg_bridge.image = cv::Mat(15, 20, CV_8UC1);
  auto msg = msg_bridge.toImageMsg();
  lc::TimeToHeader(timestamp, msg->header);
  return sensor_msgs::ImageConstPtr(msg);
}

PointToPlaneICPDepthOdometryParams DefaultPointToPlaneICPDepthOdometryParams() {
  PointToPlaneICPDepthOdometryParams params;
  params.icp = pc::DefaultPointToPlaneICPParams();
  DefaultDepthOdometryParams(params);
  return params;
}

ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams
DefaultImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams() {
  ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams params;
  params.aligner = pc::DefaultPointCloudWithKnownCorrespondencesAlignerParams();
  params.lk_optical_flow_feature_detector_and_matcher = vc::DefaultLKOpticalFlowFeatureDetectorAndMatcherParams();
  params.detector = "lk_optical_flow";
  params.use_clahe = false;
  params.clahe_grid_length = 8;
  params.clahe_clip_limit = 40;
  params.min_x_distance_to_border = 0;
  params.min_y_distance_to_border = 0;
  params.min_num_inliers = 0;
  DefaultDepthOdometryParams(params);
  return params;
}

void DefaultDepthOdometryParams(DepthOdometryParams& params) {
  params.max_time_diff = 1.0;
  params.position_covariance_threshold = 1.0;
  params.orientation_covariance_threshold = 1.0;
}

DepthOdometryWrapperParams DefaultDepthOdometryWrapperParams() {
  DepthOdometryWrapperParams params;
  params.max_image_and_point_cloud_time_diff = 0.01;
  params.method = "icp";
  params.body_T_haz_cam = Eigen::Isometry3d::Identity();
  params.haz_cam_A_haz_depth = Eigen::Affine3d::Identity();
  params.icp = DefaultPointToPlaneICPDepthOdometryParams();
  // TODO(rsoussan): Fill this in!
  // params.image_feature = ;
  return params;
}
}  // namespace depth_odometry
