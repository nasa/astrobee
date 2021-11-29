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

#ifndef LOCALIZATION_MEASUREMENTS_DEPTH_IMAGE_H_
#define LOCALIZATION_MEASUREMENTS_DEPTH_IMAGE_H_

#include <localization_common/time.h>

#include <boost/optional.hpp>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace localization_measurements {
class DepthImage {
 public:
  DepthImage(const cv::Mat& image, const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud);
  // Point lookups using depth image col and rows need to use the unfiltered point cloud since the indices
  // of the unfiltered cloud correlate to the image space indices.
  boost::optional<const pcl::PointXYZI&> UnfilteredPoint3D(const int col, const int row) const;
  boost::optional<const pcl::PointXYZI&> UnfilteredPoint3D(const double col, const double row) const;
  boost::optional<pcl::PointXYZI> InterpolatePoint3D(const double col, const double row) const;
  const cv::Mat& image() const { return image_; }
  const pcl::PointCloud<pcl::PointXYZI>::Ptr unfiltered_point_cloud() const { return unfiltered_point_cloud_; }

 private:
  static bool ValidPoint(const boost::optional<const pcl::PointXYZI&> point);

  cv::Mat image_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr unfiltered_point_cloud_;
};
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_DEPTH_IMAGE_H_
