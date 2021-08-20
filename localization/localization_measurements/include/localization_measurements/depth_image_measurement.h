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

#ifndef LOCALIZATION_MEASUREMENTS_DEPTH_IMAGE_MEASUREMENT_H_
#define LOCALIZATION_MEASUREMENTS_DEPTH_IMAGE_MEASUREMENT_H_

#include <localization_common/time.h>
#include <localization_measurements/measurement.h>

#include <boost/optional.hpp>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace localization_measurements {
struct DepthImageMeasurement : public Measurement {
  DepthImageMeasurement(const cv::Mat& image, const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud,
                        const localization_common::Time timestamp)
      : Measurement(timestamp), image(image), point_cloud(point_cloud) {}

  boost::optional<const pcl::PointXYZI&> Point3D(const int col, const int row) {
    if (col >= image.cols || row >= image.rows) return boost::none;
    return point_cloud->points[image.cols * row + col];
  }
  boost::optional<const pcl::PointXYZI&> Point3D(const double col, const double row) {
    return Point3D(static_cast<int>(std::round(col)), static_cast<int>(std::round(row)));
  }
  cv::Mat image;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud;
};
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_DEPTH_IMAGE_MEASUREMENT_H_
