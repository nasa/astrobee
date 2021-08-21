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
  boost::optional<pcl::PointXYZI> InterpolatePoint3D(const double col, const double row) {
    double col_alpha = col - std::floor(col);
    double row_alpha = row - std::floor(row);
    const auto floor_col_floor_row_point = Point3D(std::floor(col), std::floor(row));
    const auto ceil_col_floor_row_point = Point3D(std::ceil(col), std::floor(row));
    const auto floor_col_ceil_row_point = Point3D(std::floor(col), std::ceil(row));
    const auto ceil_col_ceil_row_point = Point3D(std::ceil(col), std::ceil(row));
    if (!ValidAccessedPoint(floor_col_floor_row_point) || !ValidAccessedPoint(ceil_col_floor_row_point) ||
        !ValidAccessedPoint(floor_col_ceil_row_point) || !ValidAccessedPoint(ceil_col_ceil_row_point))
      return boost::none;
    // Interpolate column points for each row option using col weights, then interpolate these results using row weights
    const auto col_interpolated_floor_row =
      Interpolate(col_alpha, *floor_col_floor_row_point, *ceil_col_floor_row_point);
    const auto col_interpolated_ceil_row = Interpolate(col_alpha, *floor_col_ceil_row_point, *ceil_col_ceil_row_point);
    return Interpolate(row_alpha, col_interpolated_floor_row, col_interpolated_ceil_row);
  }

  cv::Mat image;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud;

 private:
  pcl::PointXYZI Interpolate(const double alpha, const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b) {
    const double beta = 1.0 - alpha;
    pcl::PointXYZI interpolated_point;
    interpolated_point.x = beta * point_a.x + alpha * point_b.x;
    interpolated_point.y = beta * point_a.y + alpha * point_b.y;
    interpolated_point.z = beta * point_a.z + alpha * point_b.z;
    interpolated_point.intensity = beta * point_a.intensity + alpha * point_b.intensity;
    return interpolated_point;
  }

  bool ValidAccessedPoint(const boost::optional<const pcl::PointXYZI&> point) { return point && ValidPoint(*point); }

  // TODO(rsoussan): Unify this with depth odometry point_cloud_utilities.cc
  bool ValidPoint(const pcl::PointXYZI& point) {
    const bool finite_point = pcl_isfinite(point.x) && pcl_isfinite(point.y) && pcl_isfinite(point.z);
    const bool nonzero_point = point.x != 0 || point.y != 0 || point.z != 0;
    const bool valid_intensity = pcl_isfinite(point.intensity);
    return finite_point && nonzero_point && valid_intensity;
  }
};
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_DEPTH_IMAGE_MEASUREMENT_H_
