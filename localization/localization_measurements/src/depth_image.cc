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

namespace localization_measurements {
namespace pc = point_cloud_common;
DepthImage::DepthImage(const cv::Mat& image, const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud)
    : image_(image), unfiltered_point_cloud_(point_cloud) {}

boost::optional<const pcl::PointXYZI&> DepthImage::UnfilteredPoint3D(const int col, const int row) const {
  if (col >= image_.cols || row >= image_.rows) return boost::none;
  return unfiltered_point_cloud_->points[image_.cols * row + col];
}
boost::optional<const pcl::PointXYZI&> DepthImage::UnfilteredPoint3D(const double col, const double row) const {
  return UnfilteredPoint3D(static_cast<int>(std::round(col)), static_cast<int>(std::round(row)));
}
boost::optional<pcl::PointXYZI> DepthImage::InterpolatePoint3D(const double col, const double row) const {
  double col_alpha = col - std::floor(col);
  double row_alpha = row - std::floor(row);
  const auto floor_col_floor_row_point = UnfilteredPoint3D(std::floor(col), std::floor(row));
  const auto ceil_col_floor_row_point = UnfilteredPoint3D(std::ceil(col), std::floor(row));
  const auto floor_col_ceil_row_point = UnfilteredPoint3D(std::floor(col), std::ceil(row));
  const auto ceil_col_ceil_row_point = UnfilteredPoint3D(std::ceil(col), std::ceil(row));
  if (!ValidPoint(floor_col_floor_row_point) || !ValidPoint(ceil_col_floor_row_point) ||
      !ValidPoint(floor_col_ceil_row_point) || !ValidPoint(ceil_col_ceil_row_point))
    return boost::none;
  // Interpolate column points for each row option using col weights, then interpolate these results using row weights
  const auto col_interpolated_floor_row =
    pc::Interpolate(col_alpha, *floor_col_floor_row_point, *ceil_col_floor_row_point);
  const auto col_interpolated_ceil_row =
    pc::Interpolate(col_alpha, *floor_col_ceil_row_point, *ceil_col_ceil_row_point);
  return pc::Interpolate(row_alpha, col_interpolated_floor_row, col_interpolated_ceil_row);
}

bool DepthImage::ValidPoint(const boost::optional<const pcl::PointXYZI&> point) {
  return point && pc::ValidPoint(*point);
}
}  // namespace localization_measurements
