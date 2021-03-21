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
#ifndef GRAPH_BAG_UTILITIES_H_
#define GRAPH_BAG_UTILITIES_H_

#include <camera/camera_params.h>
#include <graph_localizer/feature_tracker.h>

#include <opencv2/core/mat.hpp>

#include <sensor_msgs/Image.h>

namespace graph_bag {
void FeatureTrackImage(const graph_localizer::FeatureTrackIdMap& feature_tracks,
                       const camera::CameraParameters& camera_params, cv::Mat& feature_track_image);
boost::optional<sensor_msgs::ImagePtr> CreateFeatureTrackImage(const sensor_msgs::ImageConstPtr& image_msg,
                                                               const graph_localizer::FeatureTrackIdMap& feature_tracks,
                                                               const camera::CameraParameters& camera_params);

cv::Point Distort(const Eigen::Vector2d& undistorted_point, const camera::CameraParameters& params);
}  // namespace graph_bag

#endif  // GRAPH_BAG_UTILITIES_H_
