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

#include <graph_bag/utilities.h>
#include <localization_common/utilities.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc.hpp>

namespace graph_bag {
void FeatureTrackImage(const graph_localizer::FeatureTrackMap& feature_tracks,
                       const camera::CameraParameters& camera_params, cv::Mat& feature_track_image) {
  for (const auto& feature_track : feature_tracks) {
    const auto& points = feature_track.second.points;
    cv::Scalar color;
    if (points.size() <= 1) {
      // Red for single point tracks
      color = cv::Scalar(50, 255, 50, 1);
    } else if (points.size() < 3) {
      // Yellow for medium length tracks
      color = cv::Scalar(255, 255, 0, 1);
    } else {
      // Green for long tracks
      color = cv::Scalar(50, 255, 50, 1);
    }

    // Draw track history
    for (int i = 0; i < static_cast<int>(points.size()) - 1; ++i) {
      const auto distorted_previous_point = Distort(points[i].image_point, camera_params);
      const auto distorted_current_point = Distort(points[i + 1].image_point, camera_params);
      cv::circle(feature_track_image, distorted_current_point, 2 /* Radius*/, cv::Scalar(0, 255, 255), -1 /*Filled*/,
                 8);
      cv::line(feature_track_image, distorted_current_point, distorted_previous_point, color, 2, 8, 0);
    }
    // Account for single point tracks
    if (points.size() == 1) {
      cv::circle(feature_track_image, Distort(points[0].image_point, camera_params), 2 /* Radius*/, color,
                 -1 /*Filled*/, 8);
    }
    // Draw feature id at most recent point
    cv::putText(feature_track_image, std::to_string(points[points.size() - 1].feature_id),
                Distort(points[points.size() - 1].image_point, camera_params), CV_FONT_NORMAL, 0.4,
                cv::Scalar(255, 0, 0));
  }
}

boost::optional<sensor_msgs::ImagePtr> CreateFeatureTrackImage(const sensor_msgs::ImageConstPtr& image_msg,
                                                               const graph_localizer::FeatureTrackMap& feature_tracks,
                                                               const camera::CameraParameters& camera_params) {
  cv_bridge::CvImagePtr feature_track_image;
  try {
    feature_track_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return boost::none;
  }

  FeatureTrackImage(feature_tracks, camera_params, feature_track_image->image);
  return feature_track_image->toImageMsg();
}

cv::Point Distort(const Eigen::Vector2d& undistorted_point, const camera::CameraParameters& params) {
  Eigen::Vector2d distorted_point;
  params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undistorted_point, &distorted_point);
  return cv::Point(distorted_point.x(), distorted_point.y());
}
}  // namespace graph_bag
