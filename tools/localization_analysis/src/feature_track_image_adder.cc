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

#include <localization_analysis/feature_track_image_adder.h>

#include <cv_bridge/cv_bridge.h>

namespace localization_analysis {
namespace vc = vision_common;

void FeatureTrackImage(const vc::SpacedFeatureTracker& feature_tracker, const camera::CameraParameters& camera_params,
                       cv::Mat& feature_track_image) {
  const auto& feature_tracks = feature_tracker.feature_tracks();
  for (const auto& feature_track_pair : feature_tracks) {
    const auto& feature_track = feature_track_pair.second;
    // Color using unique color for id
    const int id = feature_track.set().crbegin()->second.feature_track_id;
    const cv::Scalar color = cv::Scalar((id * 123) % 255, (id * 456) % 255, (id * 789) % 255);

    // Draw track history
    if (feature_track.size() > 1) {
      for (auto point_it = feature_track.set().begin(); point_it != std::prev(feature_track.set().end()); ++point_it) {
        const auto& point1 = point_it->second.image_point;
        const auto& point2 = std::next(point_it)->second.image_point;
        const auto distorted_previous_point = Distort(point1, camera_params);
        const auto distorted_current_point = Distort(point2, camera_params);
        cv::circle(feature_track_image, distorted_current_point, 2, cv::Scalar(0, 255, 255), -1, 8);
        cv::line(feature_track_image, distorted_current_point, distorted_previous_point, color, 2, 8, 0);
      }
    } else {
      cv::circle(feature_track_image, Distort(feature_track.set().crbegin()->second.image_point, camera_params), 2,
                 color, -1, 8);
    }
    // Draw feature id at most recent point
    cv::putText(feature_track_image, std::to_string(feature_track.set().crbegin()->second.feature_track_id),
                Distort(feature_track.set().crbegin()->second.image_point, camera_params), cv::FONT_HERSHEY_PLAIN, 2,
                cv::Scalar(255, 0, 0));
  }
}

void MarkSmartFactorPoints(const std::vector<boost::shared_ptr<const factor_adders::RobustSmartFactor>> smart_factors,
                           const camera::CameraParameters& camera_params, const gtsam::Values& values,
                           cv::Mat& feature_track_image) {
  for (const auto& smart_factor : smart_factors) {
    const auto& points = smart_factor->measured();
    const auto& latest_point = points.front();
    const auto distorted_point = Distort(latest_point, camera_params);
    const cv::Point2f rectangle_offset(25, 25);
    cv::rectangle(feature_track_image, distorted_point + rectangle_offset, distorted_point - rectangle_offset,
                  cv::Scalar(200, 100, 0), 4, 8);

    // Draw track history using gray points
    for (const auto& point : points) {
      const auto distorted_point = Distort(point, camera_params);
      cv::circle(feature_track_image, distorted_point, 6 /* Radius*/, cv::Scalar(100, 100, 100), -1 /*Filled*/, 8);
    }
    // Draw reprojected triangulated point in blue, draw red line to latest measurement showing projection error
    const auto triangulated_point = smart_factor->serialized_point(values);
    const auto cameras = smart_factor->cameras(values);

    // Cameras are in same order as keys
    const auto& measurements = smart_factor->measured();
    if (triangulated_point) {
      // Latest camera is first camera
      const auto projected_point = cameras.front().project2(*triangulated_point);
      const auto distorted_projected_point = localization_analysis::Distort(projected_point, camera_params);
      cv::circle(feature_track_image, distorted_projected_point, 7 /* Radius*/, cv::Scalar(255, 0, 0), -1 /*Filled*/,
                 8);
      cv::line(feature_track_image, distorted_projected_point, distorted_point, cv::Scalar(0, 0, 255), 2, 8, 0);
    }
  }
}

boost::optional<sensor_msgs::ImagePtr> CreateFeatureTrackImage(
  const sensor_msgs::ImageConstPtr& image_msg, const vc::SpacedFeatureTracker& feature_tracker,
  const camera::CameraParameters& camera_params,
  const std::vector<boost::shared_ptr<const factor_adders::RobustSmartFactor>>& smart_factors,
  const gtsam::Values& values) {
  cv_bridge::CvImagePtr feature_track_image;
  try {
    feature_track_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return boost::none;
  }

  FeatureTrackImage(feature_tracker, camera_params, feature_track_image->image);
  MarkSmartFactorPoints(smart_factors, camera_params, values, feature_track_image->image);
  return feature_track_image->toImageMsg();
}

cv::Point2f Distort(const Eigen::Vector2d& undistorted_point, const camera::CameraParameters& params) {
  Eigen::Vector2d distorted_point;
  params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undistorted_point, &distorted_point);
  return cv::Point2f(distorted_point.x(), distorted_point.y());
}
}  // namespace localization_analysis
