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

#include <ff_util/ff_names.h>
#include <graph_bag/utilities.h>
#include <localization_common/utilities.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace graph_bag {
namespace lc = localization_common;
namespace mc = msg_conversions;

void FeatureTrackImage(const graph_localizer::FeatureTrackIdMap& feature_tracks,
                       const camera::CameraParameters& camera_params, cv::Mat& feature_track_image) {
  for (const auto& feature_track : feature_tracks) {
    const auto& points = feature_track.second->points();
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
    if (points.size() > 1) {
      for (auto point_it = points.begin(); point_it != std::prev(points.end()); ++point_it) {
        const auto& point1 = point_it->second.image_point;
        const auto& point2 = std::next(point_it)->second.image_point;
        const auto distorted_previous_point = Distort(point1, camera_params);
        const auto distorted_current_point = Distort(point2, camera_params);
        cv::circle(feature_track_image, distorted_current_point, 2 /* Radius*/, cv::Scalar(0, 255, 255), -1 /*Filled*/,
                   8);
        cv::line(feature_track_image, distorted_current_point, distorted_previous_point, color, 2, 8, 0);
      }
    } else {
      cv::circle(feature_track_image, Distort(points.cbegin()->second.image_point, camera_params), 2 /* Radius*/, color,
                 -1 /*Filled*/, 8);
    }
    // Draw feature id at most recent point
    cv::putText(feature_track_image, std::to_string(points.crbegin()->second.feature_id),
                Distort(points.crbegin()->second.image_point, camera_params), CV_FONT_NORMAL, 0.4,
                cv::Scalar(255, 0, 0));
  }
}

void MarkSmartFactorPoints(const std::vector<const SmartFactor*> smart_factors,
                           const camera::CameraParameters& camera_params, cv::Mat& feature_track_image) {
  for (const auto smart_factor : smart_factors) {
    const auto& point = smart_factor->measured().back();
    const auto distorted_point = Distort(point, camera_params);
    cv::circle(feature_track_image, distorted_point, 15 /* Radius*/, cv::Scalar(200, 100, 0), -1 /*Filled*/, 8);
  }
}

boost::optional<sensor_msgs::ImagePtr> CreateSemanticMatchesImage(const sensor_msgs::ImageConstPtr& image_msg,
                                                                  const std::vector<graph_localizer::SemanticLocFactorAdder::SemanticMatch>& sem_matches,
                                                                  const GraphBagParams& params, bool show_img) {
  float resize_scale = 0.5;
  cv_bridge::CvImagePtr semantic_match_image;
  try {
    semantic_match_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return boost::none;
  }

  cv::Mat undist_viz;
  cv::remap(semantic_match_image->image, undist_viz, params.undist_map_x, params.undist_map_y, cv::INTER_LINEAR);
  cv::Size original_size = semantic_match_image->image.size();
  cv::Point top_left = (undist_viz.size() - original_size)/2;

  cv::Mat &viz = semantic_match_image->image;
  viz = undist_viz(cv::Rect(top_left.x, top_left.y, original_size.width, original_size.height));

  // This is rather ugly to hardcode
  for (const auto& match : sem_matches) {
    cv::Point size_half(viz.size().width/2., viz.size().height/2.);
    cv::Point map_pt;

    if (match.have_map_point) {
      map_pt = cv::Point(match.map_point_px[0], match.map_point_px[1]) + size_half;
      cv::circle(viz, map_pt, 5/resize_scale, cv::Scalar(0,255,0), cv::FILLED);
      if (params.class_names.count(match.cls) > 0) {
        cv::putText(viz, params.class_names.at(match.cls), map_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5/resize_scale, cv::Scalar(0,255,0), 2/resize_scale);
      }
    }

    if (match.have_matched_det) {
      cv::Point center = cv::Point(match.det_center_px[0], match.det_center_px[1]) + size_half;
      cv::Point size = cv::Point(match.det_bbox_size_px[0], match.det_bbox_size_px[1]);
      if (match.have_map_point) {
        cv::rectangle(viz, center - size/2, center + size/2, cv::Scalar(255,0,0), 2/resize_scale);
        cv::line(viz, center, map_pt, cv::Scalar(0,0,255), 2/resize_scale, cv::LINE_AA);
      } else {
        cv::rectangle(viz, center - size/2, center + size/2, cv::Scalar(255,0,0), 1/resize_scale);
      }
    }
  }

  if (show_img) {
    cv::imshow("Semantic Matches", viz);
    cv::waitKey(2);
  }

  // cut image in half
  cv::resize(viz, viz, cv::Size(0,0), 0.5, 0.5);

  return semantic_match_image->toImageMsg();
}


boost::optional<sensor_msgs::ImagePtr> CreateFeatureTrackImage(const sensor_msgs::ImageConstPtr& image_msg,
                                                               const graph_localizer::FeatureTrackIdMap& feature_tracks,
                                                               const camera::CameraParameters& camera_params,
                                                               const std::vector<const SmartFactor*>& smart_factors) {
  cv_bridge::CvImagePtr feature_track_image;
  try {
    feature_track_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return boost::none;
  }

  FeatureTrackImage(feature_tracks, camera_params, feature_track_image->image);
  MarkSmartFactorPoints(smart_factors, camera_params, feature_track_image->image);
  return feature_track_image->toImageMsg();
}

cv::Point Distort(const Eigen::Vector2d& undistorted_point, const camera::CameraParameters& params) {
  Eigen::Vector2d distorted_point;
  params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undistorted_point, &distorted_point);
  return cv::Point(distorted_point.x(), distorted_point.y());
}

std::vector<const SmartFactor*> SmartFactors(const graph_localizer::GraphLocalizer& graph) {
  std::vector<const SmartFactor*> smart_factors;
  for (const auto factor : graph.graph_factors()) {
    const auto smart_factor = dynamic_cast<const SmartFactor*>(factor.get());
    if (smart_factor) {
      smart_factors.emplace_back(smart_factor);
    }
  }
  return smart_factors;
}

bool string_ends_with(const std::string& str, const std::string& ending) {
  if (str.length() >= ending.length()) {
    return (0 == str.compare(str.length() - ending.length(), ending.length(), ending));
  } else {
    return false;
  }
}

void SaveImuBiasTesterPredictedStates(const std::vector<lc::CombinedNavState>& imu_bias_tester_predicted_states,
                                      rosbag::Bag& bag) {
  for (const auto& state : imu_bias_tester_predicted_states) {
    geometry_msgs::PoseStamped pose_msg;
    lc::PoseToMsg(state.pose(), pose_msg.pose);
    lc::TimeToHeader(state.timestamp(), pose_msg.header);
    SaveMsg(pose_msg, TOPIC_IMU_BIAS_TESTER_POSE, bag);
    geometry_msgs::Vector3Stamped velocity_msg;
    mc::VectorToMsg(state.velocity(), velocity_msg.vector);
    lc::TimeToHeader(state.timestamp(), velocity_msg.header);
    SaveMsg(velocity_msg, TOPIC_IMU_BIAS_TESTER_VELOCITY, bag);
  }
}
}  // namespace graph_bag
