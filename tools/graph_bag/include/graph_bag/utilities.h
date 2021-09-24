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
#include <graph_localizer/graph_localizer.h>
#include <localization_common/utilities.h>
#include <graph_bag/graph_bag_params.h>

#include <opencv2/core/mat.hpp>

#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>

#include <string>
#include <vector>

namespace graph_bag {
// TODO(rsoussan): put these somewhere else!
using Calibration = gtsam::Cal3_S2;
using Camera = gtsam::PinholePose<Calibration>;
using SmartFactor = gtsam::RobustSmartProjectionPoseFactor<Calibration>;

void FeatureTrackImage(const graph_localizer::FeatureTrackIdMap& feature_tracks,
                       const camera::CameraParameters& camera_params, cv::Mat& feature_track_image);
void MarkSmartFactorPoints(const std::vector<const SmartFactor*> smart_factors,
                           const camera::CameraParameters& camera_params, const double feature_track_min_separation,
                           const int max_num_factors, cv::Mat& feature_track_image);
boost::optional<sensor_msgs::ImagePtr> CreateSemanticMatchesImage(const sensor_msgs::ImageConstPtr& image_msg,
                                                                  const std::vector<graph_localizer::SemanticLocFactorAdder::SemanticMatch>& sem_matches,
                                                                  const GraphBagParams& params, bool show_img = false);
boost::optional<sensor_msgs::ImagePtr> CreateFeatureTrackImage(
  const sensor_msgs::ImageConstPtr& image_msg, const graph_localizer::FeatureTrackIdMap& feature_tracks,
  const camera::CameraParameters& camera_params, const std::vector<const SmartFactor*>& smart_factors = {});

cv::Point Distort(const Eigen::Vector2d& undistorted_point, const camera::CameraParameters& params);

std::vector<const SmartFactor*> SmartFactors(const graph_localizer::GraphLocalizer& graph);

bool string_ends_with(const std::string& str, const std::string& ending);

void SaveImuBiasTesterPredictedStates(
  const std::vector<localization_common::CombinedNavState>& imu_bias_tester_predicted_states, rosbag::Bag& bag);

template <typename MsgType>
void SaveMsg(const MsgType& msg, const std::string& topic, rosbag::Bag& bag) {
  const ros::Time timestamp = localization_common::RosTimeFromHeader(msg.header);
  bag.write("/" + topic, timestamp, msg);
}
}  // namespace graph_bag

#endif  // GRAPH_BAG_UTILITIES_H_
