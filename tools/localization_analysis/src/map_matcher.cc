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

#include <ff_common/ff_names.h>
#include <graph_localizer/utilities.h>
#include <localization_analysis/map_matcher.h>
#include <localization_analysis/utilities.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <image_transport/image_transport.h>

#include <vector>

namespace localization_analysis {
namespace lc = localization_common;
namespace mc = msg_conversions;
MapMatcher::MapMatcher(const std::string& input_bag_name, const std::string& map_file, const std::string& image_topic,
                       const std::string& output_bag_name, const std::string& config_prefix,
                       const std::string& save_noloc_imgs)
    : input_bag_(input_bag_name, rosbag::bagmode::Read),
      output_bag_(output_bag_name, rosbag::bagmode::Write),
      nonloc_bag_(),
      image_topic_(image_topic),
      map_(map_file, true),
      map_feature_matcher_(&map_),
      config_prefix_(config_prefix),
      feature_averager_("Total number of features detected"),
      match_count_(0),
      image_count_(0) {
  config_reader::ConfigReader config;
  config.AddFile("geometry.config");
  lc::LoadGraphLocalizerConfig(config, config_prefix);
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  body_T_nav_cam_ = lc::LoadTransform(config, "nav_cam_transform");
  sparse_mapping_min_num_landmarks_ = mc::LoadInt(config, "loc_adder_min_num_matches");
  if (!save_noloc_imgs.empty()) {
    nonloc_bag_.open(save_noloc_imgs, rosbag::bagmode::Write);
  }
}

// TODO(rsoussan): Use common code with graph_bag
bool MapMatcher::GenerateVLFeatures(const sensor_msgs::ImageConstPtr& image_msg,
                                    ff_msgs::VisualLandmarks& vl_features) {
  // Convert image to cv image
  cv_bridge::CvImageConstPtr image;
  try {
    image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  if (!map_feature_matcher_.Localize(image, &vl_features)) return false;
  return true;
}

void MapMatcher::AddMapMatches() {
  std::vector<std::string> topics;
  topics.push_back(std::string("/") + image_topic_);
  rosbag::View view(input_bag_, rosbag::TopicQuery(topics));
  image_count_ = view.size();
  for (const rosbag::MessageInstance msg : view) {
    if (string_ends_with(msg.getTopic(), image_topic_)) {
      sensor_msgs::ImageConstPtr image_msg = msg.instantiate<sensor_msgs::Image>();
      ff_msgs::VisualLandmarks vl_msg;
      if (GenerateVLFeatures(image_msg, vl_msg)) {
        match_count_++;
        feature_averager_.Update(vl_msg.landmarks.size());
        const ros::Time timestamp = lc::RosTimeFromHeader(image_msg->header);
        output_bag_.write(std::string("/") + TOPIC_LOCALIZATION_ML_FEATURES, timestamp, vl_msg);
        if (graph_localizer::ValidVLMsg(vl_msg, sparse_mapping_min_num_landmarks_)) {
          const gtsam::Pose3 sparse_mapping_global_T_body =
            lc::PoseFromMsgWithExtrinsics(vl_msg.pose, body_T_nav_cam_.inverse());
          const auto pose_msg =
            graph_localizer::PoseMsg(lc::EigenPose(sparse_mapping_global_T_body), lc::TimeFromHeader(vl_msg.header));
          output_bag_.write(std::string("/") + TOPIC_SPARSE_MAPPING_POSE, timestamp, pose_msg);
        }
      } else if (nonloc_bag_.isOpen()) {
        const ros::Time timestamp = lc::RosTimeFromHeader(image_msg->header);
        nonloc_bag_.write(std::string("/") + image_topic_, timestamp, image_msg);
      }
    }
  }
}

void MapMatcher::LogResults() {
  std::stringstream ss;
  ss << "Localized " << match_count_ << " / " << image_count_ << " images with mean of " << feature_averager_.average()
     << " features";
  ROS_INFO_STREAM(ss.str());
}
}  // namespace localization_analysis
