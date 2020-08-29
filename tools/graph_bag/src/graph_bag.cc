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

#include <ff_common/utils.h>
#include <ff_util/ff_names.h>
#include <graph_bag/graph_bag.h>
#include <graph_localizer/utilities.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>
#include <msg_conversions/msg_conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <ros/time.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <glog/logging.h>

#include <chrono>
#include <cstdlib>
#include <vector>

namespace {
cv::Point Distort(const Eigen::Vector2d& undistorted_point, const camera::CameraParameters& params) {
  Eigen::Vector2d distorted_point;
  params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undistorted_point, &distorted_point);
  return cv::Point(distorted_point.x(), distorted_point.y());
}
}  // namespace

namespace graph_bag {
namespace lc = localization_common;

GraphBag::GraphBag(const std::string& bag_name, const std::string& map_file, const std::string& image_topic,
                   const std::string& results_bag)
    : bag_(bag_name, rosbag::bagmode::Read),
      map_(map_file, true),
      map_feature_matcher_(&map_),
      kImageTopic_(image_topic),
      results_bag_(results_bag, rosbag::bagmode::Write) {
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");
  config.AddFile("localization.config");
  config.AddFile("optical_flow.config");

  if (!config.ReadFiles()) {
    ROS_FATAL("Failed to read config files.");
    // TODO(rsoussan): read params somewhere else!!!
    exit(0);
  }

  map_feature_matcher_.ReadParams(&config);
  optical_flow_tracker_.ReadParams(&config);
  // Needed for feature tracks visualization
  nav_cam_params_.reset(new camera::CameraParameters(&config, "nav_cam"));
  body_T_nav_cam_ = lc::LoadTransform(config, "nav_cam_transform");
}

// TODO(rsoussan): remove this? cite leo?
// TODO(rsoussan): draw latest tracks as circles?
// TODO(rsoussan): draw larger arrow for most recent track
void GraphBag::FeatureTrackImage(const graph_localizer::FeatureTrackMap& feature_tracks,
                                 cv::Mat& feature_track_image) const {
  int num_feature_tracks = 0;
  int longest_track = 0;

  for (const auto& feature_track : feature_tracks) {
    const auto& points = feature_track.second.points;
    // change color based on track length
    cv::Scalar color(255, 255, 0, 1);  // yellow
    if (points.size() > 5) {
      color = cv::Scalar(50, 255, 50, 1);  // green
    } else {
      if (points.size() <= 1) {
        color = cv::Scalar(255, 0, 0, 1);  // red
        const auto distorted_point = Distort(points[0].image_point, *nav_cam_params_);
        cv::circle(feature_track_image, distorted_point, 3, color);
        continue;
      }
    }

    for (int i = 1; i < points.size(); ++i) {
      const auto distorted_current_point = Distort(points[i].image_point, *nav_cam_params_);
      const auto distorted_previous_point = Distort(points[i - 1].image_point, *nav_cam_params_);
      arrowedLine(feature_track_image, distorted_previous_point, distorted_current_point, color);
    }
    if (points.size() > longest_track) {
      longest_track = points.size();
    }
    ++num_feature_tracks;
  }

  DLOG(INFO) << "FeatureTrackImage: Drew " << num_feature_tracks << " tracks, the longest was: " << longest_track;
}

bool string_ends_with(const std::string& str, const std::string& ending) {
  if (str.length() >= ending.length()) {
    return (0 == str.compare(str.length() - ending.length(), ending.length(), ending));
  } else {
    return false;
  }
}

ff_msgs::Feature2dArray GraphBag::GenerateOFFeatures(const sensor_msgs::ImageConstPtr& image_msg) {
  ff_msgs::Feature2dArray of_features;
  optical_flow_tracker_.OpticalFlow(image_msg, &of_features);
  return of_features;
}

bool GraphBag::GenerateVLFeatures(const sensor_msgs::ImageConstPtr& image_msg, ff_msgs::VisualLandmarks& vl_features) {
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

void GraphBag::SaveSparseMappingPoseMsg(const geometry_msgs::PoseStamped& sparse_mapping_pose_msg) {
  const ros::Time timestamp = lc::RosTimeFromHeader(sparse_mapping_pose_msg.header);
  results_bag_.write(TOPIC_SPARSE_MAPPING_POSE, timestamp, sparse_mapping_pose_msg);
}

void GraphBag::SaveOpticalFlowTracksImage(const sensor_msgs::ImageConstPtr& image_msg,
                                          const graph_localizer::FeatureTrackMap* const feature_tracks) {
  if (feature_tracks == nullptr) return;
  cv_bridge::CvImagePtr feature_track_image;
  try {
    feature_track_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  FeatureTrackImage(*feature_tracks, feature_track_image->image);
  const auto feature_track_image_msg = feature_track_image->toImageMsg();
  const ros::Time timestamp = lc::RosTimeFromHeader(image_msg->header);
  results_bag_.write(kFeatureTracksImageTopic_, timestamp, *feature_track_image_msg);
}

void GraphBag::SaveLocState(const ff_msgs::EkfState& loc_msg, const std::string& topic) {
  const ros::Time timestamp = lc::RosTimeFromHeader(loc_msg.header);
  results_bag_.write(topic, timestamp, loc_msg);
}

void GraphBag::Run() {
  std::vector<std::string> topics;
  topics.push_back(std::string("/") + TOPIC_HARDWARE_IMU);
  topics.push_back(TOPIC_HARDWARE_IMU);
  topics.push_back(std::string("/") + kImageTopic_);
  topics.push_back(kImageTopic_);
  // Only use recorded ar features
  topics.push_back(std::string("/") + TOPIC_LOCALIZATION_AR_FEATURES);
  topics.push_back(TOPIC_LOCALIZATION_AR_FEATURES);

  rosbag::View view(bag_, rosbag::TopicQuery(topics));
  // Required to start bias estimation
  graph_localizer_wrapper_.ResetBiasesAndLocalizer();
  const auto start_time = std::chrono::steady_clock::now();
  for (rosbag::MessageInstance const m : view) {
    if (string_ends_with(m.getTopic(), TOPIC_HARDWARE_IMU)) {
      sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      graph_localizer_wrapper_.ImuCallback(*imu_msg);
      imu_augmentor_wrapper_.ImuCallback(*imu_msg);

      // Save imu augmented loc msg if available
      const auto imu_augmented_loc_msg = imu_augmentor_wrapper_.LatestImuAugmentedLocalizationMsg();
      if (!imu_augmented_loc_msg) {
        LOG_EVERY_N(WARNING, 50) << "Run: Failed to get latest imu augmented loc msg.";
      } else {
        SaveLocState(*imu_augmented_loc_msg, TOPIC_GNC_EKF);
      }
    } else if (string_ends_with(m.getTopic(), kImageTopic_)) {
      sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();

      // Handle of features before vl features since graph requires states to
      // exist at timestamp already when adding vl features, and these states
      // are created when adding of features
      const ff_msgs::Feature2dArray of_features = GenerateOFFeatures(image_msg);
      graph_localizer_wrapper_.OpticalFlowCallback(of_features);
      SaveOpticalFlowTracksImage(image_msg, graph_localizer_wrapper_.feature_tracks());

      // Handle vl features
      ff_msgs::VisualLandmarks vl_features;
      if (GenerateVLFeatures(image_msg, vl_features)) {
        graph_localizer_wrapper_.VLVisualLandmarksCallback(vl_features);
        const auto sparse_mapping_pose_msg = graph_localizer_wrapper_.LatestSparseMappingPoseMsg();
        if (sparse_mapping_pose_msg) {
          SaveSparseMappingPoseMsg(*sparse_mapping_pose_msg);
        }
      }

      // Save latest graph localization msg, which should have just been optimized after adding of and/or vl features.
      // Pass latest loc state to imu augmentor if it is available.
      const auto localization_msg = graph_localizer_wrapper_.LatestLocalizationStateMsg();
      if (!localization_msg) {
        LOG_EVERY_N(WARNING, 50) << "Run: Failed to get localization msg.";
      } else {
        imu_augmentor_wrapper_.LocalizationStateCallback(*localization_msg);
        SaveLocState(*localization_msg, TOPIC_GRAPH_LOC_STATE);
      }
    } else if (string_ends_with(m.getTopic(), TOPIC_LOCALIZATION_AR_FEATURES)) {
      const ff_msgs::VisualLandmarksConstPtr vl_features = m.instantiate<ff_msgs::VisualLandmarks>();
      graph_localizer_wrapper_.ARVisualLandmarksCallback(*vl_features);
    } else {
      continue;
    }
  }
  const auto end_time = std::chrono::steady_clock::now();
  LOG(INFO) << "Total run time: " << std::chrono::duration<double>(end_time - start_time).count() << " seconds.";
}
}  // namespace graph_bag
