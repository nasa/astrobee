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
#include <graph_bag/parameter_reader.h>
#include <graph_localizer/utilities.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Core>

#include <opencv2/highgui/highgui_c.h>

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
    : results_bag_(results_bag, rosbag::bagmode::Write) {
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");
  config.AddFile("tools/graph_bag.config");

  if (!config.ReadFiles()) {
    ROS_FATAL("Failed to read config files.");
    exit(0);
  }

  LiveMeasurementSimulatorParams params;
  LoadLiveMeasurementSimulatorParams(config, bag_name, map_file, image_topic, params);
  live_measurement_simulator_.reset(new LiveMeasurementSimulator(params));
  save_optical_flow_images_ = params.save_optical_flow_images;
  // Needed for feature tracks visualization
  nav_cam_params_.reset(new camera::CameraParameters(&config, "nav_cam"));
}

void GraphBag::FeatureTrackImage(const graph_localizer::FeatureTrackMap& feature_tracks,
                                 cv::Mat& feature_track_image) const {
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
    for (int i = 0; i < points.size() - 1; ++i) {
      const auto distorted_previous_point = Distort(points[i].image_point, *nav_cam_params_);
      const auto distorted_current_point = Distort(points[i + 1].image_point, *nav_cam_params_);
      cv::circle(feature_track_image, distorted_current_point, 2 /* Radius*/, cv::Scalar(0, 255, 255), -1 /*Filled*/,
                 8);
      cv::line(feature_track_image, distorted_current_point, distorted_previous_point, color, 2, 8, 0);
    }
    // Account for single point tracks
    if (points.size() == 1) {
      cv::circle(feature_track_image, Distort(points[0].image_point, *nav_cam_params_), 2 /* Radius*/, color,
                 -1 /*Filled*/, 8);
    }
    // Draw feature id at most recent point
    cv::putText(feature_track_image, std::to_string(points[points.size() - 1].feature_id),
                Distort(points[points.size() - 1].image_point, *nav_cam_params_), CV_FONT_NORMAL, 0.4,
                cv::Scalar(255, 0, 0));
  }
}

void GraphBag::SaveSparseMappingPoseMsg(const geometry_msgs::PoseStamped& sparse_mapping_pose_msg) {
  const ros::Time timestamp = lc::RosTimeFromHeader(sparse_mapping_pose_msg.header);
  results_bag_.write("/" + std::string(TOPIC_SPARSE_MAPPING_POSE), timestamp, sparse_mapping_pose_msg);
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
  results_bag_.write("/" + kFeatureTracksImageTopic_, timestamp, *feature_track_image_msg);
}

void GraphBag::SaveImuBiasTesterPredictedStates(
    const std::vector<lc::CombinedNavState>& imu_bias_tester_predicted_states) {
  for (const auto state : imu_bias_tester_predicted_states) {
    geometry_msgs::PoseStamped pose_msg;
    lc::PoseToMsg(state.pose(), pose_msg.pose);
    lc::TimeToHeader(state.timestamp(), pose_msg.header);
    results_bag_.write("/" + kImuBiasTesterPoseTopic_ + "/pose", ros::Time(state.timestamp()), pose_msg);
  }
}

void GraphBag::SaveLocState(const ff_msgs::EkfState& loc_msg, const std::string& topic) {
  const ros::Time timestamp = lc::RosTimeFromHeader(loc_msg.header);
  results_bag_.write("/" + topic, timestamp, loc_msg);
}

void GraphBag::Run() {
  // Required to start bias estimation
  graph_localizer_wrapper_.ResetBiasesAndLocalizer();
  const auto start_time = std::chrono::steady_clock::now();
  while (live_measurement_simulator_->ProcessMessage()) {
    const lc::Time current_time = live_measurement_simulator_->CurrentTime();
    const auto imu_msg = live_measurement_simulator_->GetImuMessage(current_time);
    if (imu_msg) {
      graph_localizer_wrapper_.ImuCallback(*imu_msg);
      imu_augmentor_wrapper_.ImuCallback(*imu_msg);
      imu_bias_tester_wrapper_.ImuCallback(*imu_msg);

      // Save imu augmented loc msg if available
      const auto imu_augmented_loc_msg = imu_augmentor_wrapper_.LatestImuAugmentedLocalizationMsg();
      if (!imu_augmented_loc_msg) {
        LOG_EVERY_N(WARNING, 50) << "Run: Failed to get latest imu augmented loc msg.";
      } else {
        SaveLocState(*imu_augmented_loc_msg, TOPIC_GNC_EKF);
      }
    }
    const auto of_msg = live_measurement_simulator_->GetOFMessage(current_time);
    if (of_msg) {
      // Handle of features before vl features since graph requires states to
      // exist at timestamp already when adding vl features, and these states
      // are created when adding of features
      graph_localizer_wrapper_.OpticalFlowCallback(*of_msg);
      if (save_optical_flow_images_) {
        const auto img_msg = live_measurement_simulator_->GetImageMessage(lc::TimeFromHeader(of_msg->header));
        if (img_msg) SaveOpticalFlowTracksImage(*img_msg, graph_localizer_wrapper_.feature_tracks());
      }

      // Save latest graph localization msg, which should have just been optimized after adding of and/or vl features.
      // Pass latest loc state to imu augmentor if it is available.
      const auto localization_msg = graph_localizer_wrapper_.LatestLocalizationStateMsg();
      if (!localization_msg) {
        LOG_EVERY_N(WARNING, 50) << "Run: Failed to get localization msg.";
      } else {
        imu_augmentor_wrapper_.LocalizationStateCallback(*localization_msg);
        SaveLocState(*localization_msg, TOPIC_GRAPH_LOC_STATE);
        const auto imu_bias_tester_predicted_states =
            imu_bias_tester_wrapper_.LocalizationStateCallback(*localization_msg);
        SaveImuBiasTesterPredictedStates(imu_bias_tester_predicted_states);
      }
    }
    const auto vl_msg = live_measurement_simulator_->GetVLMessage(current_time);
    if (vl_msg) {
      graph_localizer_wrapper_.VLVisualLandmarksCallback(*vl_msg);
      const auto sparse_mapping_pose_msg = graph_localizer_wrapper_.LatestSparseMappingPoseMsg();
      if (sparse_mapping_pose_msg) {
        SaveSparseMappingPoseMsg(*sparse_mapping_pose_msg);
      }
    }
    const auto ar_msg = live_measurement_simulator_->GetARMessage(current_time);
    if (ar_msg) {
      graph_localizer_wrapper_.ARVisualLandmarksCallback(*ar_msg);
    }
  }
  const auto end_time = std::chrono::steady_clock::now();
  LOG(INFO) << "Total run time: " << std::chrono::duration<double>(end_time - start_time).count() << " seconds.";
}
}  // namespace graph_bag
