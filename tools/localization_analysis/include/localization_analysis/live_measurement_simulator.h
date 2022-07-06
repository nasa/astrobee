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

#ifndef LOCALIZATION_ANALYSIS_LIVE_MEASUREMENT_SIMULATOR_H_
#define LOCALIZATION_ANALYSIS_LIVE_MEASUREMENT_SIMULATOR_H_

#include <ff_common/utils.h>
#include <ff_msgs/DepthOdometry.h>
#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/VisualLandmarks.h>
#include <ff_util/ff_names.h>
#include <vision_msgs/Detection2DArray.h>
#include <localization_analysis/live_measurement_simulator_params.h>
#include <localization_analysis/message_buffer.h>
#include <lk_optical_flow/lk_optical_flow.h>
#include <localization_common/time.h>
#include <localization_node/localization.h>
#include <sparse_mapping/sparse_map.h>

#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <map>
#include <string>
#include <utility>

namespace localization_analysis {
class LiveMeasurementSimulator {
 public:
  explicit LiveMeasurementSimulator(const LiveMeasurementSimulatorParams& params);

  bool ProcessMessage();

  localization_common::Time CurrentTime();

  boost::optional<sensor_msgs::ImageConstPtr> GetImageMessage(const localization_common::Time current_time);
  boost::optional<sensor_msgs::Imu> GetImuMessage(const localization_common::Time current_time);
  boost::optional<ff_msgs::Feature2dArray> GetOFMessage(const localization_common::Time current_time);
  boost::optional<ff_msgs::DepthOdometry> GetDepthOdometryMessage(const localization_common::Time current_time);
  boost::optional<ff_msgs::VisualLandmarks> GetVLMessage(const localization_common::Time current_time);
  boost::optional<ff_msgs::VisualLandmarks> GetARMessage(const localization_common::Time current_time);
  boost::optional<vision_msgs::Detection2DArray> GetSMMessage(const localization_common::Time current_time);
  boost::optional<ff_msgs::FlightMode> GetFlightModeMessage(const localization_common::Time current_time);

 private:
  ff_msgs::Feature2dArray GenerateOFFeatures(const sensor_msgs::ImageConstPtr& image_msg);

  bool GenerateVLFeatures(const sensor_msgs::ImageConstPtr& image_msg, ff_msgs::VisualLandmarks& vl_features);

  rosbag::Bag bag_;
  sparse_mapping::SparseMap map_;
  localization_node::Localizer map_feature_matcher_;
  LiveMeasurementSimulatorParams params_;
  lk_optical_flow::LKOpticalFlow optical_flow_tracker_;
  const std::string kImageTopic_;
  std::unique_ptr<rosbag::View> view_;
  boost::optional<rosbag::View::iterator> view_it_;
  std::map<localization_common::Time, sensor_msgs::ImageConstPtr> img_buffer_;
  MessageBuffer<sensor_msgs::Imu> imu_buffer_;
  MessageBuffer<ff_msgs::DepthOdometry> depth_odometry_buffer_;
  MessageBuffer<ff_msgs::FlightMode> flight_mode_buffer_;
  MessageBuffer<ff_msgs::Feature2dArray> of_buffer_;
  MessageBuffer<ff_msgs::VisualLandmarks> vl_buffer_;
  MessageBuffer<ff_msgs::VisualLandmarks> ar_buffer_;
  MessageBuffer<vision_msgs::Detection2DArray> sm_buffer_;
  localization_common::Time current_time_;
};
}  // namespace localization_analysis

#endif  // LOCALIZATION_ANALYSIS_LIVE_MEASUREMENT_SIMULATOR_H_
