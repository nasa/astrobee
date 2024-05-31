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

#include <localization_analysis/parameter_reader.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

namespace localization_analysis {
namespace lc = localization_common;
namespace mc = msg_conversions;

void LoadMessageBufferParams(const std::string& message_type, config_reader::ConfigReader& config,
                             MessageBufferParams& params, const std::string& prefix) {
  LOAD_PARAM(params.msg_delay, config, prefix + message_type + "_");
  LOAD_PARAM(params.min_msg_spacing, config, prefix + message_type + "_");
}

void LoadLiveMeasurementSimulatorParams(config_reader::ConfigReader& config, const std::string& bag_name,
                                        const std::string& map_file, const std::string& image_topic,
                                        LiveMeasurementSimulatorParams& params) {
  // Note using image features is set in the offline replay tool
  LoadMessageBufferParams("imu", config, params.imu, "or_");
  LoadMessageBufferParams("flight_mode", config, params.flight_mode, "or_");
  LoadMessageBufferParams("depth_odometry", config, params.depth_odometry, "or_");
  LoadMessageBufferParams("of", config, params.of, "or_");
  LoadMessageBufferParams("vl", config, params.vl, "or_");
  LoadMessageBufferParams("ar", config, params.ar, "or_");
  LoadMessageBufferParams("vio", config, params.vio, "or_");
  LOAD_PARAM(params.save_optical_flow_images, config, "or_");
  LOAD_PARAM(params.use_bag_depth_odom_msgs, config, "or_");
  params.bag_name = bag_name;
  params.map_file = map_file;
  params.image_topic = image_topic;
}

void LoadGraphLocalizerSimulatorParams(config_reader::ConfigReader& config, GraphLocalizerSimulatorParams& params) {
  LOAD_PARAM(params.optimization_time, config, "or_loc_");
}

void LoadGraphVIOSimulatorParams(config_reader::ConfigReader& config, GraphVIOSimulatorParams& params) {
  LOAD_PARAM(params.optimization_time, config, "or_vio_");
}

void LoadOfflineReplayParams(config_reader::ConfigReader& config, OfflineReplayParams& params) {
  LOAD_PARAM(params.save_optical_flow_images, config, "or_");
  LOAD_PARAM(params.log_relative_time, config, "or_");
  LOAD_PARAM(params.sparse_mapping_min_num_landmarks, config, "or_");
  LOAD_PARAM(params.ar_min_num_landmarks, config, "or_");
  params.nav_cam_params.reset(new camera::CameraParameters(&config, "nav_cam"));
  params.body_T_nav_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.body_T_dock_cam = lc::LoadTransform(config, "dock_cam_transform");
}
}  // namespace localization_analysis
