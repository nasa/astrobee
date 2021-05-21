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

#include <graph_bag/parameter_reader.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

namespace graph_bag {
namespace lc = localization_common;
namespace mc = msg_conversions;

void LoadMessageBufferParams(const std::string& message_type, config_reader::ConfigReader& config,
                             MessageBufferParams& params) {
  params.msg_delay = mc::LoadDouble(config, message_type + "_msg_delay");
  params.min_msg_spacing = mc::LoadDouble(config, message_type + "_min_msg_spacing");
}

void LoadLiveMeasurementSimulatorParams(config_reader::ConfigReader& config, const std::string& bag_name,
                                        const std::string& map_file, const std::string& image_topic,
                                        LiveMeasurementSimulatorParams& params) {
  LoadMessageBufferParams("imu", config, params.imu);
  LoadMessageBufferParams("flight_mode", config, params.flight_mode);
  LoadMessageBufferParams("of", config, params.of);
  LoadMessageBufferParams("vl", config, params.vl);
  LoadMessageBufferParams("ar", config, params.ar);
  params.save_optical_flow_images = mc::LoadBool(config, "save_optical_flow_images");
  params.bag_name = bag_name;
  params.map_file = map_file;
  params.image_topic = image_topic;
}

void LoadGraphLocalizerSimulatorParams(config_reader::ConfigReader& config, GraphLocalizerSimulatorParams& params) {
  params.optimization_time = mc::LoadDouble(config, "optimization_time");
}

void LoadGraphBagParams(config_reader::ConfigReader& config, GraphBagParams& params) {
  params.save_optical_flow_images = mc::LoadBool(config, "save_optical_flow_images");
  params.nav_cam_params.reset(new camera::CameraParameters(&config, "nav_cam"));
  params.body_T_nav_cam = lc::LoadTransform(config, "nav_cam_transform");
  params.sparse_mapping_min_num_landmarks = mc::LoadInt(config, "loc_adder_min_num_matches");
  params.ar_min_num_landmarks = mc::LoadInt(config, "ar_tag_loc_adder_min_num_matches");
}
}  // namespace graph_bag
