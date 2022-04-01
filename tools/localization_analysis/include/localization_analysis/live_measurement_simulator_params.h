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
#ifndef LOCALIZATION_ANALYSIS_LIVE_MEASUREMENT_SIMULATOR_PARAMS_H_
#define LOCALIZATION_ANALYSIS_LIVE_MEASUREMENT_SIMULATOR_PARAMS_H_

#include <localization_analysis/message_buffer_params.h>

#include <string>

namespace localization_analysis {
struct LiveMeasurementSimulatorParams {
  MessageBufferParams imu;
  MessageBufferParams flight_mode;
  MessageBufferParams depth_odometry;
  MessageBufferParams of;
  MessageBufferParams vl;
  MessageBufferParams ar;
  MessageBufferParams img;
  std::string bag_name;
  std::string map_file;
  std::string image_topic;
  bool use_image_features;
  bool save_optical_flow_images;
};
}  // namespace localization_analysis

#endif  // LOCALIZATION_ANALYSIS_LIVE_MEASUREMENT_SIMULATOR_PARAMS_H_
