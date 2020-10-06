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
#ifndef GRAPH_BAG_LIVE_MEASUREMENT_SIMULATOR_PARAMS_H_
#define GRAPH_BAG_LIVE_MEASUREMENT_SIMULATOR_PARAMS_H_

#include <graph_bag/message_buffer_params.h>

#include <string>

namespace graph_bag {
struct LiveMeasurementSimulatorParams {
  MessageBufferParams imu;
  MessageBufferParams of;
  MessageBufferParams vl;
  MessageBufferParams ar;
  std::string bag_name;
  std::string map_file;
  std::string image_topic;
};
}  // namespace graph_bag

#endif  // GRAPH_BAG_LIVE_MEASUREMENT_SIMULATOR_PARAMS_H_
