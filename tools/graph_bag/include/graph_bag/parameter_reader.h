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
#ifndef GRAPH_BAG_PARAMETER_READER_H_
#define GRAPH_BAG_PARAMETER_READER_H_

#include <config_reader/config_reader.h>
#include <graph_bag/graph_bag_params.h>
#include <graph_bag/graph_localizer_simulator_params.h>
#include <graph_bag/live_measurement_simulator_params.h>
#include <graph_bag/message_buffer_params.h>

#include <string>

namespace graph_bag {
void LoadMessageBufferParams(config_reader::ConfigReader& config, MessageBufferParams& params);
void LoadLiveMeasurementSimulatorParams(config_reader::ConfigReader& config, const std::string& bag_name,
                                        const std::string& map_file, const std::string& image_topic,
                                        LiveMeasurementSimulatorParams& params);
void LoadGraphLocalizerSimulatorParams(config_reader::ConfigReader& config, GraphLocalizerSimulatorParams& params);
void LoadGraphBagParams(config_reader::ConfigReader& config, GraphBagParams& params);
}  // namespace graph_bag

#endif  // GRAPH_BAG_PARAMETER_READER_H_
