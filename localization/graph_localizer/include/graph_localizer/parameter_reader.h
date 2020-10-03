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
#ifndef GRAPH_LOCALIZER_PARAMETER_READER_H_
#define GRAPH_LOCALIZER_PARAMETER_READER_H_

#include <config_reader/config_reader.h>
#include <graph_localizer/calibration_params.h>
#include <graph_localizer/factor_params.h>
#include <graph_localizer/feature_tracker_params.h>
#include <graph_localizer/graph_initialization_params.h>
#include <graph_localizer/graph_localizer_params.h>
#include <graph_localizer/graph_values_params.h>
#include <graph_localizer/noise_params.h>
#include <graph_localizer/sanity_checker_params.h>

namespace graph_localizer {
void LoadCalibrationParams(config_reader::ConfigReader& config, CalibrationParams& params);
void LoadFactorParams(config_reader::ConfigReader& config, FactorParams& params);
void LoadFeatureTrackerParams(config_reader::ConfigReader& config, FeatureTrackerParams& params);
void LoadGraphValuesParams(config_reader::ConfigReader& config, GraphValuesParams& params);
void LoadImuIntegrationParams(config_reader::ConfigReader& config, GraphInitializationParams& params);
void LoadNoiseParams(config_reader::ConfigReader& config, NoiseParams& params);
void LoadSanityCheckerParams(config_reader::ConfigReader& config, SanityCheckerParams& params);
// Loads all params except some in graph_initialization_params (biases and start pose) that are
// not loaded from config files
void LoadGraphLocalizerParams(config_reader::ConfigReader& config, GraphLocalizerParams& params);
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_PARAMETER_READER_H_
