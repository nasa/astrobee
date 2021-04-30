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
#include <graph_localizer/graph_initializer_params.h>
#include <graph_localizer/graph_localizer_nodelet_params.h>
#include <graph_localizer/graph_localizer_params.h>
#include <graph_localizer/graph_optimizer_params.h>
#include <graph_localizer/graph_values_params.h>
#include <graph_localizer/noise_params.h>
#include <graph_localizer/loc_factor_adder_params.h>
#include <graph_localizer/projection_factor_adder_params.h>
#include <graph_localizer/rotation_factor_adder_params.h>
#include <graph_localizer/sanity_checker_params.h>
#include <graph_localizer/smart_projection_factor_adder_params.h>
#include <graph_localizer/standstill_factor_adder_params.h>

namespace graph_localizer {
void LoadCalibrationParams(config_reader::ConfigReader& config, CalibrationParams& params);
void LoadFactorParams(config_reader::ConfigReader& config, FactorParams& params);
void LoadLocFactorAdderParams(config_reader::ConfigReader& config, LocFactorAdderParams& params);
void LoadARTagLocFactorAdderParams(config_reader::ConfigReader& config, LocFactorAdderParams& params);
void LoadProjectionFactorAdderParams(config_reader::ConfigReader& config, ProjectionFactorAdderParams& params);
void LoadRotationFactorAdderParams(config_reader::ConfigReader& config, RotationFactorAdderParams& params);
void LoadSmartProjectionFactorAdderParams(config_reader::ConfigReader& config,
                                          SmartProjectionFactorAdderParams& params);
void LoadStandstillFactorAdderParams(config_reader::ConfigReader& config, StandstillFactorAdderParams& params);
void LoadFeatureTrackerParams(config_reader::ConfigReader& config, FeatureTrackerParams& params);
void LoadGraphValuesParams(config_reader::ConfigReader& config, GraphValuesParams& params);
void LoadImuIntegrationParams(config_reader::ConfigReader& config, GraphInitializerParams& params);
void LoadNoiseParams(config_reader::ConfigReader& config, NoiseParams& params);
void LoadSanityCheckerParams(config_reader::ConfigReader& config, SanityCheckerParams& params);
// Loads all params except some (biases and start pose) that are
// not loaded from config files
void LoadGraphInitializerParams(config_reader::ConfigReader& config, GraphInitializerParams& params);
void LoadGraphLocalizerParams(config_reader::ConfigReader& config, GraphLocalizerParams& params);
void LoadGraphOptimizerParams(config_reader::ConfigReader& config, GraphOptimizerParams& params);
void LoadGraphLocalizerNodeletParams(config_reader::ConfigReader& config, GraphLocalizerNodeletParams& params);
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_PARAMETER_READER_H_
