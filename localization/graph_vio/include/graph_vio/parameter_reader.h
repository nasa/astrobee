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
#ifndef GRAPH_VIO_PARAMETER_READER_H_
#define GRAPH_VIO_PARAMETER_READER_H_

#include <config_reader/config_reader.h>
#include <graph_vio/calibration_params.h>
#include <graph_vio/combined_nav_state_graph_values_params.h>
#include <graph_vio/combined_nav_state_node_updater_params.h>
#include <graph_vio/factor_params.h>
#include <graph_vio/feature_point_node_updater_params.h>
#include <graph_vio/feature_tracker_params.h>
#include <graph_vio/graph_initializer_params.h>
#include <graph_vio/graph_vio_nodelet_params.h>
#include <graph_vio/graph_vio_params.h>
#include <graph_vio/projection_factor_adder_params.h>
#include <graph_vio/sanity_checker_params.h>
#include <graph_vio/smart_projection_factor_adder_params.h>
#include <graph_vio/standstill_factor_adder_params.h>

namespace graph_vio {
void LoadCalibrationParams(config_reader::ConfigReader& config, CalibrationParams& params);
void LoadFactorParams(config_reader::ConfigReader& config, FactorParams& params);
void LoadProjectionFactorAdderParams(config_reader::ConfigReader& config, ProjectionFactorAdderParams& params);
void LoadSmartProjectionFactorAdderParams(config_reader::ConfigReader& config,
                                          SmartProjectionFactorAdderParams& params);
void LoadStandstillFactorAdderParams(config_reader::ConfigReader& config, StandstillFactorAdderParams& params);
void LoadFeatureTrackerParams(config_reader::ConfigReader& config, FeatureTrackerParams& params);
void LoadImuIntegrationParams(config_reader::ConfigReader& config, GraphInitializerParams& params);
void LoadSanityCheckerParams(config_reader::ConfigReader& config, SanityCheckerParams& params);
// Loads all params except some (biases and start pose) that are
// not loaded from config files
void LoadGraphInitializerParams(config_reader::ConfigReader& config, GraphInitializerParams& params);
void LoadCombinedNavStateGraphValuesParams(config_reader::ConfigReader& config,
                                           CombinedNavStateGraphValuesParams& params);
void LoadCombinedNavStateNodeUpdaterParams(config_reader::ConfigReader& config,
                                           CombinedNavStateNodeUpdaterParams& params);
void LoadFeaturePointNodeUpdaterParams(config_reader::ConfigReader& config, FeaturePointNodeUpdaterParams& params);
void LoadGraphVIOParams(config_reader::ConfigReader& config, GraphVIOParams& params);
void LoadGraphVIONodeletParams(config_reader::ConfigReader& config, GraphVIONodeletParams& params);
}  // namespace graph_vio

#endif  // GRAPH_VIO_PARAMETER_READER_H_
