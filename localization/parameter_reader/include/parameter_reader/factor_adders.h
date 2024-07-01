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
#ifndef PARAMETER_READER_FACTOR_ADDERS_H_
#define PARAMETER_READER_FACTOR_ADDERS_H_

#include <config_reader/config_reader.h>
#include <factor_adders/factor_adder_params.h>
#include <factor_adders/loc_factor_adder_params.h>
#include <factor_adders/relative_pose_factor_adder_params.h>
#include <factor_adders/depth_odometry_factor_adder_params.h>
#include <factor_adders/standstill_factor_adder_params.h>
#include <factor_adders/vo_smart_projection_factor_adder_params.h>

#include <gtsam/slam/SmartFactorParams.h>

#include <string>

namespace parameter_reader {
void LoadFactorAdderParams(config_reader::ConfigReader& config, factor_adders::FactorAdderParams& params,
                           const std::string& prefix = "");

void LoadLocFactorAdderParams(config_reader::ConfigReader& config, factor_adders::LocFactorAdderParams& params,
                              const std::string& prefix = "", const std::string& camera_name = "nav");

void LoadDepthOdometryFactorAdderParams(config_reader::ConfigReader& config,
                                        factor_adders::DepthOdometryFactorAdderParams& params,
                                        const std::string& prefix = "", const std::string& camera_name = "haz");

void LoadRelativePoseFactorAdderParams(config_reader::ConfigReader& config,
                                       factor_adders::RelativePoseFactorAdderParams& params,
                                       const std::string& prefix = "");

void LoadStandstillFactorAdderParams(config_reader::ConfigReader& config,
                                     factor_adders::StandstillFactorAdderParams& params,
                                     const std::string& prefix = "");

void LoadVoSmartProjectionFactorAdderParams(config_reader::ConfigReader& config,
                                            factor_adders::VoSmartProjectionFactorAdderParams& params,
                                            const std::string& prefix = "");

void LoadSmartProjectionParams(config_reader::ConfigReader& config, gtsam::SmartProjectionParams& params,
                               const std::string& prefix = "");
}  // namespace parameter_reader

#endif  // PARAMETER_READER_FACTOR_ADDERS_H_
