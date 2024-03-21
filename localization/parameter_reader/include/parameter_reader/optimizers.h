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
#ifndef PARAMETER_READER_OPTIMIZERS_H_
#define PARAMETER_READER_OPTIMIZERS_H_

#include <config_reader/config_reader.h>
#include <optimizers/isam2_optimizer_params.h>
#include <optimizers/nonlinear_optimizer_params.h>
#include <optimizers/optimizer_params.h>

#include <string>

namespace parameter_reader {
void LoadOptimizerParams(config_reader::ConfigReader& config, optimizers::OptimizerParams& params,
                         const std::string& prefix = "");

void LoadISAM2OptimizerParams(config_reader::ConfigReader& config, optimizers::ISAM2OptimizerParams& params,
                              const std::string& prefix = "");

void LoadNonlinearOptimizerParams(config_reader::ConfigReader& config, optimizers::NonlinearOptimizerParams& params,
                                  const std::string& prefix = "");
}  // namespace parameter_reader

#endif  // PARAMETER_READER_OPTIMIZERS_H_
