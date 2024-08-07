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
#ifndef LOCALIZATION_ANALYSIS_GRAPH_VIO_SIMULATOR_PARAMS_H_
#define LOCALIZATION_ANALYSIS_GRAPH_VIO_SIMULATOR_PARAMS_H_

namespace localization_analysis {
struct GraphVIOSimulatorParams {
  // Simulated optimization time. Messages that are received during optimization
  // are added once this duration has passed.
  double optimization_time;
};
}  // namespace localization_analysis

#endif  // LOCALIZATION_ANALYSIS_GRAPH_VIO_SIMULATOR_PARAMS_H_
