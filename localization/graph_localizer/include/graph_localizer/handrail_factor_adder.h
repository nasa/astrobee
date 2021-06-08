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

#ifndef GRAPH_LOCALIZER_HANDRAIL_FACTOR_ADDER_H_
#define GRAPH_LOCALIZER_HANDRAIL_FACTOR_ADDER_H_

#include <graph_localizer/handrail_factor_adder_params.h>
#include <graph_optimizer/factor_adder.h>
#include <localization_measurements/handrail_points_measurement.h>

#include <vector>

namespace graph_localizer {
class HandrailFactorAdder : public graph_optimizer::FactorAdder<localization_measurements::HandrailPointsMeasurement,
                                                                HandrailFactorAdderParams> {
  using Base =
    graph_optimizer::FactorAdder<localization_measurements::HandrailPointsMeasurement, HandrailFactorAdderParams>;

 public:
  HandrailFactorAdder(const HandrailFactorAdderParams& params);

  std::vector<graph_optimizer::FactorsToAdd> AddFactors(
    const localization_measurements::HandrailPointsMeasurement& handrail_points_measurement) final;

  void AddPointToLineFactors(const localization_measurements::HandrailPointsMeasurement& handrail_points_measurement,
                             std::vector<graph_optimizer::FactorsToAdd>& factors_to_add);
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_HANDRAIL_FACTOR_ADDER_H_
