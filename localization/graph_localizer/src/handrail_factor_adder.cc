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

#include <graph_localizer/handrail_factor_adder.h>
#include <graph_localizer/utilities.h>
#include <localization_common/logger.h>

#include <gtsam/inference/Symbol.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
HandrailFactorAdder::HandrailFactorAdder(const HandrailFactorAdderParams& params) : HandrailFactorAdder::Base(params) {}

std::vector<go::FactorsToAdd> HandrailFactorAdder::AddFactors(const lm::HandrailPointsMeasurement& measurement) {
  go::FactorsToAdd handrail_factors_to_add;
  return {handrail_factors_to_add};
}
}  // namespace graph_localizer
