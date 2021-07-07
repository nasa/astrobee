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

#ifndef GRAPH_LOCALIZER_SEMANTIC_FLOW_FACTOR_ADDER_H_
#define GRAPH_LOCALIZER_SEMANTIC_FLOW_FACTOR_ADDER_H_

#include <graph_localizer/semantic_object_tracker.h>
#include <graph_localizer/semantic_flow_factor_adder_params.h>
#include <graph_optimizer/cumulative_factor_adder.h>

#include <gtsam/slam/SmartFactorParams.h>

namespace graph_localizer {
class SemanticFlowFactorAdder : public graph_optimizer::CumulativeFactorAdder<SemanticFlowFactorAdderParams> {
  using Base = graph_optimizer::CumulativeFactorAdder<SemanticFlowFactorAdderParams>;
  public:
    SemanticFlowFactorAdder(const SemanticFlowFactorAdderParams& params,
                        std::shared_ptr<const SemanticObjectTracker> object_tracker);

    std::vector<graph_optimizer::FactorsToAdd> AddFactors() final;

  private:
    std::shared_ptr<const SemanticObjectTracker> object_tracker_;
    gtsam::SmartProjectionParams smart_projection_params_;
};
} // namespace graph_localizer

#endif // GRAPH_LOCALIZER_SEMANTIC_FLOW_FACTOR_ADDER_H_
