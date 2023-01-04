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

#include <graph_vio/smart_projection_graph_action_completer.h>
#include <graph_vio/utilities.h>
#include <graph_optimizer/utilities.h>
#include <localization_common/logger.h>

#include <gtsam/base/Vector.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

namespace graph_vio {
namespace fa = factor_adders;
namespace go = graph_optimizer;
namespace gv = graph_values;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
SmartProjectionGraphActionCompleter::SmartProjectionGraphActionCompleter(
  const fa::SmartProjectionFactorAdderParams& params,
  std::shared_ptr<const gv::CombinedNavStateGraphValues> graph_values)
    : params_(params), graph_values_(std::move(graph_values)) {
  smart_projection_params_.verboseCheirality = params_.verbose_cheirality;
  smart_projection_params_.setRankTolerance(1e-9);
  smart_projection_params_.setLandmarkDistanceThreshold(params_.landmark_distance_threshold);
  smart_projection_params_.setDynamicOutlierRejectionThreshold(params_.dynamic_outlier_rejection_threshold);
  smart_projection_params_.setRetriangulationThreshold(params_.retriangulation_threshold);
  smart_projection_params_.setEnableEPI(params_.enable_EPI);
}

go::GraphActionCompleterType SmartProjectionGraphActionCompleter::type() const {
  return go::GraphActionCompleterType::SmartFactor;
}

bool SmartProjectionGraphActionCompleter::DoAction(go::FactorsToAdd& factors_to_add,
                                                   gtsam::NonlinearFactorGraph& graph_factors) {
  go::DeleteFactors<RobustSmartFactor>(graph_factors);
  if (params_.splitting) SplitSmartFactorsIfNeeded(*graph_values_, factors_to_add);
  return true;
}

// TODO(rsoussan): Clean this function up (duplicate code), address other todo's
void SmartProjectionGraphActionCompleter::SplitSmartFactorsIfNeeded(const gv::CombinedNavStateGraphValues& graph_values,
                                                                    go::FactorsToAdd& factors_to_add) {
  for (auto& factor_to_add : factors_to_add.Get()) {
    auto smart_factor = dynamic_cast<RobustSmartFactor*>(factor_to_add.factor.get());
    if (!smart_factor) continue;
    // Can't remove measurements if there are only 2 or fewer
    if (smart_factor->measured().size() <= 2) continue;
    const auto point = smart_factor->triangulateSafe(smart_factor->cameras(graph_values.values()));
    if (point.valid()) continue;
    {
      const auto fixed_smart_factor =
        FixSmartFactorByRemovingIndividualMeasurements(params_, *smart_factor, smart_projection_params_, graph_values);
      if (fixed_smart_factor) {
        factor_to_add.factor = *fixed_smart_factor;
        continue;
      }
    }
    {
      const auto fixed_smart_factor =
        FixSmartFactorByRemovingMeasurementSequence(params_, *smart_factor, smart_projection_params_, graph_values);
      if (fixed_smart_factor) {
        factor_to_add.factor = *fixed_smart_factor;
        continue;
      }
    }
    LogDebug("SplitSmartFactorsIfNeeded: Failed to fix smart factor");
  }
}
}  // namespace graph_vio
