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

#ifndef GRAPH_LOCALIZER_SEMANTIC_LOC_FACTOR_ADDER_H_
#define GRAPH_LOCALIZER_SEMANTIC_LOC_FACTOR_ADDER_H_

#include <graph_optimizer/factor_adder.h>
#include <graph_localizer/semantic_loc_factor_adder_params.h>
#include <localization_common/averager.h>
#include <localization_common/combined_nav_state.h>
#include <localization_measurements/matched_projections_measurement.h>
#include <localization_measurements/semantic_dets_measurement.h>

#include <vector>


namespace graph_localizer {
class GraphLocalizer;

class SemanticLocFactorAdder : public graph_optimizer::FactorAdder<localization_measurements::SemanticDetsMeasurement,
                                                                   SemanticLocFactorAdderParams> {
 using Base =
  graph_optimizer::FactorAdder<localization_measurements::SemanticDetsMeasurement, SemanticLocFactorAdderParams>;
 public:
  SemanticLocFactorAdder(const SemanticLocFactorAdderParams& params,
                         const graph_optimizer::GraphActionCompleterType graph_action_completer_type);

  std::vector<graph_optimizer::FactorsToAdd> AddFactors(
    const localization_measurements::SemanticDetsMeasurement& semantic_dets);

  void SetCombinedNavState(const boost::optional<localization_common::CombinedNavState>& state);
    
  typedef struct SemanticMatch {
    int cls;
    bool have_map_point;
    Eigen::Vector2d map_point_px;
    bool have_matched_det;
    Eigen::Vector2d det_center_px;
    Eigen::Vector2d det_bbox_size_px;

    SemanticMatch(int c, Eigen::Vector2d mpp, Eigen::Vector2d dcp, Eigen::Vector2d dbsp) :
     cls(c), have_map_point(true), map_point_px(mpp), have_matched_det(true), det_center_px(dcp), det_bbox_size_px(dbsp) {} 

    SemanticMatch(int c, Eigen::Vector2d mpp) :
     cls(c), have_map_point(true), map_point_px(mpp), have_matched_det(false) {} 

    SemanticMatch(int c, Eigen::Vector2d dcp, Eigen::Vector2d dbsp) :
     cls(c), have_map_point(false), have_matched_det(true), det_center_px(dcp), det_bbox_size_px(dbsp) {} 
  } SemanticMatch;
  const std::vector<SemanticMatch>& last_matches() const { return last_matches_; }

 private:
  std::map<int, std::vector<Eigen::Isometry3d>> object_poses_; // map from class to vector of positions
  std::vector<SemanticMatch> last_matches_;

  void ComputeFactorsToAdd(std::vector<graph_optimizer::FactorsToAdd> &factors_to_add,
                           localization_measurements::MatchedProjectionsMeasurement &measurement);
  graph_optimizer::GraphActionCompleterType type() const;

  graph_optimizer::GraphActionCompleterType graph_action_completer_type_;
  localization_common::CombinedNavState last_combined_nav_state_;
  
  localization_common::Averager num_semantic_objects_averager_ = localization_common::Averager("Num Semantic Objects");
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_SEMANTIC_LOC_FACTOR_ADDER_H_
