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

#ifndef GRAPH_LOCALIZER_HUNGARIAN_ASSIGNER_H_
#define GRAPH_LOCALIZER_HUNGARIAN_ASSIGNER_H_

#include <graph_localizer/semantic_loc_factor_adder_params.h>
#include <localization_measurements/semantic_dets_measurement.h>
#include <localization_common/utilities.h>
#include <map>

namespace graph_localizer {
class HungarianAssigner {
public:
  HungarianAssigner(const SemanticLocFactorAdderParams& params);

  typedef std::pair<size_t, std::shared_ptr<const Eigen::Isometry3d>> Assignment;
  typedef std::vector<Assignment> AssignmentSet;
  AssignmentSet assign(const Eigen::Isometry3d& world_T_body, const localization_measurements::SemanticDets &dets);
private:
  const SemanticLocFactorAdderParams params_;
  // map from class to vector of positions, map for easy lookup
  std::map<int, std::vector<std::shared_ptr<Eigen::Isometry3d>>> object_poses_; 

  typedef std::pair<size_t, size_t> AssignmentInd;
  typedef std::vector<AssignmentInd> AssignmentIndSet;
  AssignmentIndSet tryAssignment(const Eigen::ArrayXXd& cost_matrix, Eigen::ArrayXXi &cell_state, int num_actual_rows);
  AssignmentIndSet solve(const Eigen::ArrayXXd& cost_matrix_in);
};
} // namespace graph_localizer

#endif // GRAPH_LOCALIZER_HUNGARIAN_ASSIGNER_H_
