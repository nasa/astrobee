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

#ifndef GRAPH_LOCALIZER_GRAPH_LOCALIZER_PARAMS_H_
#define GRAPH_LOCALIZER_GRAPH_LOCALIZER_PARAMS_H_

#include <factor_adders/loc_factor_adder_params.h>
#include <node_adders/pose_node_adder_params.h>
#include <node_adders/timestamped_node_adder_model_params.h>
#include <optimizers/nonlinear_optimizer.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer_params.h>

#include <boost/serialization/serialization.hpp>

namespace graph_localizer {
struct GraphLocalizerParams {
  factor_adders::LocFactorAdderParams ar_tag_loc_factor_adder;
  factor_adders::LocFactorAdderParams sparse_map_loc_factor_adder;
  node_adders::PoseNodeAdderParams pose_node_adder;
  node_adders::TimestampedNodeAdderModelParams pose_node_adder_model;
  optimizers::NonlinearOptimizerParams nonlinear_optimizer;
  sliding_window_graph_optimizer::SlidingWindowGraphOptimizerParams sliding_window_graph_optimizer;
  // Max gap between vio measurements. If this is exceeded, graph localizer is reset.
  double max_vio_measurement_gap;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(ar_tag_loc_factor_adder);
    ar& BOOST_SERIALIZATION_NVP(sparse_map_loc_factor_adder);
    ar& BOOST_SERIALIZATION_NVP(pose_node_adder);
    ar& BOOST_SERIALIZATION_NVP(pose_node_adder_model);
    ar& BOOST_SERIALIZATION_NVP(nonlinear_optimizer);
    ar& BOOST_SERIALIZATION_NVP(sliding_window_graph_optimizer);
    ar& BOOST_SERIALIZATION_NVP(max_vio_measurement_gap);
  }
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_PARAMS_H_
