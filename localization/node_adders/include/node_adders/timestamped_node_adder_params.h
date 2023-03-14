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
#ifndef NODE_ADDERS_TIMESTAMPED_NODE_ADDER_PARAMS_H_
#define NODE_ADDERS_TIMESTAMPED_NODE_ADDER_PARAMS_H_

#include <localization_common/time.h>

#include <boost/serialization/serialization.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <vector>

namespace node_adders {
template <typename NodeType>
struct TimestampedNodeAdderParams {
  NodeType start_node;
  std::vector<gtsam::SharedNoiseModel> start_noise_models;
  double huber_k;
  bool add_priors;
  localization_common::Time starting_time;
  // Only kept if there are at least min_num_states and not more than max_num_states
  double ideal_duration;
  // TODO(rsoussan): Rename states to nodes?
  int min_num_states;
  int max_num_states;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(start_node);
    ar& BOOST_SERIALIZATION_NVP(start_noise_models);
    ar& BOOST_SERIALIZATION_NVP(huber_k);
    ar& BOOST_SERIALIZATION_NVP(add_priors);
    ar& BOOST_SERIALIZATION_NVP(starting_time);
    ar& BOOST_SERIALIZATION_NVP(ideal_duration);
    ar& BOOST_SERIALIZATION_NVP(min_num_states);
    ar& BOOST_SERIALIZATION_NVP(max_num_states);
  }
};
}  // namespace node_adders

#endif  // NODE_ADDERS_TIMESTAMPED_NODE_ADDER_PARAMS_H_
