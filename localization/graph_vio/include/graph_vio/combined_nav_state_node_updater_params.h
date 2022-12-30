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
#ifndef GRAPH_VIO_COMBINED_NAV_STATE_NODE_UPDATER_PARAMS_H_
#define GRAPH_VIO_COMBINED_NAV_STATE_NODE_UPDATER_PARAMS_H_

#include <graph_vio/combined_nav_state_graph_values_params.h>
#include <localization_common/combined_nav_state.h>

#include <boost/serialization/serialization.hpp>

namespace graph_vio {
struct CombinedNavStateNodeUpdaterParams {
  double starting_prior_translation_stddev;
  double starting_prior_quaternion_stddev;
  double starting_prior_velocity_stddev;
  double starting_prior_accel_bias_stddev;
  double starting_prior_gyro_bias_stddev;
  double huber_k;
  localization_common::CombinedNavState global_N_body_start;
  bool add_priors;
  CombinedNavStateGraphValuesParams graph_values;
  bool threshold_bias_uncertainty;
  double accel_bias_stddev_threshold;
  double gyro_bias_stddev_threshold;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(starting_prior_translation_stddev);
    ar& BOOST_SERIALIZATION_NVP(starting_prior_quaternion_stddev);
    ar& BOOST_SERIALIZATION_NVP(starting_prior_velocity_stddev);
    ar& BOOST_SERIALIZATION_NVP(starting_prior_accel_bias_stddev);
    ar& BOOST_SERIALIZATION_NVP(starting_prior_gyro_bias_stddev);
    ar& BOOST_SERIALIZATION_NVP(huber_k);
    ar& BOOST_SERIALIZATION_NVP(global_N_body_start);
    ar& BOOST_SERIALIZATION_NVP(add_priors);
    ar& BOOST_SERIALIZATION_NVP(graph_values);
    ar& BOOST_SERIALIZATION_NVP(threshold_bias_uncertainty);
    ar& BOOST_SERIALIZATION_NVP(accel_bias_stddev_threshold);
    ar& BOOST_SERIALIZATION_NVP(gyro_bias_stddev_threshold);
  }
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_COMBINED_NAV_STATE_NODE_UPDATER_PARAMS_H_
