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
#ifndef NODE_ADDERS_COMBINED_NAV_STATE_NODE_ADDER_MODEL_PARAMS_H_
#define NODE_ADDERS_COMBINED_NAV_STATE_NODE_ADDER_MODEL_PARAMS_H_

#include <node_adders/timestamped_node_adder_model_params.h>

#include <imu_integration/imu_integrator_params.h>

#include <boost/serialization/serialization.hpp>

namespace node_adders {
struct CombinedNavStateNodeAdderModelParams : public TimestampedNodeAdderModelParams {
  imu_integration::ImuIntegratorParams imu_integrator;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TimestampedNodeAdderModelParams);
    ar& BOOST_SERIALIZATION_NVP(imu_integrator);
  }
};
}  // namespace node_adders

#endif  // NODE_ADDERS_COMBINED_NAV_STATE_NODE_ADDER_MODEL_PARAMS_H_
