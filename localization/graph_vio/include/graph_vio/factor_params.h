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
#ifndef GRAPH_VIO_FACTOR_PARAMS_H_
#define GRAPH_VIO_FACTOR_PARAMS_H_

#include <factor_adders/projection_factor_adder_params.h>
#include <factor_adders/smart_projection_factor_adder_params.h>
#include <factor_adders/standstill_factor_adder_params.h>

namespace graph_vio {
struct FactorParams {
  factor_adders::SmartProjectionFactorAdderParams smart_projection_adder;
  factor_adders::StandstillFactorAdderParams standstill_adder;
  factor_adders::ProjectionFactorAdderParams projection_adder;
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_FACTOR_PARAMS_H_
