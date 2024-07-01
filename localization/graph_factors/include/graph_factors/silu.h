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

#ifndef GRAPH_FACTORS_SILU_H_
#define GRAPH_FACTORS_SILU_H_

#include <gtsam/base/OptionalJacobian.h>

namespace graph_factors {
// Sigmoid linear unit function
double Silu(const double x, gtsam::OptionalJacobian<1, 1> d_silu_d_x = boost::none);

double SiluWithOffset(const double x, const double offset, gtsam::OptionalJacobian<1, 1> d_silu_d_x = boost::none);

// Create silu that creates a two way ramp:  __/
//                                          /
// centered at x = 0 rather than a single ramp _/ Note that this is discontinous at x = 0.
double SiluWithOffsetTwoWay(const double x, const double offset,
                            gtsam::OptionalJacobian<1, 1> d_silu_d_x = boost::none);
}  // namespace graph_factors

#endif  // GRAPH_FACTORS_SILU_H_
