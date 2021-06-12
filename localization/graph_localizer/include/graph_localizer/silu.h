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

#ifndef GRAPH_LOCALIZER_SILU_H_
#define GRAPH_LOCALIZER_SILU_H_

#include <gtsam/base/OptionalJacobian.h>

namespace graph_localizer {
// Sigmoid linear unit function
double Silu(const double x, gtsam::OptionalJacobian<1, 1> d_silu_d_x = boost::none);

double SiluWithOffset(const double x, const double offset, gtsam::OptionalJacobian<1, 1> d_silu_d_x = boost::none);

// Create symmetric silu that maps negative values to positive so the silu creates a two way ramp: \__/ rather
// than a single ramp _/ Note that this is discontinous at x = 0.
double SiluWithOffsetSymmetric(const double x, const double offset, gtsam::OptionalJacobian<1, 1> d_silu_d_x);
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_SILU_H_
