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

#include <graph_localizer/combined_nav_state.h>

namespace graph_localizer {
CombinedNavState::CombinedNavState(const gtsam::NavState& nav_state, const gtsam::imuBias::ConstantBias& bias,
                                   const Time timestamp)
    : nav_state_(nav_state), bias_(bias), timestamp_(timestamp) {}

CombinedNavState::CombinedNavState(const gtsam::Pose3& pose, const gtsam::Velocity3& velocity,
                                   const gtsam::imuBias::ConstantBias& bias, const Time timestamp)
    : CombinedNavState(gtsam::NavState(pose, velocity), bias, timestamp) {}
}  // namespace graph_localizer
