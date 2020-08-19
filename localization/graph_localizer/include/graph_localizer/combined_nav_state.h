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

#ifndef GRAPH_LOCALIZER_COMBINED_NAV_STATE_H_
#define GRAPH_LOCALIZER_COMBINED_NAV_STATE_H_

#include <graph_localizer/time.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

namespace graph_localizer {
struct CombinedNavStateNoise {
  gtsam::SharedNoiseModel pose_noise;
  gtsam::SharedNoiseModel velocity_noise;
  gtsam::SharedNoiseModel bias_noise;
};

class CombinedNavState {
 public:
  CombinedNavState(const gtsam::NavState& nav_state, const gtsam::imuBias::ConstantBias& bias, const Time timestamp);
  CombinedNavState(const gtsam::Pose3& pose, const gtsam::Velocity3& velocity, const gtsam::imuBias::ConstantBias& bias,
                   const Time timestamp);
  Time timestamp() const { return timestamp_; }
  const gtsam::NavState& nav_state() const { return nav_state_; }
  gtsam::Pose3 pose() const { return nav_state().pose(); }
  const gtsam::Velocity3& velocity() const { return nav_state().velocity(); }
  const gtsam::imuBias::ConstantBias& bias() const { return bias_; }

 private:
  Time timestamp_;
  gtsam::NavState nav_state_;
  gtsam::imuBias::ConstantBias bias_;
};

}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_COMBINED_NAV_STATE_H_
