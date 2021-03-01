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

#ifndef IMU_INTEGRATION_BUTTERO1_H_
#define IMU_INTEGRATION_BUTTERO1_H_

#include <imu_integration/filter.h>

#include <array>

namespace imu_integration {
template <class Params>
class ButterO1 : public Filter {
 public:
  ButterO1() : initialized_(false) {}
  // Returns filtered value and timestamp
  double AddValue(const double value) final {
    if (!initialized_) Initialize(value, Params::kGain);

    // Shift input vals
    xv_[0] = xv_[1];
    xv_[1] = xv_[2];
    xv_[2] = xv_[3];
    // Add new value
    xv_[3] = value / Params::kGain;
    // Shift output vals
    yv_[0] = yv_[1];
    yv_[1] = yv_[2];
    yv_[2] = yv_[3];
    // Generate new output
    yv_[3] = (xv_[0] + xv_[3]) + Params::kX12 * (xv_[1] + xv_[2]) + (Params::kY2 * yv_[2]);
    // Return most recent output
    return yv_[3];
  }

 private:
  void Initialize(const double first_value, const double gain) {
    for (auto& val : xv_) {
      val = first_value / gain;
    }
    for (auto& val : yv_) {
      val = first_value;
    }

    initialized_ = true;
  }

  // Notation taken from mkfilter site
  // /www/usr/fisher/helpers/mkfilter
  std::array<double, 4> xv_;
  std::array<double, 4> yv_;
  bool initialized_;
};

// 62.5 Hz
struct ParamsButterO1S62_5Lp3N29_16 {
  static constexpr double kGain = 2.999100930e+01;
  static constexpr double kX12 = 2.9560160746;
  static constexpr double kY2 = 0.7361865327;
};
using ButterO1S62_5Lp3N29_16 = ButterO1<ParamsButterO1S62_5Lp3N29_16>;

struct ParamsButterO1S62_5Lp3N20_83 {
  static constexpr double kGain = 2.2743339590e+01;
  static constexpr double kX12 = 1.9999996372;
  static constexpr double kY2 = 0.7361865327;
};
using ButterO1S62_5Lp3N20_83 = ButterO1<ParamsButterO1S62_5Lp3N20_83>;

struct ParamsButterO1S62_5Lp3N15_83 {
  static constexpr double kGain = 1.5479761468e+01;
  static constexpr double kX12 = 1.0418847728;
  static constexpr double kY2 = 0.7361865327;
};
using ButterO1S62_5Lp3N15_83 = ButterO1<ParamsButterO1S62_5Lp3N15_83>;

// 125Hz
// 2500 rpm: 41.666Hz
struct ParamsButterO1S125Lp3N41_66 {
  static constexpr double kGain = 4.270814376e+01;
  static constexpr double kX12 = 1.9996372182;
  static constexpr double kY2 = 0.8595285604;
};
using ButterO1S125Lp3N41_66 = ButterO1<ParamsButterO1S125Lp3N41_66>;

// 2000 rpm: 33.333Hz
struct ParamsButterO1S125Lp3N33_33 {
  static constexpr double kGain = 3.144018095e+01;
  static constexpr double kX12 = 1.2082237395;
  static constexpr double kY2 = 0.8595285604;
};
using ButterO1S125Lp3N33_33 = ButterO1<ParamsButterO1S125Lp3N33_33>;

// 2800 rpm: 46.666Hz
struct ParamsButterO1S125Lp3N46_66{
  static constexpr double kGain = 4.839840415e+01;
  static constexpr double kX12 = 2.3992967530;
  static constexpr double kY2 = 0.8595285604;
};
using ButterO1S125Lp3N46_66 = ButterO1<ParamsButterO1S125Lp3N46_66>;



}  // namespace imu_integration

#endif  // IMU_INTEGRATION_BUTTERO1_H_
