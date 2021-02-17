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

#ifndef IMU_INTEGRATION_BUTTERO5_H_
#define IMU_INTEGRATION_BUTTERO5_H_

#include <imu_integration/filter.h>

#include <localization_common/logger.h>

#include <array>

namespace imu_integration {
template <class Params>
class ButterO5 : public Filter {
 public:
  ButterO5() : initialized_(false) {}
  // Returns filtered value and timestamp
  double AddValue(const double value) final {
    if (!initialized_) Initialize(value, Params::kGain);

    const int last_index = xv_.size() - 1;
    // Shift input and output vals
    for (int i = 0; i < last_index; ++i) {
      xv_[i] = xv_[i + 1];
      yv_[i] = yv_[i + 1];
    }
    // Add new values
    xv_[last_index] = value / Params::kGain;
    // Generate new output
    yv_[last_index] = (xv_[0] + xv_[7]) + Params::kX16 * (xv_[1] + xv_[6]) + Params::kX25 * (xv_[2] + xv_[5]) +
                      Params::kX34 * (xv_[3] + xv_[4]) + (Params::kY2 * yv_[2]) + (Params::kY3 * yv_[3]) +
                      (Params::kY4 * yv_[4]) + (Params::kY5 * yv_[5]) + (Params::kY6 * yv_[6]);

    // Return most recent output
    return yv_[last_index];
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
  std::array<double, 8> xv_;
  std::array<double, 8> yv_;
  bool initialized_;
};

struct ParamsButterO5S62_5Lp3N29_16 {
  static constexpr double kGain = 7.974174280e+04;
  static constexpr double kX16 = 6.9560160746;
  static constexpr double kX25 = 20.7800803730;
  static constexpr double kX34 = 34.5601607460;
  static constexpr double kY2 = 0.3750929134;
  static constexpr double kY3 = -2.2411521809;
  static constexpr double kY4 = 5.3982113474;
  static constexpr double kY5 = -6.5588143289;
  static constexpr double kY6 = 4.0250747177;
};
using ButterO5S62_5Lp3N29_16 = ButterO5<ParamsButterO5S62_5Lp3N29_16>;

struct ParamsButterO5S62_5Lp1N29_16 {
  static constexpr double kGain = 1.444638217e+07;
  static constexpr double kX16 = 6.9560160746;
  static constexpr double kX25 = 20.7800803730;
  static constexpr double kX34 = 34.5601607460;
  static constexpr double kY2 = 0.7221701429;
  static constexpr double kY3 = -3.8457619644;
  static constexpr double kY4 = 8.2000057707;
  static constexpr double kY5 = -8.7511375257;
  static constexpr double kY6 = 4.6747148135;
};
using ButterO5S62_5Lp1N29_16 = ButterO5<ParamsButterO5S62_5Lp1N29_16>;

struct ParamsButterO5S62_5Lp0_5N29_16 {
  static constexpr double kGain = 4.274918013e+08;
  static constexpr double kX16 = 6.9560160746;
  static constexpr double kX25 = 20.7800803730;
  static constexpr double kX34 = 34.5601607460;
  static constexpr double kY2 = 0.8498599655;
  static constexpr double kY3 = -4.3875359464;
  static constexpr double kY4 = 9.0628533836;
  static constexpr double kY5 = -9.3625201736;
  static constexpr double kY6 = 4.8373424748;
};
using ButterO5S62_5Lp0_5N29_16 = ButterO5<ParamsButterO5S62_5Lp0_5N29_16>;
}  // namespace imu_integration

#endif  // IMU_INTEGRATION_BUTTERO5_H_
