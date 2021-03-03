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

struct ParamsButterO5S62_5Lp3N20_83 {
  static constexpr double kGain = 6.0459547194e+04;
  static constexpr double kX16 = 5.9994195281e+00;
  static constexpr double kX25 = 1.5997097640e+01;
  static constexpr double kX34 = 2.4994195281e+01;
  static constexpr double kY2 = 3.7509291343e-01;
  static constexpr double kY3 = -2.2411521809e+00;
  static constexpr double kY4 = 5.3982113474e+00;
  static constexpr double kY5 = -6.5588143289e+00;
  static constexpr double kY6 = 4.0250747177e+00;
};
using ButterO5S62_5Lp3N20_83 = ButterO5<ParamsButterO5S62_5Lp3N20_83>;

struct ParamsButterO5S62_5Lp3N15_83 {
  static constexpr double kGain = 4.1144934894e+04;
  static constexpr double kX16 = 5.0412147780e+00;
  static constexpr double kX25 = 1.1206073890e+01;
  static constexpr double kX34 = 1.5412147780e+01;
  static constexpr double kY2 = 3.7509291343e-01;
  static constexpr double kY3 = -2.2411521809e+00;
  static constexpr double kY4 = 5.3982113474e+00;
  static constexpr double kY5 = -6.5588143289e+00;
  static constexpr double kY6 = 4.0250747177e+00;
};
using ButterO5S62_5Lp3N15_83 = ButterO5<ParamsButterO5S62_5Lp3N15_83>;

struct ParamsButterO5S125Lp3N41_66 {
  static constexpr double kGain = 1.556798187e+06;
  static constexpr double kX16 = 5.9996372182;
  static constexpr double kX25 = 15.9981860910;
  static constexpr double kX34 = 24.9963721820;
  static constexpr double kY2 = 0.6135091304;
  static constexpr double kY3 = -3.3668482801;
  static constexpr double kY4 = 7.4066054096;
  static constexpr double kY5 = -8.1654743335;
  static constexpr double kY6 = 4.5121464160;
};
using ButterO5S125Lp3N41_66 = ButterO5<ParamsButterO5S125Lp3N41_66>;

struct ParamsButterO5S125Lp3N46_66 {
  static constexpr double kGain = 1.764219683e+06;
  static constexpr double kX16 = 6.3992967530;
  static constexpr double kX25 = 17.9964837650;
  static constexpr double kX34 = 28.9929675300;
  static constexpr double kY2 = 0.6135091304;
  static constexpr double kY3 = -3.3668482801;
  static constexpr double kY4 = 7.4066054096;
  static constexpr double kY5 = -8.1654743335;
  static constexpr double kY6 = 4.5121464160;
};
using ButterO5S125Lp3N46_66 = ButterO5<ParamsButterO5S125Lp3N46_66>;

struct ParamsButterO5S125Lp3N33_33 {
  static constexpr double kGain = 1.146058161e+06;
  static constexpr double kX16 = 5.2082237395;
  static constexpr double kX25 = 12.0411186980;
  static constexpr double kX34 = 17.0822373950;
  static constexpr double kY2 = 0.6135091304;
  static constexpr double kY3 = -3.3668482801;
  static constexpr double kY4 = 7.4066054096;
  static constexpr double kY5 = -8.1654743335;
  static constexpr double kY6 = 4.5121464160;
};
using ButterO5S125Lp3N33_33 = ButterO5<ParamsButterO5S125Lp3N33_33>;
}  // namespace imu_integration

#endif  // IMU_INTEGRATION_BUTTERO5_H_
