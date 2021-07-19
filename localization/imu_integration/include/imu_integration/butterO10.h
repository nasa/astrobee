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

#ifndef IMU_INTEGRATION_BUTTERO10_H_
#define IMU_INTEGRATION_BUTTERO10_H_

#include <imu_integration/filter.h>

#include <localization_common/logger.h>

#include <array>

namespace imu_integration {
template <class Params>
class ButterO10 : public Filter {
 public:
  ButterO10() : initialized_(false) {}
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
    yv_[last_index] = (xv_[0] + xv_[12]) + Params::kX111 * (xv_[1] + xv_[11]) + Params::kX210 * (xv_[2] + xv_[10]) +
                      Params::kX39 * (xv_[3] + xv_[9]) + Params::kX48 * (xv_[4] + xv_[8]) +
                      Params::kX57 * (xv_[5] + xv_[7]) + Params::kX6 * xv_[6] + (Params::kY2 * yv_[2]) +
                      (Params::kY3 * yv_[3]) + (Params::kY4 * yv_[4]) + (Params::kY5 * yv_[5]) +
                      (Params::kY6 * yv_[6]) + (Params::kY7 * yv_[7]) + (Params::kY8 * yv_[8]) +
                      (Params::kY9 * yv_[9]) + (Params::kY10 * yv_[10]) + (Params::kY11 * yv_[11]);

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
  std::array<double, 13> xv_;
  std::array<double, 13> yv_;
  bool initialized_;
};

struct ParamsButterO10S62_5Lp3N20_83 {
  static constexpr double kGain = 1.204389877e+09;
  static constexpr double kX111 = 10.9994195280;
  static constexpr double kX210 = 55.9941952810;
  static constexpr double kX39 = 174.9738787600;
  static constexpr double kX48 = 374.930343370;
  static constexpr double kX57 = 581.8781008900;
  static constexpr double kX6 = 671.8537210700;
  static constexpr double kY2 = -0.1440551095;
  static constexpr double kY3 = 1.7182080129;
  static constexpr double kY4 = -9.2532994947;
  static constexpr double kY5 = 29.6350557040;
  static constexpr double kY6 = -62.5163645520;
  static constexpr double kY7 = 90.7861740400;
  static constexpr double kY8 = -91.9332301310;
  static constexpr double kY9 = 64.1155101770;
  static constexpr double kY10 = -29.4805657920;
  static constexpr double kY11 = 8.0725645969;
};
using ButterO10S62_5Lp3N20_83 = ButterO10<ParamsButterO10S62_5Lp3N20_83>;
}  // namespace imu_integration

#endif  // IMU_INTEGRATION_BUTTERO10_H_
