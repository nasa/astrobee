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

#ifndef IMU_INTEGRATION_BUTTERO3_H_
#define IMU_INTEGRATION_BUTTERO3_H_

#include <imu_integration/filter.h>

#include <localization_common/logger.h>

#include <array>

namespace imu_integration {
template <class Params>
class ButterO3 : public Filter {
 public:
  ButterO3() : initialized_(false) {}
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
    yv_[last_index] = (xv_[0] + xv_[5]) + Params::kX14 * (xv_[1] + xv_[4]) + Params::kX23 * (xv_[2] + xv_[3]) +
                      (Params::kY2 * yv_[2]) + (Params::kY3 * yv_[3]) + (Params::kY4 * yv_[4]);
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
  std::array<double, 6> xv_;
  std::array<double, 6> yv_;
  bool initialized_;
};

struct ParamsButterO3S62_5Lp3N29_16 {
  static constexpr double kGain = 1.526307756e+03;
  static constexpr double kX14 = 4.9560160746;
  static constexpr double kX23 = 9.8680482239;
  static constexpr double kY2 = 0.5457868345;
  static constexpr double kY3 = -1.9654812209;
  static constexpr double kY4 = 2.3989592965;
};
using ButterO3S62_5Lp3N29_16 = ButterO3<ParamsButterO3S62_5Lp3N29_16>;

struct ParamsButterO3S62_5Lp3N20_83 {
  static constexpr double kGain = 1.1572342485e+03;
  static constexpr double kX14 = 3.9994195281;
  static constexpr double kX23 = 6.9982585842;
  static constexpr double kY2 = 5.4578683446e-01;
  static constexpr double kY3 = -1.9654812209e+00;
  static constexpr double kY4 = 2.3989592965e+00;
};
using ButterO3S62_5Lp3N20_83 = ButterO3<ParamsButterO3S62_5Lp3N20_83>;

struct ParamsButterO3S62_5Lp3N15_83 {
  static constexpr double kGain = 7.8754026491e+02;
  static constexpr double kX14 = 3.0412147780e+00;
  static constexpr double kX23 = 4.1236443339e+00;
  static constexpr double kY2 = 5.4578683446e-01;
  static constexpr double kY3 = -1.9654812209e+00;
  static constexpr double kY4 = 2.3989592965e+00;
};
using ButterO3S62_5Lp3N15_83 = ButterO3<ParamsButterO3S62_5Lp3N15_83>;

}  // namespace imu_integration

#endif  // IMU_INTEGRATION_BUTTERO3_H_
