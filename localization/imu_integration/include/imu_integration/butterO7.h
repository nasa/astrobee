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

#ifndef IMU_INTEGRATION_BUTTERO7_H_
#define IMU_INTEGRATION_BUTTERO7_H_

#include <imu_integration/filter.h>

#include <localization_common/logger.h>

#include <array>

namespace imu_integration {
template <class Params>
class ButterO7 : public Filter {
 public:
  ButterO7() : initialized_(false) {}
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
    yv_[last_index] = (xv_[0] + xv_[9]) + Params::kX18 * (xv_[1] + xv_[8]) + Params::kX27 * (xv_[2] + xv_[7]) +
                      Params::kX36 * (xv_[3] + xv_[6]) + Params::kX45 * (xv_[4] + xv_[5]) + (Params::kY2 * yv_[2]) +
                      (Params::kY3 * yv_[3]) + (Params::kY4 * yv_[4]) + (Params::kY5 * yv_[5]) +
                      (Params::kY6 * yv_[6]) + (Params::kY7 * yv_[7]) + (Params::kY8 * yv_[8]);

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
  std::array<double, 10> xv_;
  std::array<double, 10> yv_;
  bool initialized_;
};

struct ParamsButterO7S62_5Lp3N20_83 {
  static constexpr double kGain = 3.168744781e+06;
  static constexpr double kX18 = 7.9994195281;
  static constexpr double kX27 = 28.9959366960;
  static constexpr double kX36 = 62.9878100890;
  static constexpr double kX45 = 90.9796834820;
  static constexpr double kY2 = 0.2561484929;
  static constexpr double kY3 = -2.1400274434;
  static constexpr double kY4 = 7.7016541929;
  static constexpr double kY5 = -15.4832538240;
  static constexpr double kY6 = 18.7878862180;
  static constexpr double kY7 = -13.7678927060;
  static constexpr double kY8 = 5.6453639087;
};
using ButterO7S62_5Lp3N20_83 = ButterO7<ParamsButterO7S62_5Lp3N20_83>;
}  // namespace imu_integration

#endif  // IMU_INTEGRATION_BUTTERO7_H_
