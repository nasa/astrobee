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

#include <imu_integration/butterworth_lowpass_filter_5th_order.h>

#include <glog/logging.h>

namespace imu_integration {
ButterworthLowpassFilter5thOrder::ButterworthLowpassFilter5thOrder() : initialized_(false) {}

void ButterworthLowpassFilter5thOrder::Initialize(const double first_value, const double gain) {
  // TODO(rsoussan): Look into filtfilt (forward and backward filtering to eliminate phase shift)
  // Initialize values so filter output starts at first value
  for (auto& val : xv_) {
    val = first_value / gain;
  }
  for (auto& val : yv_) {
    val = first_value;
  }

  initialized_ = true;
}

double ButterworthLowpassFilter5thOrder::AddValue(const double value) {
  // Corner frequency at 3Hz and notch filter at 29.16 Hz, 5th order
  /* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 5 -a 4.8000000000e-02 0.0000000000e+00 -Z 4.6656000000e-01
   -l */
  static constexpr double GAIN = 7.974174280e+04;

  if (!initialized_) Initialize(value, GAIN);

  const int last_index = xv_.size() - 1;
  // Shift input and output vals
  for (int i = 0; i < last_index; ++i) {
    xv_[i] = xv_[i + 1];
    yv_[i] = yv_[i + 1];
  }
  // Add new values
  xv_[last_index] = value / GAIN;
  // Generate new output
  yv_[last_index] = (xv_[0] + xv_[7]) + 6.9560160746 * (xv_[1] + xv_[6]) + 20.7800803730 * (xv_[2] + xv_[5]) +
                    34.5601607460 * (xv_[3] + xv_[4]) + (-0.0000000000 * yv_[0]) + (-0.0000000000 * yv_[1]) +
                    (0.3750929134 * yv_[2]) + (-2.2411521809 * yv_[3]) + (5.3982113474 * yv_[4]) +
                    (-6.5588143289 * yv_[5]) + (4.0250747177 * yv_[6]);
  // Return most recent output
  return yv_[last_index];
}
}  // namespace imu_integration
