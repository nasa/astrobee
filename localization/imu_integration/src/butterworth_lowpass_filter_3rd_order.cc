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

#include <imu_integration/butterworth_lowpass_filter_3rd_order.h>

#include <glog/logging.h>

namespace imu_integration {
ButterworthLowpassFilter3rdOrder::ButterworthLowpassFilter3rdOrder() : initialized_(false) {}

void ButterworthLowpassFilter3rdOrder::Initialize(const double first_value, const double gain) {
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

double ButterworthLowpassFilter3rdOrder::AddValue(const double value) {
  // Corner frequency at 3Hz and notch filter at 29.16 Hz, 3rd order
  /* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
    Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 3 -a 4.8000000000e-02 0.0000000000e+00 -Z 4.6656000000e-01
    -l */
  static constexpr double GAIN = 1.526307756e+03;

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
  yv_[last_index] = (xv_[0] + xv_[5]) + 4.9560160746 * (xv_[1] + xv_[4]) + 9.8680482239 * (xv_[2] + xv_[3]) +
                    (-0.0000000000 * yv_[0]) + (-0.0000000000 * yv_[1]) + (0.5457868345 * yv_[2]) +
                    (-1.9654812209 * yv_[3]) + (2.3989592965 * yv_[4]);
  // Return most recent output
  return yv_[last_index];
}
}  // namespace imu_integration
