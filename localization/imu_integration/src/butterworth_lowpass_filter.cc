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

#include <imu_integration/butterworth_lowpass_filter.h>

#include <glog/logging.h>

namespace imu_integration {
ButterworthLowpassFilter::ButterworthLowpassFilter() : initialized_(false) {}

void ButterworthLowpassFilter::Initialize(const double first_value, const double gain) {
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

double ButterworthLowpassFilter::AddValue(const double value) {
  // Corner frequency at 3Hz and notch filter at 29.16 Hz, 1st order
  /* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
     Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 1 -a 4.8000000000e-02 0.0000000000e+00
     -Z 4.6656000000e-01 -l */
  static constexpr double GAIN = 2.999100930e+01;

  if (!initialized_) Initialize(value, GAIN);

  // Shift input vals
  xv_[0] = xv_[1];
  xv_[1] = xv_[2];
  xv_[2] = xv_[3];
  // Add new value
  xv_[3] = value / GAIN;
  // Shift output vals
  yv_[0] = yv_[1];
  yv_[1] = yv_[2];
  yv_[2] = yv_[3];
  // Generate new output
  yv_[3] = (xv_[0] + xv_[3]) + 2.9560160746 * (xv_[1] + xv_[2]) + (-0.0000000000 * yv_[0]) + (-0.0000000000 * yv_[1]) +
           (0.7361865327 * yv_[2]);
  // Return most recent output
  return yv_[3];
}
}  // namespace imu_integration
