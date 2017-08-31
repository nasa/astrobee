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

#ifndef FLASHLIGHT_FLASHLIGHT_H_
#define FLASHLIGHT_FLASHLIGHT_H_

#include <i2c/i2c_new.h>

namespace flashlight {

enum class Mode : std::uint8_t {
  GROUP = 1,
  INDIVIDUAL = 2
};

class Flashlight {
 public:
  explicit Flashlight(i2c::Device const &dev);

  bool Init(i2c::Error* ec);

  bool SetBrightness(int brightness, i2c::Error* ec);
  bool SetEnabled(bool enabled, i2c::Error* ec);
  bool SetMode(Mode mode, i2c::Error* ec);

  int brightness() { return brightness_; }
  bool enabled() { return enabled_; }
  Mode mode() { return mode_; }

 private:
  i2c::Device dev_;

  int brightness_;
  bool enabled_;
  Mode mode_;
};

}  // namespace flashlight

#endif  // FLASHLIGHT_FLASHLIGHT_H_
