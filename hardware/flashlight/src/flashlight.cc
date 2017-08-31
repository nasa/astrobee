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

#include "flashlight/flashlight.h"

#include <i2c/i2c_new.h>

#include <glog/logging.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <thread>

namespace flashlight {

namespace {

constexpr std::uint8_t REG_MODE1    = 0x00;
constexpr std::uint8_t REG_MODE2    = 0x01;
constexpr std::uint8_t REG_PWM0     = 0x02;
constexpr std::uint8_t REG_PWM1     = 0x03;  // unused
constexpr std::uint8_t REG_PWM2     = 0x04;  // unused
constexpr std::uint8_t REG_PWM3     = 0x05;  // unused
constexpr std::uint8_t REG_GRPPWM   = 0x06;
constexpr std::uint8_t REG_GRPFREQ  = 0x07;  // unused
constexpr std::uint8_t REG_LEDOUT   = 0x08;
constexpr std::uint8_t REG_SUBADR1  = 0x09;  // unused
constexpr std::uint8_t REG_SUBADR2  = 0x0A;  // unused
constexpr std::uint8_t REG_SUBADR3  = 0x0B;  // unused
constexpr std::uint8_t REG_ALLCALL  = 0x0C;  // unused

constexpr std::uint8_t AINC_NONE    = 0x00;
constexpr std::uint8_t AINC_ALL     = 0x80;
constexpr std::uint8_t AINC_IND     = 0xA0;
constexpr std::uint8_t AINC_GRP     = 0xC0;
constexpr std::uint8_t AINC_IND_GRP = AINC_IND | AINC_GRP;

constexpr std::uint8_t MODE1_ALLCALL = 0x01;
constexpr std::uint8_t MODE1_SUB3    = 0x01 << 1;  // unused
constexpr std::uint8_t MODE1_SUB2    = 0x01 << 2;  // unused
constexpr std::uint8_t MODE1_SUB1    = 0x01 << 3;  // unused
constexpr std::uint8_t MODE1_SLEEP   = 0x01 << 4;

constexpr std::uint8_t LEDOUT_OFF = 0x00;
constexpr std::uint8_t LEDOUT_ON  = 0x01;  // unused
constexpr std::uint8_t LEDOUT_IND = 0x02;
constexpr std::uint8_t LEDOUT_GRP = 0x03;

constexpr std::chrono::microseconds WAKE_DELAY(500);

}  // namespace

Flashlight::Flashlight(i2c::Device const& dev)
  : dev_(dev), brightness_(0), enabled_(false), mode_(Mode::GROUP) {
}

bool Flashlight::Init(i2c::Error* ec) {
  // Wake up chip, set to ignore all call, open-drain, non-inverted drivers,
  // group dimming control.
  const std::uint8_t buf[] { AINC_ALL | REG_MODE1, 0x00, 0x00 };
  if (dev_.Write(buf, sizeof(buf), ec) < 0) {
    LOG(ERROR) << "error initializing flashlight";
    return false;
  }

  // Wait for internal oscillator to come online
  std::this_thread::sleep_for(WAKE_DELAY);

  // Set LED0 to be full-on (we dim via the group pwm)
  // Set LED[1-3] to be off.
  // Set group PWM to be off for now
  const std::uint8_t buf2[] {
    AINC_ALL | REG_PWM0,
    0xff, 0x00, 0x00, 0x00, 0x00 };
  if (dev_.Write(buf2, sizeof(buf2), ec) < 0) {
    LOG(ERROR) << "error configuring flashlight";
    return false;
  }

  return true;
}

bool Flashlight::SetBrightness(int brightness, i2c::Error* ec) {
  std::uint8_t reg = REG_GRPPWM;
  if (mode_ == Mode::INDIVIDUAL) {
    reg = REG_PWM0;
  }

  brightness = std::max(0, std::min(brightness, 255));

  if (dev_.WriteRegister(reg,
                         static_cast<std::uint8_t>(brightness & 0xFF),
                         ec) < 0) {
    LOG(ERROR) << "error setting brightness";
    return false;
  }

  brightness_ = brightness;
  return true;
}

bool Flashlight::SetMode(Mode mode, i2c::Error* ec) {
  std::uint8_t buf[7] { 0x00 };

  if (mode == Mode::INDIVIDUAL) {
    buf[0] = static_cast<uint8_t>(brightness_);
    buf[4] = 0x00;  // don't care
    buf[6] = LEDOUT_IND;
  } else {
    buf[0] = 0xff;
    buf[4] = static_cast<uint8_t>(brightness_);
    buf[6] = LEDOUT_GRP;
  }

  if (!enabled_)
    buf[6] = LEDOUT_OFF;

  if (dev_.WriteRegister(AINC_ALL | REG_PWM0, buf, sizeof(buf), ec) < 0) {
    LOG(ERROR) << "error swapping brightness and such";
    return false;
  }

  mode_ = mode;
  return true;
}

bool Flashlight::SetEnabled(bool enabled, i2c::Error* ec) {
  std::uint8_t val = LEDOUT_OFF;
  if (enabled && mode_ == Mode::GROUP) {
    val = LEDOUT_GRP;
  } else if (enabled && mode_ == Mode::INDIVIDUAL) {
    val = LEDOUT_IND;
  }

  if (dev_.WriteRegister(REG_LEDOUT, val, ec) < 0) {
    LOG(ERROR) << "error enabling/disabling flashlight";
    return false;
  }

  enabled_ = enabled;
  return true;
}


}  // namespace flashlight
