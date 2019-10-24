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

// Common freeflyer code
#include <common/init.h>

// Gflag code
#include <gflags/gflags.h>
#include <gflags/gflags_completions.h>

// Proxy library
#include <signal_lights/signal_lights.h>

// C++ includes
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <iostream>
#include <thread>

// Supported effects
typedef enum {
  LEDC_EFFECT_ALL_OFF = 0,
  LEDC_EFFECT_ALL_ON,
  LEDC_EFFECT_ONE_ON,
  LEDC_EFFECT_CHASE,
  LEDC_EFFECT_STROBE,
  NUM_EFFECTS
} LEDC_EFFECT;

// Gflag defaults
DEFINE_string(y, "/dev/i2c-2", "i2c bus of signal lights");
DEFINE_int32(a, 0x23, "i2c address of signal lights");

// Single-switch options
DEFINE_int32(m, 1, "Mode: [0] shutdown [1] nominal");
DEFINE_int32(s, 0, "Scale: [0] 255 [1] 128 [2] 64 [3] 32");
DEFINE_int32(e, 0, "Effect: [0] off [1] on [2] single [3] chase [4] strobe)");
DEFINE_double(f, 30.0, "Update frequency in Hz (0 - 60))");
DEFINE_int32(r, 31, "Red value (0 - 31)");
DEFINE_int32(g, 31, "Green value (0 - 31)");
DEFINE_int32(b, 31, "Blue value (0 - 31)");
DEFINE_int32(d, 2000, "Duration: [Effect 3 and 4] duration in milliseconds");
DEFINE_int32(x, 5, "Argument: [Effect 2] index [Effect 3] length");

// Set all pixels for a given command
void SetPixels(signal_lights::SignalLights &sig, uint8_t effect, uint16_t t,
               uint8_t arg, uint64_t timestamp, uint8_t r, uint8_t g,
               uint8_t b) {
  // Determine the phase offset in the cycle
  uint16_t phase = (t > 0 ? timestamp % t : 1);
  uint16_t strobe = (phase < t / 2 ? phase : t - phase);

  // Calculate the pixel values for the effect
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    switch (effect) {
      // Turn a all LEDs off
      case LEDC_EFFECT_ALL_OFF:
        sig.Set(i, 0, 0, 0);
        break;
      // Turn a all LEDs on
      case LEDC_EFFECT_ALL_ON:
        sig.Set(i, r, g, b);
        break;
      // Turn a single LED and leave the others untouched
      case LEDC_EFFECT_ONE_ON:
        if (i == arg) sig.Set(i, r, g, b);
        break;
      // Chaser
      case LEDC_EFFECT_CHASE: {
        uint8_t chase = i - NUM_LEDS * phase / t;
        if (chase >= 0 && chase <= arg)
          sig.Set(i, r * chase / arg, g * chase / arg, b * chase / arg);
        else
          sig.Set(i, 0, 0, 0);
      } break;
      // Strobe
      case LEDC_EFFECT_STROBE:
        sig.Set(i, r * 2 * strobe / t, g * 2 * strobe / t, b * 2 * strobe / t);
        break;
    }
  }
}

// Main entry point for application
int main(int argc, char **argv) {
  // Set up gflags
  google::SetUsageMessage("Usage: signal_lights_tool <options> [value]\n");
  google::SetVersionString("0.3.0");
  common::InitFreeFlyerApplication(&argc, &argv);

  // Check the values entered by the user
  if (FLAGS_m < 0 || FLAGS_m > 1) {
    std::cerr << "Mode must be either 0 or 1" << std::endl;
    return -1;
  }
  if (FLAGS_s < 0 || FLAGS_s > 1) {
    std::cerr << "Scale must be in range 0 - 3" << std::endl;
    return -1;
  }
  if (FLAGS_e < 0 || FLAGS_e > 4) {
    std::cerr << "Effect must be in range 0 - 4" << std::endl;
    return -1;
  }
  if (FLAGS_f < 1.0 || FLAGS_f > 60.0) {
    std::cerr << "Frequency must be in range 1 - 60" << std::endl;
    return -1;
  }
  if (FLAGS_r < 0 || FLAGS_r > 31 || FLAGS_g < 0 || FLAGS_g > 31 ||
      FLAGS_b < 0 || FLAGS_b > 31) {
    std::cerr << "RGB values must be in range 0 - 31" << std::endl;
    return -1;
  }
  if (FLAGS_d < 0 || FLAGS_d > 10000) {
    std::cerr << "Duration must be in range 0 - 10000" << std::endl;
    return -1;
  }
  if (FLAGS_x < 0 || FLAGS_x > 43) {
    std::cerr << "Argument must be in range 0 - 43" << std::endl;
    return -1;
  }

  // Echo command
  std::cout << "Command " << std::endl
            << " - Mode: " << FLAGS_m << std::endl
            << " - Frequency: " << FLAGS_f << std::endl
            << " - Scale: " << FLAGS_s << std::endl
            << " - Red: " << FLAGS_r << std::endl
            << " - Green: " << FLAGS_g << std::endl
            << " - Blue: " << FLAGS_b << std::endl
            << " - Effect: " << FLAGS_e << std::endl
            << " - Duration: " << FLAGS_d << std::endl
            << " - Argument: " << FLAGS_x << std::endl;

  auto device = signal_lights::Device(std::string(FLAGS_y).c_str(), static_cast<uint8_t>(FLAGS_a));

  // Add the device to the device map
  signal_lights::SignalLights sig(device);

  // Convert to a chrono type
  // std::chrono::time_point<std::chrono::steady_clock>
  auto delay_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::duration<double>(sig.GetPollDuration(FLAGS_f)));

  // Block while sending the data
  auto tb = std::chrono::steady_clock::now();
  while (true) {
    auto ti = std::chrono::steady_clock::now();

    // Update all LEDs
    SetPixels(
        sig, FLAGS_e, FLAGS_d, FLAGS_x,
        std::chrono::duration_cast<std::chrono::milliseconds>(ti - tb).count(),
        FLAGS_r, FLAGS_g, FLAGS_b);

    // Poll the low-level driver, and check if we have new metadata
    if (sig.Poll()) {
      std::cout << "Firmware version:" << std::endl;
      std::cout << "- Hash: " << sig.GetHash() << std::endl;
      std::cout << "- Build time: " << sig.GetTime() << std::endl;
    }

    // Sleep for a little bit
    std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::microseconds>(delay_us));
  }

  // Success
  return 0;
}
