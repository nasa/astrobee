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


#include <ff_common/init.h>

#include <glog/logging.h>
#include <gflags/gflags.h>

#include <laser/laser.h>
#include <i2c/i2c_new.h>

#include <cstdint>
#include <iomanip>
#include <system_error>  // NOLINT

constexpr std::uint8_t kDefaultAddr  = 0x27;
constexpr char         kDefaultDev[] = "/dev/i2c-1";

DEFINE_int32(addr, kDefaultAddr, "i2c address of laser");
DEFINE_string(dev, kDefaultDev, "i2c bus of laser");

DEFINE_bool(init, true, "initialize laser");
DEFINE_bool(on, false, "turn on laser");
DEFINE_bool(off, false, "turn off laser");

DECLARE_bool(logtostderr);

#define HANDLE_ERR(b) \
    do { \
      if (!(b)) { \
        auto ec = std::error_code(err, std::generic_category()); \
        if (ec == std::errc::io_error) { \
          LOG(ERROR) << "laser does not appear to be at address 0x" \
                     << std::right << std::hex << std::setfill('0') \
                     << std::setw(2) << FLAGS_addr; \
          return -1; \
        } \
        LOG(FATAL) << "error initializing laser: " << ec.message(); \
      } \
    } while (0);

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  i2c::Error err;

  auto bus = i2c::Open(FLAGS_dev, &err);
  if (!bus) {
    LOG(FATAL) << "error creating bus: "
               << std::error_code(err, std::system_category()).message();
  }

  bus->SetRetries(3);

  auto dev = bus->DeviceAt(static_cast<std::uint8_t>(FLAGS_addr & 0xff));
  laser::Laser pew_pew(dev);

  if (FLAGS_init) {
    HANDLE_ERR(pew_pew.Init(&err));
  }

  if (FLAGS_off) {
    HANDLE_ERR(pew_pew.SetEnabled(false, &err));
  }

  if (FLAGS_on) {
    HANDLE_ERR(pew_pew.SetEnabled(true, &err));
  }

  return 0;
}
