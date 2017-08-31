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


#include <common/init.h>

#include <glog/logging.h>
#include <gflags/gflags.h>

#include <dock/dock.h>
#include <i2c/i2c_new.h>

#include <cstdint>
#include <iomanip>
#include <system_error>  // NOLINT

constexpr std::uint8_t kDefaultLoopAddr  = 0x27;
constexpr std::uint8_t kDefaultDockAddr  = 0x41;
constexpr char         kDefaultDev[] = "/dev/i2c-1";
constexpr std::uint8_t kDefaultBay = 1;

DEFINE_int32(loopaddr, kDefaultLoopAddr, "i2c address of loopback line");
DEFINE_int32(dockaddr, kDefaultDockAddr, "i2c address of dock");
DEFINE_string(dev, kDefaultDev, "i2c bus of everything");

DEFINE_bool(init, false, "initialize dock & loop");
DEFINE_bool(status, true, "is astrobee docked?");
DEFINE_bool(undock, false, "undock astrobee");
DEFINE_int32(bay, kDefaultBay, "undock astrobee from specifed bay [1, 2]");

DECLARE_bool(logtostderr);

#define HANDLE_ERR(b, addr, dev) \
    do { \
      if (!(b)) { \
        auto ec = std::error_code(err, std::generic_category()); \
        if (ec == std::errc::io_error) { \
          LOG(ERROR) << dev " does not appear to be at address 0x" \
                     << std::right << std::hex << std::setfill('0') \
                     << std::setw(2) << addr; \
          return -1; \
        } \
        LOG(FATAL) << "error initializing " dev ": " << ec.message(); \
      } \
    } while (0);

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  common::InitFreeFlyerApplication(&argc, &argv);

  i2c::Error err;

  auto bus = i2c::Open(FLAGS_dev, &err);
  if (!bus) {
    LOG(FATAL) << "error creating bus: "
               << std::error_code(err, std::system_category()).message();
  }

  bus->SetRetries(3);

  auto loop = bus->DeviceAt(static_cast<std::uint8_t>(FLAGS_loopaddr & 0xff));
  auto dock = bus->DeviceAt(static_cast<std::uint8_t>(FLAGS_dockaddr & 0xff));
  dock::Dock d(dock, loop);

  if (FLAGS_init) {
    HANDLE_ERR(d.Init(&err), FLAGS_loopaddr, "loop");
  }

  if (FLAGS_status) {
    bool docked;
    HANDLE_ERR(d.Docked(&docked, &err), FLAGS_loopaddr, "loop");
    LOG(INFO) << "state: " << (docked ? "DOCKED" : "UNDOCKED");
  }

  if (FLAGS_undock) {
    if (FLAGS_bay < 1 || FLAGS_bay > 2) {
      LOG(ERROR) << "bay must be either 1 or 2";
      return -1;
    }

    bool docked;
    HANDLE_ERR(d.Docked(&docked, &err), FLAGS_loopaddr, "loop");
    if (!docked) {
      LOG(ERROR) << "must be docked to undock!";
      return -1;
    }

    dock::Bay b = (FLAGS_bay == 1) ? dock::kBayOne : dock::kBayTwo;
    HANDLE_ERR(d.Undock(b, &err), FLAGS_dockaddr, "dock");
    LOG(INFO) << "Undocked from bay " << FLAGS_bay;
  }

  return 0;
}
