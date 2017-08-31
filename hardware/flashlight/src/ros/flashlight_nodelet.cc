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


#include <flashlight/ros/flashlight_nodelet.h>

namespace flashlight {
namespace ros {

FlashlightNodelet::FlashlightNodelet() :
    ::ff_util::FreeFlyerNodelet() {
}

FlashlightNodelet::~FlashlightNodelet() {
}

namespace {

using Request = ::ff_hw_msgs::SetFlashlight::Request;
using Response = ::ff_hw_msgs::SetFlashlight::Response;

bool HandleError(i2c::Error err, Response *resp,
                 std::string const& name, const char * msg) {
  auto ec = std::error_code(err, std::generic_category());
  ROS_ERROR_NAMED(name, "error %s: %s", msg, ec.message().data());
  resp->success = false;
  resp->status_message.assign(ec.message());
  return true;
}

}  // namespace

bool FlashlightNodelet::OnService(Request &req, Response &resp) {
  i2c::Error err;

  resp.success = false;
  if (req.brightness > 200) {
    resp.status_message.assign("brightness out of range");
    return true;
  }

  if (!light_->SetBrightness(req.brightness, &err)) {
    return HandleError(err, &resp, getName(), "setting brightness");
  }

  if (req.brightness > 0) {
    if (!light_->SetEnabled(true, &err)) {
      return HandleError(err, &resp, getName(), "enabling flashlight");
    }
  } else {
    if (!light_->SetEnabled(false, &err)) {
      return HandleError(err, &resp, getName(), "disabling flashlight");
    }
  }

  resp.success = true;
  return true;
}

void FlashlightNodelet::Initialize(::ros::NodeHandle *nh) {
  std::string dev;
  int addr;

  config_reader::ConfigReader config_params;
  config_params.AddFile("hw/flashlights.config");
  if (!config_params.ReadFiles()) {
    NODELET_FATAL("Couldn't load lua parameters!");
    throw new std::runtime_error("Couldn't load lua parameters!");
  }

  // Parameters can't be change during execution so don't set up reload timer

  config_reader::ConfigReader::Table flashlight(&config_params,
                                                            GetName().c_str());

  if (!flashlight.GetStr("device", &dev)) {
    NODELET_FATAL("Device not specified!");
    throw new std::runtime_error("Device not specified!");
  }

  if (!flashlight.GetInt("address", &addr)) {
    NODELET_FATAL("no i2c address provided through parameters");
    throw new std::runtime_error("no i2c address provided");
  }

  if (addr < 0x07 || addr > 0x78) {
    NODELET_FATAL("i2c address (%02x) invalid", addr);
    throw std::range_error("addr out of range");
  }

  // Try to create our i2c bus
  i2c::Error err;
  auto bus = i2c::Open(dev, &err);
  if (!bus) {
    NODELET_FATAL("unable to open i2c bus");
    throw std::system_error(err, std::system_category());
  }

  light_.reset(new Flashlight(bus->DeviceAt(addr)));
  if (!light_->Init(&err)) {
    auto ec = std::error_code(err, std::generic_category());
    if (ec == std::errc::io_error) {
      NODELET_FATAL("flashlight does not appear to be at address 0x%02x", addr);
    } else {
      NODELET_FATAL("error initializing flashlight: %s", ec.message().data());
    }
    throw std::system_error(ec);
  }

  if (GetName() == "flashlight_front") {
    server_ = nh->advertiseService(SERVICE_HARDWARE_LIGHT_FRONT_CONTROL,
                                 &FlashlightNodelet::OnService,
                                 this);
  } else {
    server_ = nh->advertiseService(SERVICE_HARDWARE_LIGHT_AFT_CONTROL,
                                  &FlashlightNodelet::OnService,
                                  this);
  }
}

}  // namespace ros
}  // namespace flashlight

PLUGINLIB_EXPORT_CLASS(flashlight::ros::FlashlightNodelet, nodelet::Nodelet)
