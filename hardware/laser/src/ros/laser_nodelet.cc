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


#include <laser/ros/laser_nodelet.h>

namespace laser {
namespace ros {

LaserNodelet::LaserNodelet() :
  ::ff_util::FreeFlyerNodelet(NODE_LASER) {
}

LaserNodelet::~LaserNodelet() {
}

namespace {

using Request = ::ff_hw_msgs::SetEnabled::Request;
using Response = ::ff_hw_msgs::SetEnabled::Response;

}  // namespace

bool LaserNodelet::OnService(Request &req, Response &resp) {
  i2c::Error err;
  if (!(resp.success = laser_->SetEnabled(req.enabled, &err))) {
    auto ec = std::error_code(err, std::generic_category());
    NODELET_ERROR("error enabling/disabling laser: %s", ec.message().data());
    resp.status_message.assign(ec.message());
  }

  return true;
}

void LaserNodelet::Initialize(::ros::NodeHandle *nh) {
  std::string dev;
  int addr;

  config_reader::ConfigReader config_params;
  config_params.AddFile("hw/laser.config");
  if (!config_params.ReadFiles()) {
    NODELET_FATAL("Couldn't load lua parameters!");
    throw new std::runtime_error("Couldn't load lua parameters!");
  }

  // Parameters can't be changed during execution so don't set up reload timer

  if (!config_params.GetStr("device", &dev)) {
    NODELET_FATAL("Device not specified!");
    throw new std::runtime_error("Device not specified!");
  }

  if (!config_params.GetInt("address", &addr)) {
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

  laser_.reset(new Laser(bus->DeviceAt(addr)));
  if (!laser_->Init(&err)) {
    auto ec = std::error_code(err, std::generic_category());
    if (ec == std::errc::io_error) {
      NODELET_FATAL("laser does not appear to be at address 0x%02x", addr);
    } else {
      NODELET_FATAL("error initializing laser: %s", ec.message().data());
    }
    throw std::system_error(ec);
  }

  server_ = nh->advertiseService(SERVICE_HARDWARE_LASER_ENABLE,
                                 &LaserNodelet::OnService, this);
}

}  // namespace ros
}  // namespace laser

PLUGINLIB_EXPORT_CLASS(laser::ros::LaserNodelet, nodelet::Nodelet)
