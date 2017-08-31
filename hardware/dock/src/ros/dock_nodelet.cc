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


#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>

#include <config_reader/config_reader.h>
#include <i2c/i2c_new.h>
#include <dock/dock.h>
#include <dock/ros/dock_nodelet.h>

#include <stdexcept>
#include <string>
#include <system_error>  // NOLINT

namespace dock {
namespace ros {

constexpr char kConfigFile[] = "hw/dock.config";

void DockNodelet::onInit() {
  ::ros::NodeHandle &pnh = getPrivateNodeHandle();

  Params p;
  config_.AddFile(kConfigFile);
  if (!ReadParams(&p)) {
    NODELET_FATAL("error reading configuration");
    throw std::runtime_error("configuration error");
  }

  // Try to create our i2c bus
  i2c::Error err;
  auto bus = i2c::Open(p.bus, &err);
  if (!bus) {
    NODELET_FATAL("unable to open i2c bus");
    throw std::system_error(err, std::system_category());
  }

  dock_.reset(new Dock(bus->DeviceAt(p.dock_addr),
                       bus->DeviceAt(p.loop_addr)));
  if (!dock_->Init(&err)) {
    auto ec = std::error_code(err, std::generic_category());
    if (ec == std::errc::io_error) {
      NODELET_FATAL("loop does not appear to be at address 0x%02x",
                    p.loop_addr);
    } else {
      NODELET_FATAL("error initializing loop: %s",
                    ec.message().data());
    }
    throw std::system_error(ec);
  }

  server_ = pnh.advertiseService("control", &DockNodelet::OnService, this);
  pub_ = pnh.advertise<ff_hw_msgs::DockStateStamped>("state", 10, true);

  seq_ = 0;
  docked_ = false;
  force_ = true;
  max_tries_ = p.max_tries;
  num_tries_ = 0;

  // initial publish of state
  ::ros::TimerEvent ev;
  OnTimer(ev);

  // setup timer for future polling
  poll_timer_ = pnh.createTimer(::ros::Duration(p.interval),
                                &DockNodelet::OnTimer, this);
}

void DockNodelet::OnTimer(::ros::TimerEvent const& tev) {
  i2c::Error err;
  bool docked = false;

  ff_hw_msgs::DockStateStamped msg;
  msg.header.stamp = ::ros::Time::now();
  msg.header.frame_id = "world";
  msg.bay = ff_hw_msgs::DockStateStamped::BAY_UNKNOWN;

  if (!dock_->Docked(&docked, &err)) {
    NODELET_WARN("error checking docked state of astrobee");
    num_tries_++;
    if (num_tries_ < max_tries_)
      return;

    NODELET_ERROR("error checking docked state of astrobee");
    msg.header.seq = ++seq_;
    msg.state = ff_hw_msgs::DockStateStamped::ERROR;
    pub_.publish(msg);
    force_ = true;
    return;
  }

  num_tries_ = 0;

  // do not send updated status message if nothing changed, but we should
  // still send a message if this is the first time through
  if (docked_ == docked && !force_) {
    return;
  }

  // prepare the message
  msg.header.seq = ++seq_;
  msg.bay = ff_hw_msgs::DockStateStamped::BAY_UNKNOWN;

  if (docked) {
    msg.state = ff_hw_msgs::DockStateStamped::DOCKED;

    // TODO(tfmorse): figure out which bay we're docked in. waiting on the
    // dock to implement the api. (for now just say bay 2, since
    // that's the only one we currently dock to.)
    msg.bay = ff_hw_msgs::DockStateStamped::BAY_TWO;
  } else {
    msg.state = ff_hw_msgs::DockStateStamped::UNDOCKED;
  }

  pub_.publish(msg);
  docked_ = docked;
  force_ = false;
}

bool DockNodelet::ReadParams(Params *p) {
  if (!config_.ReadFiles()) {
    NODELET_ERROR("failed to read config files.");
    return false;
  }

  if (!config_.GetStr("bus", &(p->bus))) {
    NODELET_FATAL("config: no i2c bus specified");
    return false;
  }

  if (!config_.GetUInt("loop_addr", &(p->loop_addr))) {
    NODELET_FATAL("config: loopback i2c device address not specified");
    return false;
  }

  if (!config_.GetUInt("dock_addr", &(p->dock_addr))) {
    NODELET_FATAL("config: loopback i2c device address not specified");
    return false;
  }

  if (p->loop_addr < 0x07 || p->loop_addr > 0x78) {
    NODELET_FATAL("config: i2c loop address (%02x) invalid", p->loop_addr);
    return false;
  }

  if (p->dock_addr < 0x07 || p->dock_addr > 0x78) {
    NODELET_FATAL("config: i2c dock address (%02x) invalid", p->dock_addr);
    return false;
  }

  if (!config_.GetReal("check_interval", &(p->interval))) {
    NODELET_FATAL("config: interval not specified");
    return false;
  }

  if (!config_.GetInt("max_tries", &(p->max_tries))) {
    NODELET_FATAL("config: max_tries not specified");
    return false;
  }

  return true;
}

namespace {

using Request = ::ff_hw_msgs::Undock::Request;
using Response = ::ff_hw_msgs::Undock::Response;

}  // namespace

bool DockNodelet::OnService(Request &req, Response &resp) {  // NOLINT
  if (req.bay < 1 || req.bay > 2) {
    NODELET_ERROR("invalid (michael) bay *explosions* *lensflare*");
    resp.value = Response::ERR_INVALID_BAY;
    return true;
  }

  if (!docked_) {
    NODELET_ERROR("cannot push away when undocked");
    resp.value = Response::ERR_NOT_DOCKED;
    return true;
  }

  i2c::Error err;
  if (!dock_->Undock(static_cast<Bay>(req.bay), &err)) {
    auto ec = std::error_code(err, std::generic_category());
    NODELET_ERROR("error undocking: %s", ec.message().data());
    resp.value = Response::ERR_I2C;
  }

  return true;
}

}  // namespace ros
}  // namespace dock

PLUGINLIB_EXPORT_CLASS(dock::ros::DockNodelet, nodelet::Nodelet)
