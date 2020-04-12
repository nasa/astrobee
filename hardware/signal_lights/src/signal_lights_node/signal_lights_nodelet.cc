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

// Standard ROS includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>

// FSW nodelet
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>

// Proxy library
#include <signal_lights/signal_lights.h>

// Services
#include <ff_hw_msgs/ConfigureLEDGroup.h>

#include <cerrno>
#include <cstring>

/**
 * \ingroup hw
 */
namespace signal_lights {

class SignalPair {
 public:
  std::string name;
  ros::Timer timer;
  signal_lights::SignalLights signal_lights;

  SignalPair(std::string name, signal_lights::Device device)
      : name(name), signal_lights(device) {}
};

class SignalLightsNodelet : public ff_util::FreeFlyerNodelet {
 public:
  SignalLightsNodelet() : ff_util::FreeFlyerNodelet(NODE_SIGNAL_LIGHTS) {}

  virtual ~SignalLightsNodelet() {
    ROS_INFO("Signal lights nodelet is shutting down");
    for (size_t i = 0; i < devices_.size(); i++) {
      devices_[i].signal_lights.SetAll(1, 1, 1);
      for (size_t p = 0; p < 4; p++) {
        devices_[i].signal_lights.Poll();
      }
    }
    ROS_INFO("Signal lights nodelet shut down complete");
  }

 protected:
  virtual void Initialize(ros::NodeHandle* nh) {
    config_params.AddFile("hw/signal_lights.config");
    if (!config_params.ReadFiles()) {
      ROS_ERROR("Could not open config file");
      return;
    }

    // Get the bus
    std::string i2c_bus;
    if (!config_params.GetStr("i2c_bus", &i2c_bus)) {
      ROS_ERROR("Could not get i2c bus");
      return;
    }

    // Get the rate
    double rate;
    // Get the max update rate
    if (!config_params.GetPosReal("control_rate_hz", &rate)) {
      ROS_ERROR("Could not get the max update rate.");
      return;
    }

    // Get the addresses
    config_reader::ConfigReader::Table devices(&config_params, "i2c_devices");

    // reserve enough space in devices vector
    devices_.reserve(devices.GetSize());

    for (int i = 0; i < devices.GetSize(); i++) {
      config_reader::ConfigReader::Table device;
      if (!devices.GetTable(i + 1, &device)) {
        ROS_ERROR("Could not get device table");
        return;
      }
      std::string name;
      if (!device.GetStr("name", &name)) {
        ROS_ERROR("Could not get device name");
        return;
      }
      int addr;
      if (!device.GetInt("addr", &addr)) {
        ROS_ERROR("Could not get device address");
        return;
      }

      // Add the device to the device map
      devices_.push_back(
          SignalPair(name, signal_lights::Device("/dev/i2c-2", (uint8_t)addr)));

      devices_[i].timer =
          nh->createTimer(ros::Duration(1.0 / rate),
                          boost::bind(&SignalLightsNodelet::TimerCallback, this,
                                      _1, &devices_[i]),
                          false, true);

      ROS_INFO("Setup one signal lights device!");
    }

    // LED update services
    sub_ = nh->subscribe(TOPIC_HARDWARE_SIGNAL_LIGHTS_CONFIG, 5,
                         &SignalLightsNodelet::ConfigureCallback, this);
  }

  // Callback for polling the signal light system
  void TimerCallback(ros::TimerEvent const& timer, SignalPair* s) {
    s->signal_lights.Poll();
  }

  // Callback for configuring the signal lights
  void ConfigureCallback(const ff_hw_msgs::ConfigureLEDGroup::ConstPtr& msg) {
    for (auto& d : devices_) {
      if (msg->side == d.name) {
        for (const auto& i : msg->leds) {
          d.signal_lights.Set(i.pos, i.red, i.green, i.blue);
        }
        return;
      }
    }
    ROS_ERROR("Signal light update received for unknown subsystem");
  }

 private:
  config_reader::ConfigReader config_params;  // LUA configuration reader
  std::vector<SignalPair> devices_;
  ros::Subscriber sub_;  // Configure service
};

PLUGINLIB_DECLARE_CLASS(signal_lights, SignalLightsNodelet,
                        signal_lights::SignalLightsNodelet, nodelet::Nodelet);

}  // namespace signal_lights
