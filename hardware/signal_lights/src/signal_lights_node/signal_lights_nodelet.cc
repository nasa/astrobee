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

// FSW shared libraries
#include <config_reader/config_reader.h>

// FSW nodelet
#include <ff_common/ff_names.h>
#include <ff_util/ff_component.h>

// Proxy library
#include <signal_lights/signal_lights.h>

// Services
#include <ff_hw_msgs/msg/configure_led_group.hpp>
namespace ff_hw_msgs {
typedef msg::ConfigureLEDGroup ConfigureLEDGroup;
}  // namespace ff_hw_msgs

#include <cerrno>
#include <cstring>

/**
 * \ingroup hw
 */
namespace signal_lights {

FF_DEFINE_LOGGER("signal_lights")

class SignalPair {
 public:
  std::string name;
  ff_util::FreeFlyerTimer timer;
  signal_lights::SignalLights signal_lights;

  SignalPair(std::string name, signal_lights::Device device)
      : name(name), signal_lights(device) {}
};

class SignalLightsNodelet : public ff_util::FreeFlyerComponent {
 public:
  explicit SignalLightsNodelet(const rclcpp::NodeOptions& options) :
  ff_util::FreeFlyerComponent(options, NODE_SIGNAL_LIGHTS, true) {}

  virtual ~SignalLightsNodelet() {
    FF_INFO("Signal lights nodelet is shutting down");
    for (size_t i = 0; i < devices_.size(); i++) {
      devices_[i].signal_lights.SetAll(1, 1, 1);
      for (size_t p = 0; p < 4; p++) {
        devices_[i].signal_lights.Poll();
      }
    }
    FF_INFO("Signal lights nodelet shut down complete");
  }

 protected:
  virtual void Initialize(NodeHandle nh) {
    config_params.AddFile("hw/signal_lights.config");
    if (!config_params.ReadFiles()) {
      FF_ERROR("Could not open config file");
      return;
    }

    // Get the bus
    std::string i2c_bus;
    if (!config_params.GetStr("i2c_bus", &i2c_bus)) {
      FF_ERROR("Could not get i2c bus");
      return;
    }

    // Get the rate
    double rate;
    // Get the max update rate
    if (!config_params.GetPosReal("control_rate_hz", &rate)) {
      FF_ERROR("Could not get the max update rate.");
      return;
    }

    // Get the addresses
    config_reader::ConfigReader::Table devices(&config_params, "i2c_devices");

    // reserve enough space in devices vector
    devices_.reserve(devices.GetSize());

    for (int i = 0; i < devices.GetSize(); i++) {
      config_reader::ConfigReader::Table device;
      if (!devices.GetTable(i + 1, &device)) {
        FF_ERROR("Could not get device table");
        return;
      }
      std::string name;
      if (!device.GetStr("name", &name)) {
        FF_ERROR("Could not get device name");
        return;
      }
      int addr;
      if (!device.GetInt("addr", &addr)) {
        FF_ERROR("Could not get device address");
        return;
      }

      // Add the device to the device map
      devices_.push_back(
          SignalPair(name, signal_lights::Device("/dev/i2c-2", (uint8_t)addr)));

      devices_[i].timer.createTimer(1.0 / rate,
                          std::bind(&SignalLightsNodelet::TimerCallback, this,
                                    &devices_[i]),
                          nh, false, true);

      FF_INFO("Setup one signal lights device!");
    }

    // LED update services
    sub_ = FF_CREATE_SUBSCRIBER(nh, ff_hw_msgs::ConfigureLEDGroup, TOPIC_HARDWARE_SIGNAL_LIGHTS_CONFIG, 5,
                         std::bind(&SignalLightsNodelet::ConfigureCallback, this, std::placeholders::_1));
  }

  // Callback for polling the signal light system
  void TimerCallback(SignalPair* s) {
    s->signal_lights.Poll();
  }

  // Callback for configuring the signal lights
  void ConfigureCallback(const std::shared_ptr<ff_hw_msgs::ConfigureLEDGroup> msg) {
    for (auto& d : devices_) {
      if (msg->side == d.name) {
        for (const auto& i : msg->leds) {
          d.signal_lights.Set(i.pos, i.red, i.green, i.blue);
        }
        return;
      }
    }
    FF_ERROR("Signal light update received for unknown subsystem");
  }

 private:
  config_reader::ConfigReader config_params;  // LUA configuration reader
  std::vector<SignalPair> devices_;
  rclcpp::Subscription<ff_hw_msgs::ConfigureLEDGroup>::SharedPtr sub_;  // Configure service
};


}  // namespace signal_lights

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(signal_lights::SignalLightsNodelet)
