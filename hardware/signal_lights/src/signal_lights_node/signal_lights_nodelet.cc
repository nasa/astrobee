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

typedef std::pair<ros::Timer, SignalLights> SignalPair;

typedef std::map<std::string, SignalPair> SignalLightsMap;

class SignalLightsNodelet : public ff_util::FreeFlyerNodelet {
 public:
  SignalLightsNodelet() : ff_util::FreeFlyerNodelet(NODE_SIGNAL_LIGHTS) {}

  virtual ~SignalLightsNodelet() {}

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
      auto it = devices_.emplace(
          name, SignalPair(ros::Timer(),
                           signal_lights::SignalLights(signal_lights::Device(
                               "/dev/i2c-2", (uint8_t)addr))));
      // Add an update timer to poll the signal light system
      it.first->second.first =
          nh->createTimer(ros::Duration(1.0 / rate),
                          boost::bind(&SignalLightsNodelet::TimerCallback, this,
                                      _1, devices_.find(name)),
                          false, true);
    }

    // LED update services
    sub_ = nh->subscribe(TOPIC_HARDWARE_SIGNAL_LIGHTS_CONFIG, 5,
                         &SignalLightsNodelet::ConfigureCallback, this);
  }

  // Callback for polling the signal light system
  void TimerCallback(ros::TimerEvent const& timer,
                     const SignalLightsMap::iterator& it) {
    it->second.second.Poll();
  }

  // Callback for configuring the signal lights
  void ConfigureCallback(const ff_hw_msgs::ConfigureLEDGroup::ConstPtr& msg) {
    auto it = devices_.find(msg->side);
    if (it == devices_.end()) {
      ROS_ERROR("Signal light update received for unknown subsystem");
      return;
    }
    for (const auto& i : msg->leds)
      it->second.second.Set(i.pos, i.red, i.green, i.blue);
  }

 private:
  config_reader::ConfigReader config_params;  // LUA configuration reader
  SignalLightsMap devices_;                   // Available devices
  ros::Subscriber sub_;                       // Configure service
};

PLUGINLIB_DECLARE_CLASS(signal_lights, SignalLightsNodelet,
                        signal_lights::SignalLightsNodelet, nodelet::Nodelet);

}  // namespace signal_lights
