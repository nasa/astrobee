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
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>

// FSW nodelet
#include <ff_util/ff_nodelet.h>

// FSW services
#include <sensor_msgs/Temperature.h>

// PMC helper libraries
#include <temp_monitor/temp_monitor.h>

/**
 * \ingroup hw
 */
namespace temp_monitor {

class TempMonitorNode : public ff_util::FreeFlyerNodelet {
  using TempMonitorMap = std::map < TempMonitorPtr, ros::Publisher >;

 public:
  // Constructor
  TempMonitorNode() : ff_util::FreeFlyerNodelet(NODE_TEMP_MONITOR) {}

  // Destructor - make sure we free dynamically allocated memory
  virtual ~TempMonitorNode() {}

 protected:
  virtual void Initialize(ros::NodeHandle *nh) {
    config_reader::ConfigReader config_params;
    config_params.AddFile("hw/temp_monitor.config");
    if (!config_params.ReadFiles())
      ROS_FATAL("Unable to load LUA parameters!");

    // Read the device information from the config table
    config_reader::ConfigReader::Table tm;
    if (!config_params.GetTable("temp_monitor", &tm))
      FF_FATAL("Could get temp_monitor item in config file");

    // Read the device information from the config table
    config_reader::ConfigReader::Table devices;
    if (!tm.GetTable("devices", &devices))
      FF_FATAL("Could get devices item in temp_monitor config file");

    // Iterate over all devices
    for (int i = 0; i < devices.GetSize(); i++) {
      config_reader::ConfigReader::Table device_info;
      if (!devices.GetTable(i + 1, &device_info))
        FF_FATAL("Could get row in table table");

      // Get the name of the device and check it matches the name of this node
      std::string name;
      if (!device_info.GetStr("name", &name))
        FF_FATAL("Could not find row 'name' in table");
      std::string type;
      if (!device_info.GetStr("type", &type))
        FF_FATAL("Could not find row 'type' in table");

      // Get the name of the device and check it matches the name of this node
      config_reader::ConfigReader::Table i2c_info;
      if (!device_info.GetTable("i2c", &i2c_info))
        FF_FATAL("Could not find table 'i2c' in table");
      std::string dev;
      if (!i2c_info.GetStr("device", &dev))
        FF_FATAL("Could not read the i2c device from the config");
      uint32_t address;
      if (!i2c_info.GetUInt("address", &address))
        FF_FATAL("Could not read the i2c address from the config");
      uint32_t retries;
      if (!i2c_info.GetUInt("retries", &retries))
        FF_FATAL("Could read the i2c retry count from the config");

      // Try and open the bus
      i2c::Error err;
      auto bus = i2c::Open(dev, &err);
      if (!bus)
        FF_FATAL("Could not open the i2c bus");
      bus->SetRetries(retries);
      // Try and contact the slave device
      auto device = bus->DeviceAt(address);
      if (!device)
        FF_FATAL("Could not find the i2c slave address");

      // Check if we already have this bus open
      TempMonitorPtr temp_sensor = TempMonitorFactory::Create(type, device);
      if (temp_sensor == nullptr) {
        FF_ERROR("Unable to create sensor " << name);
        continue;
      }

      // Setup message: temperature
      monitors_[temp_sensor] = nh->advertise < sensor_msgs::Temperature > (
        TOPIC_HARDWARE_TEMP_MONITOR_PREFIX + name, 1);
    }

    // Start the rate timer for temperature publishing
    double rate;
    if (!tm.GetReal("rate", &rate))
      FF_FATAL("Could not find row 'rate' in table");
    timer_ = nh->createTimer(ros::Duration(ros::Rate(rate)),
      &TempMonitorNode::TimerCallback, this, false, true);
  }

  // Timer callback for telemetry publishing
  void TimerCallback(const ros::TimerEvent & event) {
    double datum;
    for (TempMonitorMap::iterator it = monitors_.begin(); it != monitors_.end(); it++) {
      if (it->first->GetTemperature(&datum)) {
        sensor_msgs::Temperature msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = GetPlatform();
        msg.temperature = datum;
        msg.variance = 0;  // Unknown
        it->second.publish(msg);
      }
    }
  }

 private:
  TempMonitorMap monitors_;                     // Monitor data structure
  ros::Timer timer_;                            // Callback timer
};

PLUGINLIB_DECLARE_CLASS(temp_monitor, TempMonitorNode,
                        temp_monitor::TempMonitorNode, nodelet::Nodelet);

}  // namespace temp_monitor
