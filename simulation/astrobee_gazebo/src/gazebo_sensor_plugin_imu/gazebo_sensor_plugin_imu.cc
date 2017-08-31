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

// ROS includes
#include <ros/ros.h>

// IMU Sensor message
#include <sensor_msgs/Imu.h>

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// STL includes
#include <string>

namespace gazebo {

class GazeboSensorPluginImu : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginImu() : FreeFlyerSensorPlugin(NODE_EPSON_IMU) {
    msg_.orientation.x = 0;
    msg_.orientation.y = 0;
    msg_.orientation.z = 0;
    msg_.orientation.w = 0;
    msg_.orientation_covariance
      = { -1, -1, -1, -1, -1, -1, -1, -1, -1 };
    msg_.angular_velocity_covariance
      = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    msg_.linear_acceleration_covariance
      = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  }

  virtual ~GazeboSensorPluginImu() {}

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle* nh, sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast < sensors::ImuSensor > (sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginImu requires an imu sensor as a parent.\n";
      return;
    }
    // Get the name of the link to which this node is attached
    msg_.header.frame_id = GetFrame();

    // Offer IMU messages to those which need them
    pub_ = nh->advertise < sensor_msgs::Imu > (TOPIC_HARDWARE_IMU, 1,
      boost::bind(&GazeboSensorPluginImu::ToggleCallback, this),
      boost::bind(&GazeboSensorPluginImu::ToggleCallback, this));

    // Connect to the sensor update event.
    update_ = sensor_->ConnectUpdated(
      std::bind(&GazeboSensorPluginImu::UpdateCallback, this));
  }

  // Turn camera on or off based on topic subscription
  void ToggleCallback() {
    if (pub_.getNumSubscribers() > 0)
      sensor_->SetActive(true);
    else
      sensor_->SetActive(false);
  }

  // Called on each sensor update event
  virtual void UpdateCallback() {
    if (!sensor_->IsActive()) return;
    msg_.header.stamp.sec = sensor_->LastMeasurementTime().sec;
    msg_.header.stamp.nsec = sensor_->LastMeasurementTime().nsec;
    msg_.orientation.x = sensor_->Orientation().X();
    msg_.orientation.y = sensor_->Orientation().Y();
    msg_.orientation.z = sensor_->Orientation().Z();
    msg_.orientation.w = sensor_->Orientation().W();
    msg_.angular_velocity.x = sensor_->AngularVelocity().X();
    msg_.angular_velocity.y = sensor_->AngularVelocity().Y();
    msg_.angular_velocity.z = sensor_->AngularVelocity().Z();
    msg_.linear_acceleration.x = sensor_->LinearAcceleration().X();
    msg_.linear_acceleration.y = sensor_->LinearAcceleration().Y();
    msg_.linear_acceleration.z = sensor_->LinearAcceleration().Z();
    pub_.publish(msg_);
  }

 private:
  // ROS variables
  ros::Publisher pub_;
  // Gazebo variables
  sensors::ImuSensorPtr sensor_;
  event::ConnectionPtr update_;
  // Intermediary data
  sensor_msgs::Imu msg_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginImu)

}   // namespace gazebo
