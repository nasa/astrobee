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


// IMU Sensor message
#include <sensor_msgs/msg/imu.hpp>

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// STL includes
#include <string>

namespace sensor_msgs {
typedef msg::Imu Imu;
}  // namespace sensor_msgs

namespace gazebo {

FF_DEFINE_LOGGER("gazebo_sensor_plugin_imu");

class GazeboSensorPluginImu : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginImu() :
    FreeFlyerSensorPlugin("epson_imu", "imu", true) {
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

  ~GazeboSensorPluginImu() {
    if (update_) {
      update_.reset();
    }
  }

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(NodeHandle& nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = sensor;
    if (!sensor_) {
      gzerr << "GazeboSensorPluginImu requires a valid sensor.\n";
      return;
    }
    imu_ = std::dynamic_pointer_cast<sensors::ImuSensor>(sensor_);
    if (!imu_) {
      gzerr << "GazeboSensorPluginImu requires an imu sensor.\n";
      return;
    }
    // Get the name of the link to which this node is attached
    msg_.header.frame_id = GetFrame();
  }

  // Only send IMU when we have the correct extrinsics
  void OnExtrinsicsReceived(NodeHandle& nh) {
    // Offer IMU messages to those which need them
    pub_ = FF_CREATE_PUBLISHER(nh, sensor_msgs::Imu, TOPIC_HARDWARE_IMU, 1);

    // Connect to the sensor update event.
    update_ = sensor_->ConnectUpdated(
      std::bind(&GazeboSensorPluginImu::UpdateCallback, this));
    sensor_->SetActive(true);
  }

  // Called on each sensor update event
  void UpdateCallback() {
    if (!sensor_->IsActive())
      return;
    msg_.header.stamp.sec = imu_->LastMeasurementTime().sec;
    msg_.header.stamp.nanosec = imu_->LastMeasurementTime().nsec;
    msg_.orientation.x = imu_->Orientation().X();
    msg_.orientation.y = imu_->Orientation().Y();
    msg_.orientation.z = imu_->Orientation().Z();
    msg_.orientation.w = imu_->Orientation().W();
    msg_.angular_velocity.x = imu_->AngularVelocity().X();
    msg_.angular_velocity.y = imu_->AngularVelocity().Y();
    msg_.angular_velocity.z = imu_->AngularVelocity().Z();
    msg_.linear_acceleration.x = imu_->LinearAcceleration().X();
    msg_.linear_acceleration.y = imu_->LinearAcceleration().Y();
    msg_.linear_acceleration.z = imu_->LinearAcceleration().Z();
    pub_->publish(msg_);
  }

 private:
  // ROS variables
  rclcpp::Publisher<sensor_msgs::Imu>::SharedPtr pub_;
  // Gazebo variables
  sensors::SensorPtr sensor_;
  sensors::ImuSensorPtr imu_;
  event::ConnectionPtr update_;
  // Intermediary data
  sensor_msgs::Imu msg_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginImu)

}   // namespace gazebo
