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

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// IMU Sensor message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// STL includes
#include <string>

namespace gazebo {

class GazeboSensorPluginNavCam : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginNavCam() :
    FreeFlyerSensorPlugin("nav_cam", "nav_cam", true) {}

  ~GazeboSensorPluginNavCam() {}

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle *nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginNavCam requires a parent camera sensor.\n";
      return;
    }

    /*
    // If we are running a fast simulation, then turn down the rate
    double simulation_speed = 1.0;
    if (!nh->getParam("/simulation_speed", simulation_speed)) {
      gzwarn << "Sim speed not specified. Trying real-time." << std::endl;
    } else if (simulation_speed > 1.0) {
      sensor_->SetUpdateRate(0.0001);
    }
    */

    // Check that we have a mono camera
    if (sensor_->Camera()->ImageFormat() != "L8")
      ROS_FATAL_STREAM("Camera format must be L8");

    // Set image constants
    msg_.is_bigendian = false;
    msg_.header.frame_id = GetFrame();
    msg_.encoding = sensor_msgs::image_encodings::MONO8;

    // Create a publisher
    pub_img_ = nh->advertise<sensor_msgs::Image>(TOPIC_HARDWARE_NAV_CAM, 1,
      boost::bind(&GazeboSensorPluginNavCam::ToggleCallback, this),
      boost::bind(&GazeboSensorPluginNavCam::ToggleCallback, this));

    // Connect to the camera update event.
    connection_ = sensor_->ConnectUpdated(
      std::bind(&GazeboSensorPluginNavCam::UpdateCallback, this));
  }

  // Turn camera on or off based on topic subscription
  void ToggleCallback() {
    if (pub_img_.getNumSubscribers() > 0)
      sensor_->SetActive(true);
    else
      sensor_->SetActive(false);
  }

  // Called when a new image must be rendered
  void UpdateCallback() {
    if (!sensor_->IsActive() || !ExtrinsicsFound())
      return;
    // Construct and send image
    msg_.header.stamp.sec = sensor_->LastMeasurementTime().sec;
    msg_.header.stamp.nsec = sensor_->LastMeasurementTime().nsec;
    msg_.height = sensor_->ImageHeight();
    msg_.width = sensor_->ImageWidth();
    msg_.step = msg_.width;
    msg_.data.resize(msg_.step * msg_.height);
    std::copy(
      reinterpret_cast<const uint8_t*>(sensor_->ImageData()),
      reinterpret_cast<const uint8_t*>(sensor_->ImageData())
        + msg_.step * msg_.height, msg_.data.begin());
    pub_img_.publish(msg_);
  }

 private:
  sensor_msgs::Image msg_;
  ros::Publisher pub_img_;
  std::shared_ptr<sensors::WideAngleCameraSensor> sensor_;
  event::ConnectionPtr connection_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginNavCam)

}   // namespace gazebo
