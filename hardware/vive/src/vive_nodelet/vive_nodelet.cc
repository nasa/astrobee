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
 *
 */


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
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// FSW nodelet
#include <ff_util/ff_nodelet.h>

// Standard messages
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

// Non-standard messages
#include <ff_hw_msgs/ViveButton.h>
#include <ff_hw_msgs/ViveLight.h>
#include <ff_hw_msgs/ViveLighthouses.h>
#include <ff_hw_msgs/ViveMotor.h>
#include <ff_hw_msgs/VivePulse.h>
#include <ff_hw_msgs/ViveSensor.h>
#include <ff_hw_msgs/ViveTrackers.h>

// Vive interface
extern "C" {
  #include <vive/vive.h>
}

// C++ includes
#include <cstdint>
#include <cmath>
#include <map>
#include <string>
#include <limits>
#include <thread>


/**
* \ingroup hw
*/
namespace vive {

class ViveNodelet : public ff_util::FreeFlyerNodelet {
 public:
  // Various constants used by the Vive system
  static constexpr double GRAVITY         = 9.80665;
  static constexpr double GYRO_SCALE      = 32.768;
  static constexpr double ACC_SCALE       = 4096.0;
  static constexpr double SWEEP_DURATION  = 400000.0;
  static constexpr double SWEEP_CENTER    = 200000.0;
  static constexpr double TICKS_PER_SEC   = 48e6;

  // Constructor - initialize vive
  ViveNodelet() : ff_util::FreeFlyerNodelet(NODE_VIVE, false) {
    kill_ = false;
    driver_ = vive_init();
  }

  // Destructor - make sure we free all vive driver types
  ~ViveNodelet() {
    kill_ = true;
    thread_.join();
    vive_kill(driver_);
  }

 protected:
  // Called when the node initializes
  void Initialize(ros::NodeHandle *nh) {
    // Latched publishers
    pub_lighthouses_ = nh->advertise<ff_hw_msgs::ViveLighthouses>(
      TOPIC_HARDWARE_VIVE_LIGHTHOUSES, 10, true);
    pub_trackers_ = nh->advertise<ff_hw_msgs::ViveTrackers>(
      TOPIC_HARDWARE_VIVE_TRACKERS, 10, true);
    // Non-latched publishers
    pub_light_ = nh->advertise<ff_hw_msgs::ViveLight>(
      TOPIC_HARDWARE_VIVE_LIGHT, 10);
    pub_button_ = nh->advertise<ff_hw_msgs::ViveButton>(
      TOPIC_HARDWARE_VIVE_BUTTON, 10);
    pub_imu_ = nh->advertise<sensor_msgs::Imu>(
      TOPIC_HARDWARE_VIVE_IMU, 10);
    // Install the callbacks
    vive_install_imu_fn(driver_, ImuCallback);
    vive_install_light_fn(driver_, LightCallback);
    vive_install_button_fn(driver_, ButtonCallback);
    vive_install_lighthouse_fn(driver_, LighthouseCallback);
    vive_install_tracker_fn(driver_, TrackerCallback);
    // Poll vive
    thread_ = std::thread(ViveNodelet::PollThread);
  }

  // Poll the driver
  static void PollThread() {
    while (!kill_)
      vive_poll(driver_);
  }

  // NB: The functions below are static to support callbacks from the C library

  // Callback to display light info
  static void LightCallback(tracker_t const * tracker, uint8_t lh, uint8_t axis,
    uint32_t synctime, uint16_t num_sensors, light_t const * measurement) {
    ff_hw_msgs::ViveLight msg;
    msg.header.frame_id = tracker->serial;
    msg.header.stamp = ros::Time::now();
    msg.lighthouse = lh;
    // Make sure we convert to RHS
    switch (axis) {
    case MOTOR_AXIS0: msg.axis = ff_hw_msgs::ViveMotor::AXIS_0; break;
    case MOTOR_AXIS1: msg.axis = ff_hw_msgs::ViveMotor::AXIS_1; break;
    default: ROS_WARN("Received light with invalid axis"); return;
    }
    // Add the pulses to the message
    msg.pulses.resize(num_sensors);
    for (uint16_t i = 0; i < num_sensors; i++) {
      msg.pulses[i].sensor = measurement[i].sensor_id;
      msg.pulses[i].angle = (M_PI / SWEEP_DURATION)
        * (static_cast<double>(measurement[i].timestamp));
      msg.pulses[i].duration =
        static_cast<double>(measurement[i].length) / TICKS_PER_SEC;
    }
    // Publish the data
    pub_light_.publish(msg);
  }

  // Called back when new IMU data is available
  static void ImuCallback(tracker_t const * t, imu_t const * measurement) {
    // Package up the IMU data
    sensor_msgs::Imu msg;
    msg.header.frame_id = t->serial;
    msg.header.stamp = ros::Time::now();
    msg.linear_acceleration.x =
      static_cast<double>(measurement->acc[0]) * GRAVITY / ACC_SCALE;
    msg.linear_acceleration.y =
      static_cast<double>(measurement->acc[1]) * GRAVITY / ACC_SCALE;
    msg.linear_acceleration.z =
      static_cast<double>(measurement->acc[2]) * GRAVITY / ACC_SCALE;
    msg.angular_velocity.x =
      static_cast<double>(measurement->gyr[0]) * (1./GYRO_SCALE) * (M_PI/180.);
    msg.angular_velocity.y =
      static_cast<double>(measurement->gyr[1]) * (1./GYRO_SCALE) * (M_PI/180.);
    msg.angular_velocity.z =
      static_cast<double>(measurement->gyr[2]) * (1./GYRO_SCALE) * (M_PI/180.);
    // Publish the data
    pub_imu_.publish(msg);
  }

  // Called when a button is pressed
  static void ButtonCallback(tracker_t const * t, button_t const * event) {
    // Package up the button data
    ff_hw_msgs::ViveButton msg;
    msg.tracker = t->serial;
    msg.mask = event->mask;
    msg.trigger_val = event->trigger;
    msg.pad_x = event->horizontal;
    msg.pad_y = event->vertical;
    // Publish the data
    pub_button_.publish(msg);
  }

  // Configuration call from the vive_tool
  static void TrackerCallback(tracker_t const * t) {
    ff_hw_msgs::ViveTracker & tracker = trackers_[t->serial];
    tracker.serial = t->serial;
    tracker.sensors.resize(t->cal.num_channels);
    for (size_t i = 0; i < t->cal.num_channels; i++) {
      Convert(t->cal.positions[i], tracker.sensors[i].position);
      Convert(t->cal.normals[i], tracker.sensors[i].normal);
    }
    // Set the IMU sensor calibration data
    Convert(t->cal.acc_bias, tracker.acc_bias);
    Convert(t->cal.acc_scale, tracker.acc_scale);
    Convert(t->cal.gyr_bias, tracker.gyr_bias);
    Convert(t->cal.gyr_scale, tracker.gyr_scale);
    // Set the default IMU transform
    Convert(&t->cal.imu_transform[0], tracker.pTi.rotation);
    Convert(&t->cal.imu_transform[4], tracker.pTi.translation);
    // Set the default IMU transform
    Convert(&t->cal.head_transform[0], tracker.pTh.rotation);
    Convert(&t->cal.head_transform[4], tracker.pTh.translation);
    // Send all trackers at once
    ff_hw_msgs::ViveTrackers msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    std::map<std::string, ff_hw_msgs::ViveTracker>::iterator it;
    for (it = trackers_.begin(); it != trackers_.end(); it++)
      msg.trackers.push_back(it->second);
    pub_trackers_.publish(msg);
  }

  // Configuration call from the vive_tool
  static void LighthouseCallback(lighthouse_t const * l) {
    ff_hw_msgs::ViveLighthouse & lighthouse = lighthouses_[l->serial];
    lighthouse.serial = l->serial;
    lighthouse.id = l->id;
    lighthouse.mode = l->mode_current;
    lighthouse.fault_mask = l->sys_faults;
    lighthouse.hw_version = l->hw_version;
    lighthouse.fw_version = l->fw_version;
    lighthouse.motors.resize(MAX_NUM_MOTORS);
    for (size_t i = 0; i < MAX_NUM_MOTORS; i++) {
      lighthouse.motors[i].axis = i;
      lighthouse.motors[i].phase = l->motors[i].phase;
      lighthouse.motors[i].tilt = l->motors[i].tilt;
      lighthouse.motors[i].gibphase = l->motors[i].gibphase;
      lighthouse.motors[i].gibmag = l->motors[i].gibmag;
      lighthouse.motors[i].curve = l->motors[i].curve;
    }
    // Get the lighthouse orientation
    Convert(l->accel, lighthouse.acceleration);
    // Send all lighthouses at once
    ff_hw_msgs::ViveLighthouses msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    std::map<std::string, ff_hw_msgs::ViveLighthouse>::iterator it;
    for (it = lighthouses_.begin(); it != lighthouses_.end(); it++)
      msg.lighthouses.push_back(it->second);
    pub_lighthouses_.publish(msg);
  }

  // UTILITY FUNCTIONS

  // Quaternion to double[4]
  template <typename T> inline
  static void Convert(geometry_msgs::Quaternion const& from, T to[4]) {
    to[0] = from.w;
    to[1] = from.x;
    to[2] = from.y;
    to[3] = from.z;
  }

  // double[4] to Quaternion
  template <typename T> inline
  static void Convert(const T from[4], geometry_msgs::Quaternion & to) {
    to.w = from[0];
    to.x = from[1];
    to.y = from[2];
    to.z = from[3];
  }

  // Vector to double[3]
  template <typename T> inline
  static void Convert(geometry_msgs::Vector3 const& from, T to[3]) {
    to[0] = from.x;
    to[1] = from.y;
    to[2] = from.z;
  }

  // double[3] to Vector
  template <typename T> inline
  static void Convert(const T from[3], geometry_msgs::Vector3 & to) {
    to.x = from[0];
    to.y = from[1];
    to.z = from[2];
  }

  // Point to double[3]
  template <typename T> inline
  static void Convert(geometry_msgs::Point const& from, T to[3]) {
    to[0] = from.x;
    to[1] = from.y;
    to[2] = from.z;
  }

  // double[3] to Point
  template <typename T> inline
  static void Convert(const T from[3], geometry_msgs::Point & to) {
    to.x = from[0];
    to.y = from[1];
    to.z = from[2];
  }

 private:
  static driver_t *driver_;
  static std::thread thread_;
  static bool kill_;
  static std::map<std::string, ff_hw_msgs::ViveTracker> trackers_;
  static std::map<std::string, ff_hw_msgs::ViveLighthouse> lighthouses_;
  static ros::Publisher pub_lighthouses_;
  static ros::Publisher pub_trackers_;
  static ros::Publisher pub_button_;
  static ros::Publisher pub_light_;
  static ros::Publisher pub_imu_;
};

// Statically allocated class types
driver_t *ViveNodelet::driver_;
std::thread ViveNodelet::thread_;
bool ViveNodelet::kill_;
std::map<std::string, ff_hw_msgs::ViveTracker> ViveNodelet::trackers_;
std::map<std::string, ff_hw_msgs::ViveLighthouse> ViveNodelet::lighthouses_;
ros::Publisher ViveNodelet::pub_lighthouses_;
ros::Publisher ViveNodelet::pub_trackers_;
ros::Publisher ViveNodelet::pub_button_;
ros::Publisher ViveNodelet::pub_light_;
ros::Publisher ViveNodelet::pub_imu_;

PLUGINLIB_EXPORT_CLASS(vive::ViveNodelet, nodelet::Nodelet);

}  // namespace vive
