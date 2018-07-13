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
#include <ff_util/ff_nodelet.h>

// FSW actions, services, messages
#include <ff_hw_msgs/OpticalFlow.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Vector3Stamped.h>

// interface class
#include <speed_cam/speed_cam.h>

/**
 * \ingroup hw
 */
namespace speed_cam {

class SpeedCamNode : public ff_util::FreeFlyerNodelet {
 public:
  SpeedCamNode() : ff_util::FreeFlyerNodelet(NODE_SPEED_CAM), speed_cam_(
    std::bind(&SpeedCamNode::ImuCallback, this, std::placeholders::_1),
    std::bind(&SpeedCamNode::CameraImageCallback, this, std::placeholders::_1,
      std::placeholders::_2,  std::placeholders::_3),
    std::bind(&SpeedCamNode::OpticalFlowCallback, this, std::placeholders::_1),
    std::bind(&SpeedCamNode::SpeedCallback, this, std::placeholders::_1),
    std::bind(&SpeedCamNode::StatusCallback, this, std::placeholders::_1),
    std::bind(&SpeedCamNode::VersionCallback, this, std::placeholders::_1)) {}
  virtual ~SpeedCamNode() {}

 protected:
  // Called on flight software stack initialization
  virtual void Initialize(ros::NodeHandle *nh) {
    // Read the config file
    config_reader::ConfigReader config_params;
    config_params.AddFile("hw/speed_cam.config");
    if (!config_params.ReadFiles())
      return InitFault("Could get read the config file");

    // Read the device information from the config table
    config_reader::ConfigReader::Table devices;
    if (!config_params.GetTable("speed_cam", &devices))
      return InitFault("Could get speed_cam item in config file");

    // Iterate over all devices
    for (int i = 0; i < devices.GetSize(); i++) {
      config_reader::ConfigReader::Table device_info;
      if (!devices.GetTable(i + 1, &device_info))
        return InitFault("Could get row in table table");

      // Get the name of the device and check it matches the name of this node
      std::string name;
      if (!device_info.GetStr("name", &name))
        return InitFault("Could not find row 'name' in table");

      if (name == GetName()) {
        config_reader::ConfigReader::Table serial;
        if (!device_info.GetTable("serial", &serial))
          return InitFault("Could not find table 'serial' in table");
        std::string port;
        if (!serial.GetStr("port", &port))
          return InitFault("Could not read the serial port from the config");
        uint32_t baud;
        if (!serial.GetUInt("baud", &baud))
          return InitFault("Could not read the serial baud from the config");
        if (speed_cam_.Initialize(port, baud) != RESULT_SUCCESS)
          return InitFault("Could not initialize the serial device");

        /*
        double timesync_secs = -1.0;
        if (!device_info.GetReal("timesync_secs", &timesync_secs))
          FF_FATAL("Could not find row 'timesync_secs' in table");
        if (timesync_secs > 0) {
          timer_ = nh->createTimer(ros::Duration(timesync_secs),
            &SpeedCamNode::TimesyncCallback, this, false);
        }
        */

        // Setup message: Inertial measurement unit
        pub_imu_ = nh->advertise < sensor_msgs::Imu > (TOPIC_HARDWARE_SPEED_CAM_IMU, 1);
        pub_camera_image_ = nh->advertise < sensor_msgs::Image > (TOPIC_HARDWARE_SPEED_CAM_CAMERA_IMAGE, 1);
        pub_optical_flow_ = nh->advertise < ff_hw_msgs::OpticalFlow > (TOPIC_HARDWARE_SPEED_CAM_OPTICAL_FLOW, 1);
        pub_speed_ = nh->advertise < geometry_msgs::Vector3Stamped > (TOPIC_HARDWARE_SPEED_CAM_SPEED, 1);

        // Exit once found
        return;
      }
    }

    // If we get here, we didn't find a configuration block in the LUA, which is a problem...
    InitFault("Could not find the speed_cam: " + GetName());
  }

  // Deal with a fault in a responsible manner
  void InitFault(std::string const& msg ) {
    NODELET_ERROR_STREAM(msg);
    AssertFault(ff_util::INITIALIZATION_FAILED, msg);
    return;
  }

  // Perform a time sync event
  void TimesyncCallback(ros::TimerEvent const& event) {
    /*
    speed_cam_.TimeSync();
    */
  }

  // Callback from speedcam when new a new scaled IMU measurement is produced
  void ImuCallback(mavlink_raw_imu_t const& message) {
    if (pub_imu_.getNumSubscribers() == 0) return;
    // mavlink -> ros conversion
    static sensor_msgs::Imu imuMsg;
    // imuMsg.header.stamp =
    //     ros::Time(message.time_usec / 1000000, (message.time_usec % 1000000) * 1000);
    imuMsg.header.stamp = ros::Time::now();
    imuMsg.header.frame_id = GetName();
    imuMsg.linear_acceleration.x = static_cast<double>(message.xacc) * MILLIMSS_TO_MSS;
    imuMsg.linear_acceleration.y = static_cast<double>(message.yacc) * MILLIMSS_TO_MSS;
    imuMsg.linear_acceleration.z = static_cast<double>(message.zacc) * MILLIMSS_TO_MSS;
    imuMsg.angular_velocity.x = static_cast<double>(message.xgyro) * MILLIRADS_TO_RADS;
    imuMsg.angular_velocity.y = static_cast<double>(message.ygyro) * MILLIRADS_TO_RADS;
    imuMsg.angular_velocity.z = static_cast<double>(message.zgyro) * MILLIRADS_TO_RADS;
    pub_imu_.publish(imuMsg);
  }

  // Callback from speedcam when new a new optical flow measurement is produced
  void CameraImageCallback(std::vector<uint8_t> const& buffer, int32_t width, int32_t height) {
    if (pub_camera_image_.getNumSubscribers() == 0) return;
    // mavlink -> ros conversion
    static sensor_msgs::Image cameraImageMsg;
    cameraImageMsg.header.stamp = ros::Time::now();
    cameraImageMsg.header.frame_id = GetName();
    cameraImageMsg.height = height;
    cameraImageMsg.width = width;
    cameraImageMsg.encoding = sensor_msgs::image_encodings::MONO8;
    cameraImageMsg.is_bigendian = false;
    cameraImageMsg.step = width;
    cameraImageMsg.data.resize(buffer.size());
    memcpy(&cameraImageMsg.data[0], &buffer[0], buffer.size());
    pub_camera_image_.publish(cameraImageMsg);
  }

  // Callback from speedcam when new a new optical flow measurement is produced
  void OpticalFlowCallback(mavlink_optical_flow_t const& message) {
    if (pub_optical_flow_.getNumSubscribers() == 0) return;
    // mavlink -> ros conversion
    static ff_hw_msgs::OpticalFlow optFlowMsg;
//    optFlowMsg.header.stamp = ros::Time(message.time_usec / 1000000, (message.time_usec % 1000000) * 1000);
    optFlowMsg.header.stamp = ros::Time::now();
    optFlowMsg.header.frame_id = GetName();
    optFlowMsg.ground_distance = message.ground_distance;
    optFlowMsg.flow_x = message.flow_x;
    optFlowMsg.flow_y = message.flow_y;
    optFlowMsg.velocity_x = message.flow_comp_m_x;
    optFlowMsg.velocity_y = message.flow_comp_m_y;
    optFlowMsg.quality = message.quality;
    pub_optical_flow_.publish(optFlowMsg);
  }

  // Callback from speedcam when new a new twist estimate is produced
  void SpeedCallback(mavlink_vision_speed_estimate_t const& message) {
    if (pub_speed_.getNumSubscribers() == 0)
      return;
    static geometry_msgs::Vector3Stamped speedMsg;
//    speedMsg.header.stamp = ros::Time(message.usec / 1000000, (message.usec % 1000000) * 1000);
    speedMsg.header.stamp = ros::Time::now();
    speedMsg.header.frame_id = GetName();
    speedMsg.vector.x = message.x;
    speedMsg.vector.y = message.y;
    speedMsg.vector.z = message.z;
    pub_speed_.publish(speedMsg);
  }

  // Callback from speedcam when a new status message is produced.
  // The status includes over-speed alerts.
  void StatusCallback(mavlink_heartbeat_t const& message) {
    if ((message.system_status & STATUS_LINEAR_SPEED_LIMIT_EXCEEDED) != 0x00)
      AssertFault(ff_util::VELOCITY_TOO_HIGH,
                  "The linear speed exceeded the limit.");
    if ((message.system_status & STATUS_ANGULAR_SPEED_LIMIT_EXCEEDED) != 0x00)
      AssertFault(ff_util::VELOCITY_TOO_HIGH,
                  "The angular speed exceeded the limit.");
  }

  void VersionCallback(uint32_t sw_version) {
    // FIXME: Should it publish the version?
  }

 private:
  SpeedCam speed_cam_;                            // Speedcam driver interface
  ros::Publisher pub_imu_;                        // Imu publisher
  ros::Publisher pub_camera_image_;               // Camera image publisher
  ros::Publisher pub_optical_flow_;               // Optical flow publisher
  ros::Publisher pub_speed_;                      // Twist publisher
  ros::Timer timer_;                              // For time sync
};

PLUGINLIB_DECLARE_CLASS(speed_cam, SpeedCamNode,
                        speed_cam::SpeedCamNode, nodelet::Nodelet);

}  // namespace speed_cam
