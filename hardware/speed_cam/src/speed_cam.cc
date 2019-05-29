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

#include <speed_cam/speed_cam.h>

namespace speed_cam {

// By default the arm is uninitialized
SpeedCam::SpeedCam(SpeedCamImuCallback cb_imu, SpeedCamCameraImageCallback cb_camera_image,
  SpeedCamOpticalFlowCallback cb_optical_flow, SpeedCamSpeedCallback cb_speed,
  SpeedCamStatusCallback cb_status, SpeedCamVersionCallback cb_version,
  SpeedCamStateCallback cb_state)
  : serial_(std::bind(&SpeedCam::ReadCallback, this,
      std::placeholders::_1, std::placeholders::_2))
  , cb_imu_(cb_imu)
  , cb_camera_image_(cb_camera_image)
  , cb_optical_flow_(cb_optical_flow)
  , cb_speed_(cb_speed)
  , cb_status_(cb_status)
  , cb_version_(cb_version)
  , cb_state_(cb_state)
  , system_id_(0)
  , comp_id_(0)
  , image_size_(0)
  , image_packets_(0)
  , image_payload_(0)
  , image_width_(0)
  , image_height_(0)
  , sw_version_(0) {
  serial_.SetTimeoutCallback(
    std::bind(&SpeedCam::TimeoutCallback, this), 1000);
}

// Initialize the serial port
SpeedCamResult SpeedCam::Initialize(std::string const& port, uint32_t baud) {
  // Check the serial opens and return failure if not
  if (!serial_.Open(port, baud))
    return RESULT_PORT_NOT_OPEN;
  // Success!
  return RESULT_SUCCESS;
}

// Sync time with the device
void SpeedCam::SetState(int32_t const& state) {
  // Pack th mavlink message
  mavlink_message_t msg;
  mavlink_msg_param_set_pack(system_id_, comp_id_, &msg, system_id_, comp_id_,
    "SPEED_CAM_STATE", static_cast<float>(state), MAVLINK_TYPE_FLOAT);
  uint8_t buf[256];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  if (len > 0)
    serial_.Write(buf, len);
}

// Callback with serial data
void SpeedCam::ReadCallback(const uint8_t* buf, size_t len) {
  mavlink_message_t message;
  mavlink_status_t status;
  for (size_t i = 0; i < len; i++) {
    bool msgReceived = mavlink_parse_char(MAVLINK_COMM_1, buf[i], &message, &status);
    if (msgReceived) {
      system_id_ = message.sysid;
      comp_id_ = message.compid;
      switch (message.msgid) {
      // IMU
      case MAVLINK_MSG_ID_RAW_IMU: {
        mavlink_raw_imu_t imu;
        mavlink_msg_raw_imu_decode(&message, &imu);
        if (cb_imu_)
          cb_imu_(imu);
        break;
        }
      // OPTICAL FLOW
      case MAVLINK_MSG_ID_OPTICAL_FLOW: {
        mavlink_optical_flow_t flow;
        mavlink_msg_optical_flow_decode(&message, &flow);
        if (cb_optical_flow_)
          cb_optical_flow_(flow);
        break;
        }
      // CAMERA IMAGE : HANDSHAKE
      case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE: {
        mavlink_data_transmission_handshake_t handshake;
        mavlink_msg_data_transmission_handshake_decode(&message, &handshake);
        image_size_ = handshake.size;
        image_packets_ = handshake.packets;
        image_payload_ = handshake.payload;
        image_width_ = handshake.width;
        image_height_ = handshake.height;
        if (image_buffer_.size() < image_size_)
          image_buffer_.resize(image_size_);
        break;
        }
      // CAMERA IMAGE : DATA
      case MAVLINK_MSG_ID_ENCAPSULATED_DATA: {
        if (image_size_ == 0 || image_packets_ == 0)
          break;
        mavlink_encapsulated_data_t img;
        mavlink_msg_encapsulated_data_decode(&message, &img);
        size_t seq = img.seqnr;
        size_t pos = seq * image_payload_;
        if (seq + 1 > image_packets_)
          break;
        size_t bytesToCopy = image_payload_;
        if (pos + image_payload_ >= image_size_)
           bytesToCopy = image_size_ - pos;
        memcpy(&image_buffer_[pos], img.data, bytesToCopy);
        if (seq + 1 == image_packets_) {
          if (cb_camera_image_)
            cb_camera_image_(image_buffer_, image_width_, image_height_);
        }
        break;
        }
      // SPEED ESTIMATE
      case MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE: {
        mavlink_vision_speed_estimate_t speed;
        mavlink_msg_vision_speed_estimate_decode(&message, &speed);
        if (cb_speed_)
          cb_speed_(speed);
        break;
        }
      // SYSTEM HEARTBEAT
      case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t status;
        mavlink_msg_heartbeat_decode(&message, &status);
        if (cb_status_)
          cb_status_(status);
        break;
        }
      // SOFTWARE VERSION
      case MAVLINK_MSG_ID_NAMED_VALUE_INT: {
        mavlink_named_value_int_t named_value;
        mavlink_msg_named_value_int_decode(&message, &named_value);
        std::string name{ named_value.name };
        if (!name.compare(0, 10, "SW_VERSION"))
          sw_version_ = static_cast<uint32_t>(named_value.value);
        if (cb_version_)
          cb_version_(sw_version_);
        break;
        }
      // SPEED CAM STATE
      case MAVLINK_MSG_ID_PARAM_VALUE: {
        mavlink_param_value_t param;
        mavlink_msg_param_value_decode(&message, &param);
        if (strcmp(param.param_id, "SPEED_CAM_STATE") == 0) {
          int32_t state = static_cast<int32_t>(param.param_value);
          if (cb_state_)
            cb_state_(state);
        }
        break;
        }
      }
    }
  }
}

// Callback with serial data
void SpeedCam::TimeoutCallback(void) {}

}  // namespace speed_cam
