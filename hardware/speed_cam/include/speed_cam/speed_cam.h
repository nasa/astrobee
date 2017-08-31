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

#ifndef SPEED_CAM_SPEED_CAM_H_
#define SPEED_CAM_SPEED_CAM_H_

// Serial abstraction
#include <serial/serial.h>

// Mavlink protocol
#include <mavlink/v2.0/common/mavlink.h>

// C++ STL includes
#include <functional>
#include <string>
#include <vector>

// Conversions to SI units
#define MILLIG_TO_MPSECSQ 0.0098
#define MILLIRADS_TO_RADS 0.0010

/**
 * \ingroup hw
 */
namespace speed_cam {

// Callback types
typedef std::function<void(mavlink_raw_imu_t const&)> SpeedCamImuCallback;
typedef std::function<void(std::vector<uint8_t> const&, int32_t, int32_t)> SpeedCamCameraImageCallback;
typedef std::function<void(mavlink_optical_flow_t const&)> SpeedCamOpticalFlowCallback;
typedef std::function<void(mavlink_vision_speed_estimate_t const&)> SpeedCamSpeedCallback;

// Result for any speed camera action
enum SpeedCamResult {
  RESULT_SUCCESS,               // Everything happened well
  RESULT_PORT_NOT_OPEN,         // Serial port could not be opened
  RESULT_PORT_WRITE_FAILURE     // Couldn't write to the serial port
};

// Class to manage the speed camera
class SpeedCam {
 public:
  // Constructor
  SpeedCam(SpeedCamImuCallback cb_imu, SpeedCamCameraImageCallback cb_camera_image,
    SpeedCamOpticalFlowCallback cb_optical_flow, SpeedCamSpeedCallback cb_speed);

  // Initialize the serial port
  SpeedCamResult Initialize(std::string const& port, uint32_t baud);

  // Client-pushed time sync event
  bool TimeSync();

 protected:
  // Asynchronous callback with serial data
  void ReadCallback(const uint8_t *buffer, size_t len);

  // Asynchronous callback for read timeout
  void TimeoutCallback(void);

 private:
  serial::Serial serial_;                         // Serial port
  SpeedCamImuCallback cb_imu_;                    // Imu data callback
  SpeedCamCameraImageCallback cb_camera_image_;   // Camera image callback
  SpeedCamOpticalFlowCallback  cb_optical_flow_;  // Optical flow callback
  SpeedCamSpeedCallback cb_speed_;                // Twist callback
  int32_t system_id_;                             // Source device
  int32_t comp_id_;                               // Sink device
  size_t image_size_;                             // Image size
  size_t image_packets_;                          // Image packets
  int32_t image_payload_;                         // Payload size / transaction
  int32_t image_width_;                           // Image width
  int32_t image_height_;                          // Image height
  std::vector<uint8_t> image_buffer_;             // Image buffer
};

}  // namespace speed_cam

#endif  // SPEED_CAM_SPEED_CAM_H_
