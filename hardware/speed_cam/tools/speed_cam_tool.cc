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

// Speed camera interface
#include <speed_cam/speed_cam.h>

// Check file access ok
#include <unistd.h>

// C++ STL inclues
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <mutex>
#include <functional>

// Default argument values
#define DEFAULT_SERIAL_PORT   "/dev/ttyACM0"
#define DEFAULT_SERIAL_BAUD   115200

namespace speed_cam {

// Global data structures for asynchornous data caching and resource locking
static std::mutex mutex_imu_, mutex_camera_image_, mutex_optical_flow_, mutex_speed_;
static mavlink_raw_imu_t msg_imu_;
static std::vector < uint8_t > msg_camera_image_;
static int32_t msg_camera_image_height_;
static int32_t msg_camera_image_width_;
static mavlink_optical_flow_t msg_optical_flow_;
static mavlink_vision_speed_estimate_t msg_speed_;

// Called when a new IMU measurement is available
void ImuCallback(mavlink_raw_imu_t const& message) {
  std::lock_guard < std::mutex > guard(mutex_imu_);
  msg_imu_ = message;
}

// Called when a new camera image measurement is available
void CameraImageCallback(std::vector<uint8_t> const& buffer, int32_t width, int32_t height) {
  std::lock_guard < std::mutex > guard(mutex_camera_image_);
  msg_camera_image_ = buffer;
  msg_camera_image_height_ = height;
  msg_camera_image_width_ = width;
}

// Called when a new optical flow measurement is available
void OpticalFlowCallback(mavlink_optical_flow_t const& message) {
  std::lock_guard < std::mutex > guard(mutex_optical_flow_);
  msg_optical_flow_ = message;
}

// Called when a new speed estimate is available
void SpeedCallback(mavlink_vision_speed_estimate_t const& message) {
  std::lock_guard < std::mutex > guard(mutex_speed_);
  msg_speed_ = message;
}

// Print an error message and fail gracefully
bool Error(std::string const& msg) {
  std::cerr << "Error: " << msg << std::endl;
  return false;
}

// Grab an unsigned integer from user input with type and value checking
uint32_t InputUnsignedInteger(uint32_t min, uint32_t max) {
  uint32_t choice;
  while (true) {
    std::cout << std::endl << "Input choice > ";
    std::string input;
    getline(std::cin, input);
    std::stringstream ss(input);
    if (ss >> choice &&  choice >= min && choice <= max)
      return choice;
    std::cerr << "Number not in range [" << min << ":" << max << "], please try again" << std::endl;
  }
  std::cerr << "Got to an unreachable section of code" << std::endl;
  return 0;
}

// Grab a floating point number from user input with type and value checking
std::string InputString() {
  std::string choice;
  while (true) {
    std::cout << std::endl << "Input file name > ";
    std::string input;
    getline(std::cin, input);
    std::stringstream choice;
    choice << input;
    choice << ".pgm";
    if (access(choice.str().c_str(), W_OK))
      return choice.str();
    std::cerr << "File cannot be written, please try again" << std::endl;
  }
  std::cerr << "Got to an unreachable section of code" << std::endl;
  return "";
}

// Print a main menu
bool MainMenu(speed_cam::SpeedCam &interface) {
  // Print title
  std::cout << std::endl;
  std::cout << "Astrobee speed cam host test" << std::endl << std::endl;
  std::cout << std::endl;
  std::cout << "*******************************************************" << std::endl;
  std::cout << "************************* MENU ************************" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  std::cout << "0. Quit" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  std::cout << "1. Print latest IMU measurement" << std::endl;
  std::cout << "2. Print latest camera image" << std::endl;
  std::cout << "3. Print latest optical flow measurement" << std::endl;
  std::cout << "4. Print latest velocity estimate" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  std::cout << "5. Send a time sync event" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  // Keep looping until we have valid input
  uint8_t choice = InputUnsignedInteger(0, 5);
  // Do something based on the choice
  switch (choice) {
    case 0: {
      std::cout << "Goodbye." << std::endl;
      return false;
    }
    // Print IMU measurement
    case 1: {
      std::lock_guard < std::mutex > guard(mutex_imu_);
      std::cout << "Time : " << static_cast<double>(msg_imu_.time_usec) / 1e6 << std::endl;
      std::cout << "Acceleration (m/sec^2)" << std::endl;
      std::cout << "- X : " << static_cast<double>(msg_imu_.xacc) * MILLIG_TO_MPSECSQ << std::endl;
      std::cout << "- Y : " << static_cast<double>(msg_imu_.yacc) * MILLIG_TO_MPSECSQ << std::endl;
      std::cout << "- Z : " << static_cast<double>(msg_imu_.zacc) * MILLIG_TO_MPSECSQ << std::endl;
      std::cout << "Angular velocity (rads/sec)" << std::endl;
      std::cout << "- X : " << static_cast<double>(msg_imu_.xgyro) * MILLIRADS_TO_RADS << std::endl;
      std::cout << "- Y : " << static_cast<double>(msg_imu_.ygyro) * MILLIRADS_TO_RADS << std::endl;
      std::cout << "- Z : " << static_cast<double>(msg_imu_.zgyro) * MILLIRADS_TO_RADS << std::endl;
      return true;
    }
    // Save raw camera image to file
    case 2: {
      std::lock_guard < std::mutex > guard(mutex_camera_image_);
      std::cout << "Please provide an extension-less file name" << std::endl;
      std::string fname = InputString();
      std::ofstream outfile(fname, std::ios_base::out | std::ios_base::binary | std::ios_base::trunc);
      if (outfile.is_open()) {
        outfile << "P5" << std::endl;
        outfile << msg_camera_image_width_ << " " << msg_camera_image_height_ << std::endl;
        outfile << 255 << std::endl;
        outfile.write(reinterpret_cast < const char* > (msg_camera_image_.data()), msg_camera_image_.size());
        outfile.close();
        std::cout << "Image successfully written to " << fname
          << " (" << msg_camera_image_width_ << " x "<< msg_camera_image_height_<< ")" << std::endl;
      } else {
        std::cerr << "Image could not be written to " << fname << std::endl;
      }
      return true;
    }
    // Print optical flow info
    case 3: {
      std::lock_guard < std::mutex > guard(mutex_optical_flow_);
      std::cout << "Time : " << static_cast<double>(msg_optical_flow_.time_usec) / 1e6 << std::endl;
      std::cout << "Ground distance : " << msg_optical_flow_.ground_distance << std::endl;
      std::cout << "Flow X : " << msg_optical_flow_.flow_x << std::endl;
      std::cout << "Flow Y : " << msg_optical_flow_.flow_y << std::endl;
      std::cout << "Velocity X : " << msg_optical_flow_.flow_comp_m_x << std::endl;
      std::cout << "Velocity Y : " << msg_optical_flow_.flow_comp_m_y << std::endl;
      std::cout << "Quality : " << static_cast<int32_t>(msg_optical_flow_.quality) << std::endl;
      return true;
    }
    // Print velocity estimate
    case 4: {
      std::lock_guard < std::mutex > guard(mutex_speed_);
      std::cout << "Time : " << static_cast<double>(msg_speed_.usec) / 1e6 << std::endl;
      std::cout << "Velocity" << std::endl;
      std::cout << "- X : " << msg_speed_.x << std::endl;;
      std::cout << "- Y : " << msg_speed_.y << std::endl;;
      std::cout << "- Z : " << msg_speed_.z << std::endl;;
      return true;
    }
    // Tiem sync event
    case 5: {
      if (interface.TimeSync())
        std::cout << "Time synchronization event sent" << std::endl;
      else
        std::cerr << "Time synchronization event did not send" << std::endl;
      return true;
    }
    default: {
      std::cerr << "Invalid selection" << std::endl;
      return true;
    }
  }
  return true;
}

}  // namespace speed_cam

// Main entry point for application
int main(int argc, char *argv[]) {
  // Set default parameter values
  std::string port = DEFAULT_SERIAL_PORT;
  uint32_t baud = DEFAULT_SERIAL_BAUD;
  // Get command line parameters
  switch (argc) {
    case 3: baud = atoi(argv[2]);        // Baud
    case 2: port = argv[1];              // Port
    case 1: break;
    default:
      std::cout << "Usage: [device] [baud]" << std::endl;
      return 0;
  }

  // Create the callbacks
  speed_cam::SpeedCamImuCallback cb_imu =
    std::bind(speed_cam::ImuCallback, std::placeholders::_1);
  speed_cam::SpeedCamCameraImageCallback cb_camera_image =
    std::bind(speed_cam::CameraImageCallback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  speed_cam::SpeedCamOpticalFlowCallback cb_optical_flow =
    std::bind(speed_cam::OpticalFlowCallback, std::placeholders::_1);
  speed_cam::SpeedCamSpeedCallback cb_speed =
    std::bind(speed_cam::SpeedCallback, std::placeholders::_1);

  // Create the interface to the perching arm
  speed_cam::SpeedCam interface(cb_imu, cb_camera_image, cb_optical_flow, cb_speed);
  if (interface.Initialize(port, baud) != speed_cam::RESULT_SUCCESS) {
    speed_cam::Error("Could not open serial port");
    return 1;
  }

  // Keep taking commands until termination request
  while (speed_cam::MainMenu(interface)) {}

  // Make for great success
  return 0;
}
