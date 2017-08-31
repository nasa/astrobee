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

#include <perching_arm/perching_arm.h>

#include <iostream>
#include <sstream>
#include <string>
#include <chrono>
#include <atomic>
#include <thread>
#include <condition_variable>   // NOLINT

#define DEFAULT_SERIAL_PORT   "/dev/ttyUSB0"
#define DEFAULT_SERIAL_BAUD   115200

namespace perching_arm {

// Cache feedback from the arm
static PerchingArmFeedback feedback_;

// Print an error message and fail gracefully
bool Error(PerchingArmResult code, std::string const& msg) {
  std::string type;
  switch (code) {
  case RESULT_SUCCESS:            type = "SUCC"; break;  // Everything happened well
  case RESULT_PORT_NOT_OPEN:      type = "PNOP"; break;  // Serial port could not be opened
  case RESULT_COMMAND_REJECTED:   type = "CREJ"; break;  // Invalid command
  case RESULT_INVALID_COMMAND:    type = "CINV"; break;  // Invalid command
  case RESULT_PORT_WRITE_FAILURE: type = "PWRT"; break;  // Port not writeable
  case RESULT_PORT_INIT_FAILURE:  type = "PINT"; break;  // Port cannot be initialized
  case RESULT_FIRMWARE_ERROR:     type = "FWER"; break;  // Firmware error
  case RESULT_RESPONSE_TIMEOUT:   type = "TIMO"; break;  // Response timeout
  }
  std::cerr << "Error (" << type << ") : " << msg << std::endl;
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
float InputFloat(float min, float max) {
  float choice;
  while (true) {
    std::cout << std::endl << "Input choice > ";
    std::string input;
    getline(std::cin, input);
    std::stringstream ss(input);
    if (ss >> choice && choice >= min && choice <= max)
      return choice;
    std::cerr << "Number not in range [" << min << ":" << max << "], please try again" << std::endl;
  }
  std::cerr << "Got to an unreachable section of code" << std::endl;
  return 0.0f;
}

// For synchronization of the primary thread and serial callback
std::condition_variable cv_;
std::mutex mutex_;

// Wait for a timeout
void WaitForResultWithTimeout(uint32_t timeout) {
  std::cout << "Waiting for result" << std::endl;
  std::unique_lock < std::mutex > lk(mutex_);
  if (cv_.wait_for(lk, std::chrono::seconds(timeout)) == std::cv_status::no_timeout)
    return;
  Error(RESULT_RESPONSE_TIMEOUT, "Timeout Waiting for response from perching arm");
}

// Asynchronous callback to copy feedback
void FeedbackCallback(PerchingArmJointState joint_state,
                      PerchingArmGripperState gripper_state,
                      PerchingArmFeedback const& feedback) {
  feedback_ = feedback;
}

// Asynchronous callback
void EventCallback(PerchingArmEvent event, float percentage_complete) {
  // Error cases
  switch (event) {
  case EVENT_NONE:                   // Progress update
    return;
  case EVENT_PROGRESS:               // Progress update
    std::cout << "Progress: " << percentage_complete << std::endl;
    return;
  case EVENT_BACK_DRIVE:             // Back drive detected
    std::cout << "Back drive detected: " << std::endl;
    break;
  case EVENT_STOW_COMPLETE:          // Arm is now stowed
  case EVENT_MOVE_COMPLETE:          // Arm has finished moving
  case EVENT_PAN_COMPLETE:           // Arm has finished panning
  case EVENT_TILT_COMPLETE:          // Arm has finished tilting
  case EVENT_DEPLOY_COMPLETE:        // Arm is now deployed
  case EVENT_OPEN_COMPLETE:          // Gripper is now open
  case EVENT_CLOSE_COMPLETE:         // Gripper is now closed
  case EVENT_CALIBRATE_COMPLETE:     // Gripper is now calibrated
    std::cout << "Success!" << std::endl;
    break;
  case EVENT_ERROR:                  // Error encountered
    Error(RESULT_FIRMWARE_ERROR, "Driver reported a problem");
    break;
  }
  // Flush the stream before unblocking
  std::cout.flush();
  // Unblock the result wait
  cv_.notify_all();
}

// Asynchronous callback
void SleepMsCallback(uint32_t ms) {
  std::this_thread::sleep_for(std::chrono::duration<uint32_t, std::milli>(ms));
}

// Print a main menu
bool MainMenu(PerchingArm  &arm) {
  // Print title
  std::cout << std::endl;
  std::cout << "Astrobee perching arm host test" << std::endl << std::endl;
  std::cout << std::endl;
  arm.PrintStates();
  std::cout << std::endl;
  std::cout << "*******************************************************" << std::endl;
  std::cout << "************************* MENU ************************" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  std::cout << "0. Quit" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  std::cout << "1. Print feedback" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  std::cout << "2. Deploy" << std::endl;
  std::cout << "3. Pan the arm" << std::endl;
  std::cout << "4. Tilt the arm" << std::endl;
  std::cout << "5. Move (pan/tilt) the arm" << std::endl;
  std::cout << "6. Open gripper" << std::endl;
  std::cout << "7. Close gripper" << std::endl;
  std::cout << "8. Calibrate gripper" << std::endl;
  std::cout << "9. Run EMI test" << std::endl;
  std::cout << "10. Stow" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  std::cout << "11: Reset Controller Board (FTDI)" << std::endl;
  std::cout << "12: Reset Controller Board (SW)" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  std::cout << "13: Set Address Manually (refer to XM430 W210 datasheet)" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  // Keep looping until we have valid input
  uint8_t choice = InputUnsignedInteger(0, 13);
  // Response code
  PerchingArmResult ret;
  // Do something based on the choice
  switch (choice) {
    case 0: {
      std::cout << "Goodbye." << std::endl;
      return false;
    }
    case 1: {
      std::cout << "TILT MOTOR" << std::endl;
      std::cout << "- Position: "
                << PerchingArm::MOTOR_POSITION_SCALE * static_cast<float>(feedback_.tilt.position)
                << " degrees (raw: " << feedback_.tilt.position << ")" << std::endl;
      std::cout << "- Velocity: "
                << PerchingArm::MOTOR_VELOCITY_SCALE * static_cast<float>(feedback_.tilt.velocity)
                << " rpm (raw: " << feedback_.tilt.velocity << ")" << std::endl;
      std::cout << "- Load: "
                << PerchingArm::MOTOR_LOAD_SCALE * static_cast<float>(feedback_.tilt.load)
                << " mA (raw: " << feedback_.tilt.load << ")" << std::endl;
      std::cout << "PAN MOTOR" << std::endl;
      std::cout << "- Position: "
                << PerchingArm::MOTOR_POSITION_SCALE * static_cast<float>(feedback_.pan.position)
                << " degrees (raw: " << feedback_.pan.position << ")" << std::endl;
      std::cout << "- Velocity: "
                << PerchingArm::MOTOR_VELOCITY_SCALE * static_cast<float>(feedback_.pan.velocity)
                << " rpm (raw: " << feedback_.pan.velocity << ")" << std::endl;
      std::cout << "- Load: "
                << PerchingArm::MOTOR_LOAD_SCALE * static_cast<float>(feedback_.pan.load)
                << " mA (raw: " << feedback_.pan.load << ")" << std::endl;
      std::cout << "GRIPPER" << std::endl;
      std::cout << "- Position: "
                << PerchingArm::GRIPPER_POSITION_SCALE * static_cast<float>(feedback_.gripper.position)
                << " percent (raw: " << feedback_.gripper.position << ")" << std::endl;
      std::cout << "- Load: "
                << PerchingArm::GRIPPER_LOAD_SCALE * static_cast<float>(feedback_.gripper.load)
                << " mA (raw: " << feedback_.gripper.load << ")" << std::endl;
      std::cout << "BOARD" << std::endl;
      std::cout << "- 5V current: "
                << PerchingArm::CURRENT_5V_SCALE * static_cast<float>(feedback_.current_5v)
                << " percent (raw: " << feedback_.current_5v << ")" << std::endl;
      std::cout << "- 11V current "
                << PerchingArm::CURRENT_11V_SCALE * static_cast<float>(feedback_.current_11v)
                << " mA (raw: " << feedback_.current_11v << ")" << std::endl;
      std::cout << "- Temperature: "
                << PerchingArm::BOARD_TEMP_SCALE * static_cast<float>(feedback_.board_temp)
                << " degrees (raw: " << feedback_.board_temp << ")" << std::endl;
      std::cout << "- Loop time: "
                << PerchingArm::LOOP_TIME_SCALE * static_cast<float>(feedback_.loop_time)
                << " ms (raw: " << feedback_.loop_time << ")" << std::endl;
      return true;
    }
    case 2: {
      ret = arm.Deploy();
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not deploy the arm");
      WaitForResultWithTimeout(60);
      return true;
    }
    case 3: {
      std::cout << "Input pan value in degrees" << std::endl;
      float pan = InputFloat(PerchingArm::MIN_PAN, PerchingArm::MAX_PAN);
      ret = arm.Pan(pan);
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not pan the arm");
      WaitForResultWithTimeout(60);
      return true;
    }
    case 4: {
      std::cout << "Input tilt value in degrees" << std::endl;
      float tilt = InputFloat(PerchingArm::MIN_TILT, PerchingArm::MAX_TILT);
      ret = arm.Tilt(tilt);
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not tilt the arm");
      WaitForResultWithTimeout(60);
      return true;
    }
    case 5: {
      std::cout << "Input pan value in degrees" << std::endl;
      float pan = InputFloat(PerchingArm::MIN_PAN, PerchingArm::MAX_PAN);
      std::cout << "Input tilt value in degrees" << std::endl;
      float tilt = InputFloat(PerchingArm::MIN_TILT, PerchingArm::MAX_TILT);
      ret = arm.Move(pan, tilt);
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not pan/tilt the arm");
      WaitForResultWithTimeout(60);
      return true;
    }
    case 6: {
      ret = arm.Open();
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not open the gripper");
      WaitForResultWithTimeout(60);
      return true;
    }
    case 7: {
      ret = arm.Close();
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not close the gripper");
      WaitForResultWithTimeout(60);
      return true;
    }
    case 8: {
      ret = arm.Calibrate();
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not calibrate the gripper");
      WaitForResultWithTimeout(60);
      return true;
    }
    case 9: {
      ret = arm.Deploy();
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not deploy the arm");
      WaitForResultWithTimeout(60);
      ret = arm.Open();
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not open the gripper");
      WaitForResultWithTimeout(60);
      ret = arm.Close();
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not open the gripper");
      WaitForResultWithTimeout(60);
      ret = arm.Move(45.0f, PerchingArm::DEPLOY_TILT);
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not pan the arm");
      WaitForResultWithTimeout(60);
      ret = arm.Move(-45.0f, PerchingArm::DEPLOY_TILT);
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not pan the arm");
      WaitForResultWithTimeout(60);
      ret = arm.Stow();
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not stow the arm");
      WaitForResultWithTimeout(60);
      return true;
    }
    case 10: {
      ret = arm.Stow();
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not stow the arm");
      WaitForResultWithTimeout(60);
      return true;
    }
    case 11: {
      ret = arm.HardReset();
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not perform a hard reset");
      std::cout << "Success!";
      return true;
    }
    case 12: {
      ret = arm.SoftReset();
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not perform a soft reset");
      std::cout << "Success!";
      return true;
    }
    case 13: {
      std::cout << "> Input target: " << std::endl;
      uint16_t target = InputUnsignedInteger(0, 10);
      std::cout << "> Input address: " << std::endl;
      uint16_t address = InputUnsignedInteger(0, 255);
      std::cout << "> Input signed integer value: " << std::endl;
      int16_t value;
      std::cin >> value;
      ret = arm.SendCommand(target, address, value);
      if (ret != RESULT_SUCCESS)
        return Error(ret, "Could not send the command");
      std::cout << "Success!";
      return true;
    }
    default: {
      std::cerr << "Invalid selection" << std::endl;
      return true;
    }
  }
  return true;
}

}  // namespace perching_arm

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
  // Create the callback
  perching_arm::PerchingArmSleepMsCallback cb_sleep_ms = std::bind(perching_arm::SleepMsCallback,
    std::placeholders::_1);
  perching_arm::PerchingArmFeedbackCallback cb_feedback = std::bind(perching_arm::FeedbackCallback,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  perching_arm::PerchingArmEventCallback cb_event = std::bind(perching_arm::EventCallback,
    std::placeholders::_1, std::placeholders::_2);
  // Create the interface to the perching arm
  perching_arm::PerchingArm arm;
  if (arm.Initialize(port, baud, cb_sleep_ms, cb_event, cb_feedback) != perching_arm::RESULT_SUCCESS) {
    perching_arm::Error(perching_arm::RESULT_PORT_INIT_FAILURE, "Could not open serial port");
    return 1;
  }
  // Keep taking commands until termination request
  while (perching_arm::MainMenu(arm)) {}
  return 0;
}
