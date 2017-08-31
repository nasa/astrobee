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

#include <ros/ros.h>

#include <ff_msgs/FamCommand.h>
#include <i2c/i2c_new.h>

#include <cerrno>
#include <cstring>
#include <string>

#define ROS_NODE_NAME "fam_cmd_i2c"

///////// ROS Parameters. See fam_cmd_i2c.yaml to change. /////////
// Frame ID
std::string frame_id_ = "fam_cmd_i2c";

// FAM command topic to listen to
std::string topic_fam_cmd_ = "/fam";

// I2C bus
std::string i2c_bus_file_ = "/dev/i2c-2";

// I2C address
int i2c_addr_ = 0x40;

// I2C retries.
int i2c_retries_ = 3;

// Scale
double scale_ = 65535.0;
///////////////////////////////////////////////////////////////////

i2c::Bus::Ptr i2c_bus_;

// Very simple checksum
uint8_t checksum(uint8_t *buf, size_t len) {
  uint8_t checksum = 0;

  for (size_t i = 0; i < len; i++)
    checksum ^= buf[i];

  return checksum;
}

bool Init(const ros::NodeHandle &nh) {
  nh.getParam("frame_id", frame_id_);
  nh.getParam("topic_fam_cmd", topic_fam_cmd_);
  nh.getParam("i2c_bus_file", i2c_bus_file_);
  nh.getParam("i2c_addr", i2c_addr_);
  nh.getParam("i2c_retries", i2c_retries_);

  return true;
}

uint16_t DoubleToUInt16(double value, double scale) {
  double t = value * scale;
  uint16_t ret;

  if (t < 0) {
    ROS_WARN("The value exceeds the min: %f", value);
    ret = 0;
  } else if (t > 65535) {
    ROS_WARN("The value exceeds the max: %f", value);
    ret = 65535;
  } else {
    ret = static_cast<uint16_t>(t);
  }

  return ret;
}

int16_t DoubleToInt16(double value, double scale) {
  double t = value * scale;
  int16_t ret;

  if (t < -32767) {
    ROS_WARN("The value exceeds the min: %f", value);
    ret = -32767;
  } else if (t > 32767) {
    ROS_WARN("The value exceeds the max: %f", value);
    ret = 32767;
  } else {
    ret = static_cast<int16_t>(t);
  }

  return ret;
}

void CommandCallback(const ff_msgs::FamCommand::ConstPtr& msg) {
  int16_t lin_acc_x = DoubleToInt16(msg->accel.x, scale_);
  int16_t lin_acc_y = DoubleToInt16(msg->accel.y, scale_);
  int16_t lin_acc_z = DoubleToInt16(msg->accel.z, scale_);
  int16_t ang_acc_x = DoubleToInt16(msg->alpha.x, scale_);
  int16_t ang_acc_y = DoubleToInt16(msg->alpha.y, scale_);
  int16_t ang_acc_z = DoubleToInt16(msg->alpha.z, scale_);
  uint16_t scale = DoubleToUInt16(scale_, 1.0);

  std::cout <<
    " lin_acc_x: " << lin_acc_x << "\t(" << msg->accel.x << ")\n" <<
    " lin_acc_y: " << lin_acc_y << "\t(" << msg->accel.y << ")\n" <<
    " lin_acc_z: " << lin_acc_z << "\t(" << msg->accel.z << ")\n" <<
    " ang_acc_x: " << ang_acc_x << "\t(" << msg->alpha.x << ")\n" <<
    " ang_acc_y: " << ang_acc_y << "\t(" << msg->alpha.y << ")\n" <<
    " ang_acc_z: " << ang_acc_z << "\t(" << msg->alpha.z << ")\n" <<
    " scale: " << scale << "\n" <<
    std::endl;

  i2c::Error err;

  auto i2c_device = i2c_bus_->DeviceAt(i2c_addr_);

  uint8_t data[15];

  data[0] = lin_acc_x >> 8;
  data[1] = lin_acc_x;
  data[2] = lin_acc_y >> 8;
  data[3] = lin_acc_y;
  data[4] = lin_acc_z >> 8;
  data[5] = lin_acc_z;
  data[6] = ang_acc_x >> 8;
  data[7] = ang_acc_x;
  data[8] = ang_acc_y >> 8;
  data[9] = ang_acc_y;
  data[10] = ang_acc_z >> 8;
  data[11] = ang_acc_z;
  data[12] = scale >> 8;
  data[13] = scale;
  data[14] = checksum(data, 14);

  for (int i = 0; i < 15; i++) {
    printf("data[%d] = 0x%02x\n", i, data[i]);
  }

  printf("\n");

  if (i2c_device.Write(data, 15, &err) != 15) {
    ROS_INFO("I2C write failed: %s", std::strerror(err));
  }
}

void Exit(int status) {
  ROS_INFO("Shutting down the node: (status = %d)", status);
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_NODE_NAME);
  // Private node handle
  ros::NodeHandle nh("~");

  if (!Init(nh)) {
    ROS_ERROR("Node initialization failed");
    Exit(EXIT_FAILURE);
  }

  ros::Subscriber sub_fam_cmd = nh.subscribe(topic_fam_cmd_, 1,
    &CommandCallback, ros::TransportHints().tcpNoDelay());

  i2c::Error err;

  i2c_bus_ = i2c::Open(i2c_bus_file_, &err);

  if (!i2c_bus_) {
    ROS_ERROR("Unable to open '%s': %s", i2c_bus_file_.c_str(),
      std::strerror(err));

    Exit(EXIT_FAILURE);
  }

  i2c_bus_->SetRetries(i2c_retries_);

  // Spin, waiting for messages to arrive.

  ros::spin();

  Exit(EXIT_SUCCESS);
}
