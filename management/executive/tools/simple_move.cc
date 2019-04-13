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

#include <ff_msgs/CommandArg.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/AckCompletedStatus.h>
#include <ff_msgs/AckStamped.h>
#include <ff_msgs/AckStatus.h>
#include <ff_util/ff_names.h>

#include <string>

std::string unique_cmd_id;

void AckCallback(ff_msgs::AckStampedConstPtr const& Ack) {
  // Check if the ack corresponds to the command we sent
  if (Ack->cmd_id  == unique_cmd_id) {
    // Check if command hasn't completed
    if (Ack->completed_status.status == ff_msgs::AckCompletedStatus::NOT) {
      ROS_INFO("Move command is being executed and has not completed.");
      return;  // Return so we don't shut down prematurely
    } else if (Ack->completed_status.status == ff_msgs::AckCompletedStatus::OK) {
      // Command completed successfully
      ROS_INFO("Move command completed successfully!");
    } else {  // Command failed
      ROS_INFO("Move command failed! Message: %s", Ack->message.c_str());
    }
  }

  // Command finshed so exit
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_pub");
  ros::NodeHandle nh;

  if (argc <= 1) {
    ROS_ERROR("Error! Must provide x, y, and z as arguments!");
    return -1;
  }

  // Make publisher to publish the command
  ros::Publisher cmd_publisher;
  cmd_publisher = nh.advertise<ff_msgs::CommandStamped>(TOPIC_COMMAND, 10, true);

  // Make subscriber to receive command acks to see if the command completed
  // successfully
  ros::Subscriber ack_subscriber;
  ack_subscriber = nh.subscribe(TOPIC_MANAGEMENT_ACK, 10, &AckCallback);

  // Make ros command message to send to the executive
  ff_msgs::CommandStamped move_cmd;

  move_cmd.header.stamp = ros::Time::now();

  // Command names listed in CommandConstants.h
  move_cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF;

  // Command id needs to be a unique id that you will use make sure the command
  // was executed, usually a combination of username and timestamp
  unique_cmd_id = "guest_science" + std::to_string(move_cmd.header.stamp.sec);
  move_cmd.cmd_id = unique_cmd_id;

  // Source of the command, set to guest_science so that the system knows that
  // the command didn't come from the ground
  move_cmd.cmd_src = "guest_science";

  // Subsystem name, not used as of yet so I just set it to Astrobee
  move_cmd.subsys_name = "Astrobee";

  // Move command has 4 arguements; frame, xyz, xyz tolerance, and rotation
  move_cmd.args.resize(4);

  // Set frame to be world, although I don't believe frame is being used so you
  // can really set it to anything
  move_cmd.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  move_cmd.args[0].s = "world";

  // Set location where you want Astrobee to go to
  move_cmd.args[1].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
  move_cmd.args[1].vec3d[0] = atof(argv[1]);  // x
  move_cmd.args[1].vec3d[1] = atof(argv[2]);  // y
  move_cmd.args[1].vec3d[2] = atof(argv[3]);  // z (This axis may not currently work

  // Tolerance not used! If you want to set the tolerance, you need to use the
  // set operational limits command. I set tolerance to 0
  move_cmd.args[2].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
  move_cmd.args[2].vec3d[0] = 0;
  move_cmd.args[2].vec3d[1] = 0;
  move_cmd.args[2].vec3d[2] = 0;

  // Target attitude, quaternion, only the first 4 values are used
  move_cmd.args[3].data_type = ff_msgs::CommandArg::DATA_TYPE_MAT33f;
  move_cmd.args[3].mat33f[0] = 0;
  move_cmd.args[3].mat33f[1] = 0;
  move_cmd.args[3].mat33f[2] = 0;
  move_cmd.args[3].mat33f[3] = 1;
  move_cmd.args[3].mat33f[4] = 0;
  move_cmd.args[3].mat33f[5] = 0;
  move_cmd.args[3].mat33f[6] = 0;
  move_cmd.args[3].mat33f[7] = 0;
  move_cmd.args[3].mat33f[8] = 0;

  // Send command
  cmd_publisher.publish(move_cmd);

  ROS_INFO("waiting for executive to execute the command...");
  ros::spin();
  return 0;
}
