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

// Required for the test framework
#include <gtest/gtest.h>

// Required for the test cases
#include <ros/ros.h>

// Rquired messahes
#include <ff_msgs/VisualeyezData.h>
#include <ff_msgs/VisualeyezDataArray.h>

// Visualeyez interface
#include <visualeyez/visualeyez.h>

// The client generates synthetic UDP data
visualeyez::VisualeyezClient vz_client_;

// The synthetic data that will be sent
ff_msgs::VisualeyezDataArray msg_;

// Timer callback
void TimerCallback(const ros::TimerEvent& event) {
  vz_client_.Send(msg_);
}

// Timeout callback
void TimeoutCallback(const ros::TimerEvent& event) {
  EXPECT_FALSE(true);
  ros::shutdown();
}

// Simple callback
void VisualeyezCallback(const ff_msgs::VisualeyezDataArray::ConstPtr& msg) {
  // Perform tests on the received data
  EXPECT_FALSE(msg == nullptr);
  EXPECT_EQ(msg->header.stamp.toSec(), msg_.header.stamp.toSec());
  EXPECT_EQ(msg->measurements.size(), msg_.measurements.size());
  for (size_t i = 0; i < msg_.measurements.size(); i++) {
    EXPECT_EQ(msg->measurements[i].tcmid, msg_.measurements[i].tcmid);
    EXPECT_EQ(msg->measurements[i].ledid, msg_.measurements[i].ledid);
    EXPECT_EQ(msg->measurements[i].position.x, msg_.measurements[i].position.x);
    EXPECT_EQ(msg->measurements[i].position.y, msg_.measurements[i].position.y);
    EXPECT_EQ(msg->measurements[i].position.z, msg_.measurements[i].position.z);
  }
  // Shutdown ROS, for now :)
  ros::shutdown();
}

// Stream test - set up a fake client to send well-formed visualeyez messages at 1Hz,
// and ensure that they start being received within ten seconds.
TEST(test_visualeyez_stream, Stream) {
  // Get a node handle
  ros::NodeHandle nh("~");
  // Add some data to test
  msg_.header.stamp = ros::Time::now();
  ff_msgs::VisualeyezData data;
  data.tcmid = 1.0;
  data.ledid = 2.0;
  data.position.x = 3.0;
  data.position.y = 4.0;
  data.position.z = 5.0;
  msg_.measurements.push_back(data);
  // Subscribe to the visualeyez topic
  ros::Subscriber sub = nh.subscribe(TOPIC_VISUALEYEZ_DATA, 1, VisualeyezCallback);
  // Create a periodic timer to send data
  ros::Timer timer = nh.createTimer(ros::Duration(1.0), TimerCallback, false, true);
  // Create a periodic timer to send data
  ros::Timer timout = nh.createTimer(ros::Duration(10), TimeoutCallback, false, true);
  // Wait until shutdown
  ros::spin();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Initialize the gtesttest framework
  testing::InitGoogleTest(&argc, argv);
  // Initialize ROS
  ros::init(argc, argv, "test_visualeyez_stream", ros::init_options::AnonymousName);
  // Run all test procedures
  return RUN_ALL_TESTS();
}
