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

// Test initialization sys monitor
// These initialization tests make sure the system manager is correctly is
// spawned. To do so, the launch file contains uses the wrong folder
// for the faults.config config file on purpose. Because the system moniotr
// can't read the config files it triggers a faulty state. If this anomaly is issued
// within 2 seconds of initialization, then the test passes.

// Required for the test cases
#include <ff_msgs/AgentStateStamped.h>
#include <ff_common/ff_names.h>

// Required for the test framework
#include <gtest/gtest.h>
#include <glog/logging.h>

#include <ros/ros.h>

bool test_done = false;
ff_msgs::AgentStateStampedConstPtr agent_state;

void AgentStateCallback(ff_msgs::AgentStateStampedConstPtr const& state) {
  agent_state = state;
}

void FaultCheckTimeoutCallback(ros::TimerEvent const& te) {
  EXPECT_EQ(agent_state->operating_state.state, ff_msgs::OpState::FAULT);
  test_done = true;
}

TEST(init_sys_monitor, SysMonitorInitFailed) {
  ros::NodeHandle nh;

  ros::Subscriber agent_state_sub = nh.subscribe<ff_msgs::AgentStateStamped>(
                                              TOPIC_MANAGEMENT_EXEC_AGENT_STATE,
                                              10,
                                              &AgentStateCallback);

  if (agent_state_sub.getNumPublishers() == 0) {
    ros::Duration(1.0).sleep();
  }

  // Check if the sys_monitor initialization
  ros::Timer fault_timer = nh.createTimer(ros::Duration(2),
                                          &FaultCheckTimeoutCallback,
                                          true,
                                          true);

  test_done = false;

  while (!test_done) {
    ros::spinOnce();
  }

  agent_state_sub.shutdown();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Initialize the gtest framework
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  // Initialize ROS
  ros::init(argc,
            argv,
            "test_init_sys_monitor",
            ros::init_options::AnonymousName);

  // Run all test procedures
  return RUN_ALL_TESTS();
}
