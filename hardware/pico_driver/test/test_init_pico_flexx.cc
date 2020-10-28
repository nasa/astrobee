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

#include <config_reader/config_reader.h>

// Required for the test cases
#include <ff_msgs/FaultState.h>
#include <ff_util/ff_names.h>

// Required for the test framework
#include <gtest/gtest.h>
#include <glog/logging.h>

#include <ros/ros.h>

// Test initialization pico flexx
// These initialization tests make sure the node is spawned. To do so,
// it spawns the node and the system manager. The node uses the wrong folder
// for the config files on purpose, which it will make it unable to read the
// config files. Because the node can't read the config files it triggers the
// initialization anomaly to the system monitor. Is this anomaly is issued
// within 2 seconds, then the test passes.


bool test_done = false;
ff_msgs::FaultStateConstPtr fault_state;
int init_fault_id = -1;

void FaultStateCallback(ff_msgs::FaultStateConstPtr const& state) {
  fault_state = state;
}

void FaultCheckTimeoutCallback(ros::TimerEvent const& te) {
  bool found = false;
  for (unsigned int i = 0; i < fault_state->faults.size() && !found; i++) {
    if (static_cast<int>(fault_state->faults[i].id) == init_fault_id) {
      found = true;
    }
  }

  EXPECT_EQ(found, true);
  test_done = true;
}

TEST(init_pico_flexx, PicoFlexxInitFailed) {
  config_reader::ConfigReader fault_config;

  fault_config.AddFile("faults.config");

  // Read fault config file into lua
  if (!fault_config.ReadFiles()) {
    ROS_ERROR("Couldn't open faults.config!");
    // Cause test to fail
    EXPECT_EQ(2, 1);
    return;
  }

  if (!fault_config.CheckValExists("pico_driver")) {
    ROS_ERROR("Couldn't extract pico flexx faults. Needed for init fault id.");
    // Cause test to fail
    EXPECT_EQ(2, 1);
    return;
  }

  config_reader::ConfigReader::Table fault_table(&fault_config, "pico_driver");

  std::string fault_key;
  // Lua indices start at one
  for (int i = 1; i < (fault_table.GetSize() + 1); i++) {
    config_reader::ConfigReader::Table fault_entry(&fault_table, i);

    // Get fault key
    if (!fault_entry.GetStr("key", &fault_key)) {
      ROS_ERROR("Fault key at index %i not in pico flexx fault table.", i);
      // Cause test to fail
      EXPECT_EQ(2, 1);
      return;
    }

    if (fault_key == "INITIALIZATION_FAILED") {
      if (!fault_entry.GetInt("id", &init_fault_id)) {
        ROS_ERROR("Fault id for pico flexx init failed fault missing.");
        // Cause test to fail
        EXPECT_EQ(2, 1);
        return;
      }
      break;
    }
  }

  if (init_fault_id == -1) {
    ROS_ERROR("Unable to extract init fault id for pico flexx.");
    // Cause test to fail
    EXPECT_EQ(2, 1);
    return;
  }

  ros::NodeHandle nh;

  ros::Subscriber fault_state_sub = nh.subscribe<ff_msgs::FaultState>(
                                            TOPIC_MANAGEMENT_SYS_MONITOR_STATE,
                                            10,
                                            &FaultStateCallback);

  if (fault_state_sub.getNumPublishers() == 0) {
    ros::Duration(1.0).sleep();
  }

  // Start timer to check if the pico flexx fault has been thrown.
  ros::Timer fault_timer = nh.createTimer(ros::Duration(2),
                                          &FaultCheckTimeoutCallback,
                                          true,
                                          true);

  test_done = false;
  while (!test_done) {
    ros::spinOnce();
  }

  fault_state_sub.shutdown();
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  // Initialize the gtest framework
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  // Initialize ROS
  ros::init(argc,
            argv,
            "test_init_pico_flexx",
            ros::init_options::AnonymousName);

  // Run all test procedures
  return RUN_ALL_TESTS();
}
