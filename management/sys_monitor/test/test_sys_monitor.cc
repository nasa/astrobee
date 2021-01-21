// Copyright 2017 Intelligent Robotics Group, NASA ARC

// Required for the test framework
#include <gtest/gtest.h>
#include <glog/logging.h>

// Required for thhe test cases
#include <ros/ros.h>
#include <config_reader/config_reader.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/Heartbeat.h>
#include <ff_msgs/Fault.h>
#include <ff_util/ff_names.h>

#include <map>
#include <string>

// Test system monitor

ros::Subscriber cmd_sub;
ros::Publisher hb_pub;

struct FaultStruct {
  std::string node_name;
  bool warning;
  bool blocking;
  std::string response;
};

std::map<unsigned int, std::shared_ptr<FaultStruct>> all_faults_;
unsigned int max_fault_id = 0;

bool ReadParams() {
  return true;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  // Initialize the gtesttest framework
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  // Initialize ROS
  ros::init(argc, argv, "test_sys_monitor", ros::init_options::AnonymousName);

  // Run all test procedures
  return RUN_ALL_TESTS();
}
