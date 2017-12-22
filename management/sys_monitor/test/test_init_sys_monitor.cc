// Copyright 2016 Intelligent Robotics Group, NASA ARC

// Required for the test framework
#include <gtest/gtest.h>
#include <glog/logging.h>

// Required for the test cases
#include <ros/ros.h>
#include <ff_msgs/AgentStateStamped.h>
#include <ff_util/ff_names.h>

bool test_done = false;

void AgentStateCallback(ff_msgs::AgentStateStampedConstPtr const& state) {
  EXPECT_EQ(state->operating_state.state, ff_msgs::OpState::READY);
  test_done = true;
}

TEST(init_sys_monitor, SysMonitorInitFailed) {
  ros::NodeHandle nh;

  ros::Subscriber agent_state_sub;

  agent_state_sub = nh.subscribe<ff_msgs::AgentStateStamped>(
                    TOPIC_MANAGEMENT_EXEC_AGENT_STATE, 10, &AgentStateCallback);

  if (agent_state_sub.getNumPublishers() == 0) {
    ros::Duration(1.0).sleep();
  }

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
