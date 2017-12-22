// Copyright 2017 Intelligent Robotics Group, NASA ARC

#include <ros/ros.h>

#include <config_reader/config_reader.h>
#include <ff_msgs/Heartbeat.h>
#include <ff_msgs/Fault.h>

#include <map>
#include <string>

config_reader::ConfigReader config_params;

ros::Publisher heartbeat_publisher;
ros::Subscriber heartbeat_subscriber;
ros::Timer reload_params_timer;

std::map<unsigned int, std::string> faults;

void HeartbeatCallback(ff_msgs::HeartbeatConstPtr const& Heartbeat) {
}

bool ReadParams() {
  std::string node_name;
  unsigned int fault_id;
  int i, j, k;

  // Read config files into lua
  if (!config_params.ReadFiles()) {
    ROS_FATAL("Fault Tester: error reading config files.");
    return false;
  }

  // Read in all faults
  config_reader::ConfigReader::Table subsystems_tbl(&config_params,
                                                                  "subsystems");

  // Go through all the subsystems
  int subsystems_tbl_size = subsystems_tbl.GetSize() + 1;
  // Lua indices start at one
  for (i = 1; i < subsystems_tbl_size; i++) {
    config_reader::ConfigReader::Table subsystem_tbl(&subsystems_tbl, i);

    config_reader::ConfigReader::Table nodes_tbl(&subsystem_tbl, "nodes");
    // Go through all the nodes
    int nodes_tbl_size = nodes_tbl.GetSize() + 1;
    for (j = 1; j < nodes_tbl_size; j++) {
      config_reader::ConfigReader::Table node_tbl(&nodes_tbl, j);
      if (!node_tbl.GetStr("name", &node_name)) {
        ROS_FATAL("Fault Tester: Unable to read name at (%i, %i).", i, j);
        return false;
      }

      config_reader::ConfigReader::Table faults_tbl(&node_tbl, "faults");
      // Go through all faults
      int faults_tbl_size = faults_tbl.GetSize() + 1;
      // Lua indices start at one
      for (k = 1; k < faults_tbl_size; k++) {
        config_reader::ConfigReader::Table fault_entry(&faults_tbl, k);
        if (!fault_entry.GetUInt("id", &fault_id)) {
          ROS_FATAL("Fault Tester Unable to read fault at %i, %i, %i", i, j, k);
          return false;
        }
        faults[fault_id] = node_name;
      }
    }
  }

  // TODO(Katie) Output faults and make sure they are correct

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fault_tester");
  ros::NodeHandle nh;

  // Fault table config is not in the standard config dir, need to set the
  // config dir to the right location.
  std::string path = common::GetConfigDir();
  path += "/management/";
  config_params.SetPath(path.c_str());

  // Add config files to config reader
  config_params.AddFile("fault_table.config");

  if (!ReadParams()) {
    exit(EXIT_FAILURE);
    return -1;
  }

  // Create a callback timer which checks to see if the config files have been
  // changed.
  /*reload_params_timer = nh.createTimer(ros::Duration(1),
      [this](ros::TimerEvent e) {
      config_params.CheckFilesUpdated(std::bind(&ReadParams, this)); },
      false, true);*/

  ros::spin();
  return 0;
}
