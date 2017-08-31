// Copyright 2016 Intelligent RObotics Group, NASA ARC

#ifndef DISK_MONITOR_DISK_MONITOR_H_
#define DISK_MONITOR_DISK_MONITOR_H_

#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <errno.h>
#include <stdint.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/statvfs.h>

#include <config_reader/config_reader.h>
#include <ff_msgs/DiskState.h>
#include <ff_msgs/DiskStateStamped.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <memory>
#include <string>
#include <vector>

namespace disk_monitor {

class DiskMonitor : public ff_util::FreeFlyerNodelet {
 public:
  DiskMonitor();
  ~DiskMonitor();
  bool PathExists(std::string path);

 protected:
  virtual void Initialize(ros::NodeHandle *nh);
  bool ReadParams();

 private:
  void OnCheckTimer(ros::TimerEvent const& event);
  bool CheckAndPublish();

  config_reader::ConfigReader config_params_;

  ff_msgs::DiskStateStamped disk_stats_msg_;

  int pub_queue_size_, update_freq_;

  ros::Publisher pub_disk_stats_;
  ros::Timer stats_timer_;

  std::string pub_topic_disk_stats_, processor_name_;
  std::vector<std::string> paths_;
};

}  // namespace disk_monitor

#endif  // DISK_MONITOR_DISK_MONITOR_H_
