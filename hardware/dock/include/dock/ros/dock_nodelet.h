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


#ifndef DOCK_ROS_DOCK_NODELET_H_
#define DOCK_ROS_DOCK_NODELET_H_

#include <config_reader/config_reader.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ff_hw_msgs/DockStateStamped.h>
#include <ff_hw_msgs/Undock.h>
#include <dock/dock.h>
#include <cstdint>
#include <memory>
#include <string>

namespace dock {
namespace ros {

struct Params {
  std::string bus;
  unsigned int loop_addr;
  unsigned int dock_addr;
  float interval;
  int max_tries;
};

class DockNodelet : public nodelet::Nodelet {
 public:
  virtual void onInit();

  bool OnService(::ff_hw_msgs::Undock::Request &req,     // NOLINT
                 ::ff_hw_msgs::Undock::Response &resp);  // NOLINT

  void OnTimer(::ros::TimerEvent const& tev);

 private:
  using DockPtr = std::unique_ptr<Dock>;

  bool ReadParams(Params *p);

  DockPtr dock_;
  std::uint32_t seq_;
  bool docked_;
  bool force_;
  int max_tries_;
  int num_tries_;

  ::config_reader::ConfigReader config_;
  ::ros::ServiceServer server_;
  ::ros::Publisher pub_;
  ::ros::Timer poll_timer_;
};

}  // namespace ros
}  // namespace dock

#endif  // DOCK_ROS_DOCK_NODELET_H_
