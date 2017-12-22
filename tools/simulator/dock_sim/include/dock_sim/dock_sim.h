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


#ifndef DOCK_SIM_DOCK_SIM_H_
#define DOCK_SIM_DOCK_SIM_H_

#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <actionlib/server/simple_action_server.h>

#include <ff_msgs/DockAction.h>
#include <ff_util/ff_names.h>

#include <nodelet/nodelet.h>

#include <string>

namespace dock_sim {

class DockSim : public nodelet::Nodelet {
 public:
  DockSim();
  ~DockSim();

  void DockGoalCallback(ff_msgs::DockGoalConstPtr const& goal);

  virtual void onInit();

 private:
  std::shared_ptr<actionlib::SimpleActionServer<ff_msgs::DockAction>> sas_dock_;
};

}  // namespace dock_sim

#endif  // DOCK_SIM_DOCK_SIM_H_
