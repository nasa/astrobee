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


#include <dock_sim/dock_sim.h>

namespace dock_sim {

DockSim::DockSim() {}

DockSim::~DockSim() {}

// TODO(Someone) Create a simulated dock and add code here to actually dock
void DockSim::DockGoalCallback(ff_msgs::DockGoalConstPtr const& goal) {
  ff_msgs::DockFeedback feedback;
  ff_msgs::DockResult result;
  switch (goal->command) {
  // DOCK //
  case ff_msgs::DockGoal::DOCK:
    feedback.state.state =
      ff_msgs::DockState::DOCKING_SWITCHING_TO_AR_LOC;
    sas_dock_->publishFeedback(feedback);
    ros::Duration(1).sleep();
    feedback.state.state =
      ff_msgs::DockState::DOCKING_MOVING_TO_APPROACH_POSE;
    sas_dock_->publishFeedback(feedback);
    ros::Duration(5).sleep();
    feedback.state.state =
      ff_msgs::DockState::DOCKING_MOVING_TO_COMPLETE_POSE;
    sas_dock_->publishFeedback(feedback);
    ros::Duration(5).sleep();
    feedback.state.state =
      ff_msgs::DockState::DOCKING_CHECKING_ATTACHED;
    sas_dock_->publishFeedback(feedback);
    ros::Duration(1).sleep();
    feedback.state.state =
      ff_msgs::DockState::DOCKING_WAITING_FOR_SPIN_DOWN;
    sas_dock_->publishFeedback(feedback);
    ros::Duration(1).sleep();
    feedback.state.state =
      ff_msgs::DockState::DOCKING_SWITCHING_TO_NO_LOC;
    sas_dock_->publishFeedback(feedback);
    ros::Duration(1).sleep();
    result.response =
      ff_msgs::DockResult::DOCKED;
    sas_dock_->setSucceeded(result);
    break;
  // UNDOCK //
  case  ff_msgs::DockGoal::UNDOCK:
    feedback.state.state =
      ff_msgs::DockState::UNDOCKING_SWITCHING_TO_ML_LOC;
    sas_dock_->publishFeedback(feedback);
    ros::Duration(1).sleep();
    feedback.state.state =
      ff_msgs::DockState::UNDOCKING_WAITING_FOR_SPIN_UP;
    sas_dock_->publishFeedback(feedback);
    ros::Duration(1).sleep();
    feedback.state.state =
      ff_msgs::DockState::UNDOCKING_MOVING_TO_APPROACH_POSE;
    sas_dock_->publishFeedback(feedback);
    ros::Duration(5).sleep();
    result.response =
      ff_msgs::DockResult::UNDOCKED;
    sas_dock_->setSucceeded(result);
    break;
  }
}

void DockSim::onInit() {
  ros::NodeHandle nh = getNodeHandle();
  sas_dock_ =
    std::make_shared<actionlib::SimpleActionServer<ff_msgs::DockAction>>
      (nh, ACTION_BEHAVIORS_DOCK,
       boost::bind(&DockSim::DockGoalCallback, this, _1),
       false);
  sas_dock_->start();
}
}  // namespace dock_sim

PLUGINLIB_EXPORT_CLASS(dock_sim::DockSim, nodelet::Nodelet)
