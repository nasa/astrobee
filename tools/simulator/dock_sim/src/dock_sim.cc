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
DockSim::DockSim() {
}

DockSim::~DockSim() {
}

// TODO(Someone) Create a simulated dock and add code here to actually dock
void DockSim::DockGoalCallback(ff_msgs::DockGoalConstPtr const& goal) {
  ff_msgs::DockFeedback feedback;
  ff_msgs::DockResult result;

  feedback.status = ff_msgs::DockFeedback::PREPARING;
  sas_dock_->publishFeedback(feedback);
  ros::Duration(1).sleep();

  feedback.status = ff_msgs::DockFeedback::MOVE_TO_START;
  sas_dock_->publishFeedback(feedback);
  ros::Duration(5).sleep();

  feedback.status = ff_msgs::DockFeedback::SWITCHING_CAMERA;
  sas_dock_->publishFeedback(feedback);
  ros::Duration(1).sleep();

  feedback.status = ff_msgs::DockFeedback::SWITCHING_MODE;
  sas_dock_->publishFeedback(feedback);
  ros::Duration(1).sleep();

  feedback.status = ff_msgs::DockFeedback::WAITING_FOR_LOC;
  sas_dock_->publishFeedback(feedback);
  ros::Duration(5).sleep();

  feedback.status = ff_msgs::DockFeedback::INGRESSING;
  sas_dock_->publishFeedback(feedback);
  ros::Duration(1).sleep();

  feedback.status = ff_msgs::DockFeedback::DEACTIVATING_PMC;
  sas_dock_->publishFeedback(feedback);
  ros::Duration(1).sleep();

  result.status = ff_msgs::DockResult::DOCKED;
  sas_dock_->setSucceeded(result);
}

// TODO(Someone) Create a simulated dock and add code here to actually undock
void DockSim::UndockGoalCallback(ff_msgs::UndockGoalConstPtr const& goal) {
  ff_msgs::UndockFeedback feedback;
  ff_msgs::UndockResult result;

  feedback.status = ff_msgs::UndockFeedback::SWITCHING_CAMERA;
  sas_undock_->publishFeedback(feedback);
  ros::Duration(1).sleep();

  feedback.status = ff_msgs::UndockFeedback::SWITCHING_MODE;
  sas_undock_->publishFeedback(feedback);
  ros::Duration(1).sleep();

  feedback.status = ff_msgs::UndockFeedback::WAITING_FOR_LOC;
  sas_undock_->publishFeedback(feedback);
  ros::Duration(5).sleep();

  feedback.status = ff_msgs::UndockFeedback::RESETTING_BIAS;
  sas_undock_->publishFeedback(feedback);
  ros::Duration(1).sleep();

  feedback.status = ff_msgs::UndockFeedback::ACTIVATING_PMC;
  sas_undock_->publishFeedback(feedback);
  ros::Duration(1).sleep();

  feedback.status = ff_msgs::UndockFeedback::ACTUATING_DOCK;
  sas_undock_->publishFeedback(feedback);
  ros::Duration(1).sleep();

  feedback.status = ff_msgs::UndockFeedback::EGRESSING;
  sas_undock_->publishFeedback(feedback);
  ros::Duration(5).sleep();

  result.status = ff_msgs::UndockResult::UNDOCKED;
  sas_undock_->setSucceeded(result);
}

void DockSim::onInit() {
  ros::NodeHandle nh = getNodeHandle();

  sas_dock_ =
      std::make_shared<actionlib::SimpleActionServer<ff_msgs::DockAction>>
                            (nh, ACTION_MOBILITY_DOCK,
                             boost::bind(&DockSim::DockGoalCallback, this, _1),
                             false);

  sas_undock_ =
      std::make_shared<actionlib::SimpleActionServer<ff_msgs::UndockAction>>
                          (nh, ACTION_MOBILITY_UNDOCK,
                           boost::bind(&DockSim::UndockGoalCallback, this, _1),
                           false);

  sas_dock_->start();
  sas_undock_->start();
}
}  // namespace dock_sim

PLUGINLIB_EXPORT_CLASS(dock_sim::DockSim, nodelet::Nodelet)
