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


#ifndef EXECUTIVE_UTILS_SEQUENCER_SEQUENCER_H_
#define EXECUTIVE_UTILS_SEQUENCER_SEQUENCER_H_

#include <ros/console.h>
#include <ros/time.h>

#include <ff_msgs/AgentStateStamped.h>
#include <ff_msgs/ControlState.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/CompressedFile.h>
#include <ff_msgs/ControlFeedback.h>
#include <ff_msgs/PlanStatusStamped.h>

#include <geometry_msgs/InertiaStamped.h>

#include <jsonloader/command.h>
#include <jsonloader/plan.h>

#include <string>
#include <vector>

namespace sequencer {

enum class ItemType {
  NONE,
  COMMAND,
  SEGMENT
};

class Sequencer {
 public:
  Sequencer();

  // because unions can't even :/ OMG.
  ItemType CurrentType(bool reset_time = true) noexcept;
  ff_msgs::CommandStamped::Ptr CurrentCommand() noexcept;
  jsonloader::Segment CurrentSegment() noexcept;

  // give feedback about the end of the current item (command/segment)
  // this advances the current item if it is a successful ack.
  //
  // returns true if there are more command/segments in the plan.
  bool Feedback(ff_msgs::AckCompletedStatus const& ack) noexcept;

  // give feedback about an index in the current segment
  void Feedback(ff_msgs::ControlFeedback const& progress) noexcept;

  // get the current plan status
  ff_msgs::PlanStatusStamped const& plan_status() noexcept;

  // I can haz validity?
  bool valid() const noexcept;
  jsonloader::Plan const& plan() const noexcept;

  // does the current plan have what it takes?
  bool HaveInertia() const noexcept;
  bool HaveOperatingLimits() const noexcept;

  // get the goods
  geometry_msgs::InertiaStamped GetInertia() const noexcept;
  bool GetOperatingLimits(ff_msgs::AgentStateStamped &state) const noexcept;

 private:
  int AppendStatus(ff_msgs::Status const& msg) noexcept;

  void Reset() noexcept;

  friend bool LoadPlan(ff_msgs::CompressedFile::ConstPtr const& cf,
                       Sequencer * seq);

  bool valid_;

  jsonloader::Plan plan_;
  ff_msgs::PlanStatusStamped status_;

  // when we started the current item
  ros::Time start_;

  // milestone is a station or segment within a plan
  int current_milestone_;

  // command within a station -1 means none
  int current_command_;

  // which waypoint within a segment we are at, 0 means the first one
  int current_index_;

  // index in the plan status where the current station is located
  int station_idx_;
  int station_duration_;
};

// load a plan from a compressed file.
// returns true if everything be cool, otherwise not.
bool LoadPlan(ff_msgs::CompressedFile::ConstPtr const& cf, Sequencer *seq);

std::vector<ff_msgs::ControlState>
Segment2Trajectory(jsonloader::Segment const& segment);

}  // end namespace sequencer

#endif  // EXECUTIVE_UTILS_SEQUENCER_SEQUENCER_H_
