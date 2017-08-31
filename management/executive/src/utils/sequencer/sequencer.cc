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


#include <executive/utils/sequencer/sequencer.h>
#include <executive/utils/sequencer/command_conversion.h>
#include <executive/utils/sequencer/plan_io.h>
#include <executive/utils/sequencer/sv/pose_vel_accel.h>

#include <msg_conversions/msg_conversions.h>

#include <jsonloader/planio.h>

#include <ff_msgs/CommandConstants.h>

#include <cstdint>
#include <unordered_map>

namespace sequencer {

namespace {

constexpr int  kMaxHistory = 64;

// TODO(tfmorse): double check that the plan's waypoint type is valid
// until then, comment out the following for BRIAN.
/*
const char * kWaypointTypes[] = {
  "ControlValues20", "PoseVelAccel"
};
*/

ff_msgs::ControlState
Waypoint2TrajectoryPoint(jsonloader::Waypoint const& w) {
  ff_msgs::ControlState p;
  sv::pose_vel_accel::StateVector sv;
  sv.vec = w.cwaypoint();

  p.pose.position = msg_conversions::eigen_to_ros_point(
    sv::pose_vel_accel::GetPosition(sv.vec).cast<double>());
  p.pose.orientation = msg_conversions::eigen_to_ros_quat(
    sv::pose_vel_accel::GetQuaternion(sv.vec).cast<double>());

  p.twist.linear = msg_conversions::eigen_to_ros_vector(
    sv::pose_vel_accel::GetVelocity(sv.vec).cast<double>());
  p.twist.angular = msg_conversions::eigen_to_ros_vector(
    sv::pose_vel_accel::GetAngularVelocity(sv.vec).cast<double>());

  p.accel.linear = msg_conversions::eigen_to_ros_vector(
    sv::pose_vel_accel::GetAcceleration(sv.vec).cast<double>());
  p.accel.angular = msg_conversions::eigen_to_ros_vector(
    sv::pose_vel_accel::GetAngularAcceleration(sv.vec).cast<double>());

  p.when.nsec = w.ctime().nsec();
  p.when.sec = w.ctime().sec();
  return p;
}

inline ff_msgs::Status MakeStatus(std::uint8_t status, int point, int command, int duration) {
  ff_msgs::Status s;
  s.status.status = status;
  s.point = point;
  s.command = command;
  s.duration = duration;
  return s;
}

}  // namespace

bool LoadPlan(ff_msgs::CompressedFile::ConstPtr const& cf, Sequencer *seq) {
  std::string out;

  // sanity check
  if (seq == nullptr)
    return false;

  bool ret = sequencer::DecompressData(
                                reinterpret_cast<const char*>(cf->file.data()),
                                cf->file.size(), cf->type, &out);
  if (!ret) {
    return false;  // default is empty & non-valid
  }

  jsonloader::Plan plan(jsonloader::LoadPlan(out));
  if (!plan.valid()) {
    return false;
  }

  // TODO(tfmorse): double check that the waypint type is valid

  seq->Reset();
  seq->plan_ = std::move(plan);
  seq->status_.name = seq->plan_.name();
  seq->valid_ = true;
  seq->status_.history.clear();
  return true;
}

std::vector<ff_msgs::ControlState>
Segment2Trajectory(jsonloader::Segment const& seg) {
  std::vector<ff_msgs::ControlState> traj;
  traj.reserve(seg.waypoints().size());
  for (jsonloader::Waypoint const& wp : seg.waypoints()) {
    traj.push_back(Waypoint2TrajectoryPoint(wp));
  }
  return traj;
}

Sequencer::Sequencer()
  : valid_(false), start_(0, 0),
    current_milestone_(0), current_command_(-1),
    current_index_(0), station_idx_(0) {
}

void Sequencer::Reset() noexcept {
  valid_ = false;
  start_ = { 0, 0 };
  current_milestone_ = 0;
  current_command_ = -1;
  current_index_ = 0;
  station_idx_ = 0;
}

jsonloader::Plan const& Sequencer::plan() const noexcept {
  return plan_;
}

ItemType Sequencer::CurrentType(bool reset_time) noexcept {
  if (!valid_)
    return ItemType::NONE;

  // whyyyy? I returned false, bro!
  if (current_milestone_ < 0 ||
      static_cast<std::size_t>(current_milestone_) >= plan_.NumMilestones())
    return ItemType::NONE;

  // CUZ WE'RE SPECIAL SNOWFLAKES, YO.
  // but srsly, we have to skip over the first station if there are no commands
  // in it and append a fake "completed" status for the ground. since this is
  // the first entry into this class, here is where we do it.
  if (current_milestone_ == 0) {
    jsonloader::Milestone const& m = plan_.GetMilestone(current_milestone_);
    jsonloader::Station const& s = dynamic_cast<jsonloader::Station const&>(m);
    if (s.commands().size() == 0) {
      AppendStatus(MakeStatus(ff_msgs::AckCompletedStatus::OK, 0, -1, 0));
      current_milestone_++;
    } else {
      station_idx_ =
        AppendStatus(MakeStatus(ff_msgs::AckCompletedStatus::NOT, 0, -1, 0));
      if (current_command_ <= 0)
        current_command_ = 0;
    }
  }

  // a one-station plan with no commands in the station. (╯°□°）╯︵ ┻━┻
  if (static_cast<std::size_t>(current_milestone_) >= plan_.NumMilestones())
    return ItemType::NONE;

  // http://imgur.com/vGyMAnB
  if (reset_time)
    start_ = ros::Time::now();

  if (plan_.GetMilestone(current_milestone_).IsSegment())
    return ItemType::SEGMENT;

  // IN FEEDBACK WE TRUST (and tequila)
  return ItemType::COMMAND;
}

jsonloader::Segment Sequencer::CurrentSegment() noexcept {
  jsonloader::Milestone const& m = plan_.GetMilestone(current_milestone_);
  if (!m.IsSegment()) {
    ROS_WARN("requesting a segment, but current milestone is not a segment.");
    return jsonloader::Segment();
  }

  // TODO(tfmorse) bump waypoints for feedback waypoint index, maybe?

  return dynamic_cast<jsonloader::Segment const&>(m);
}

ff_msgs::CommandStamped::Ptr Sequencer::CurrentCommand() noexcept {
  ff_msgs::CommandStamped::Ptr cmd(new ff_msgs::CommandStamped());
  cmd->subsys_name = "INVALID";
  cmd->cmd_name = "INVALID";
  cmd->cmd_src = "plan";
  cmd->cmd_origin = "plan";
  cmd->args.clear();

  jsonloader::Milestone const& m = plan_.GetMilestone(current_milestone_);
  if (!m.IsStation()) {
    ROS_WARN("requesting a command, but current milestone is not a station.");
    cmd.reset();
    return cmd;
  }

  jsonloader::Station const& s = dynamic_cast<jsonloader::Station const&>(m);
  jsonloader::Command * plan_cmd = s.commands()[current_command_].get();

  auto it = internal::kCmdGenMap.find(plan_cmd->type());
  if (it == internal::kCmdGenMap.end()) {
    ROS_ERROR("do not know how to generate command %s",
              plan_cmd->type().data());
    cmd.reset();
    return cmd;
  }

  if (!it->second.fn(plan_cmd, cmd.get())) {
    ROS_ERROR("error convertiong command %s",
              plan_cmd->type().data());
    cmd.reset();
    return cmd;
  }

  // success. 성공. erfolg. успех
  cmd->subsys_name = it->second.subsystem;
  cmd->cmd_name = it->second.name;
  return cmd;
}

bool Sequencer::Feedback(ff_msgs::AckCompletedStatus const& ack) noexcept {
  // WE DUN!
  ros::Time end = ros::Time::now();
  ros::Duration d = end - start_;

  // update the current thing's status
  AppendStatus(MakeStatus(ack.status, current_milestone_, current_command_, d.sec));

  // skip ahead
  jsonloader::Milestone const& m = plan_.GetMilestone(current_milestone_);
  if (m.IsSegment()) {
    current_milestone_++;
    current_command_ = -1;
    current_index_ = 0;
  } else {
    jsonloader::Station const& s = dynamic_cast<jsonloader::Station const&>(m);
    current_command_++;
    station_duration_ += d.sec;

    if (static_cast<std::size_t>(current_command_) < s.commands().size())
      return true;

    // update station status, since all the commands are done
    status_.history[station_idx_].duration = station_duration_;
    status_.history[station_idx_].status.status = ack.status;

    current_command_ = -1;
    current_milestone_++;
    station_idx_ = 0;
  }

  if (static_cast<std::size_t>(current_milestone_) >= plan_.NumMilestones())
    return false;

  // check for an empty station to skip
  jsonloader::Milestone const& nm = plan_.GetMilestone(current_milestone_);
  if (nm.IsSegment()) {
    return true;
  }

  jsonloader::Station const& s = dynamic_cast<jsonloader::Station const&>(nm);
  if (s.commands().size() > 0) {
    current_command_ = 0;

    // log a temporary entry for the current station, keep track of the index
    station_idx_ =
        AppendStatus(  // feel like lisp yet?
            MakeStatus(ff_msgs::AckCompletedStatus::NOT,
                       current_milestone_, -1, 0));
    station_duration_ = 0;
    return true;
  }

  // update plan status for the skipped station, CUZ WE ON FIAH
  AppendStatus(MakeStatus(ff_msgs::AckCompletedStatus::OK,
                          current_milestone_, -1, 0));

  current_milestone_++;
  return static_cast<std::size_t>(current_milestone_) < plan_.NumMilestones();
}

void Sequencer::Feedback(ff_msgs::ControlProgress const& progress) noexcept {
  current_index_ = progress.index;  // ain't no thang
}

int Sequencer::AppendStatus(ff_msgs::Status const& msg) noexcept {
  status_.history.push_back(msg);
  if (status_.history.size() > kMaxHistory)
    status_.history.erase(status_.history.begin());
  // return index to pushed item
  return status_.history.size() - 1;
}

bool Sequencer::valid() const noexcept {
  return valid_;
}

ff_msgs::PlanStatusStamped const& Sequencer::plan_status() noexcept {
  status_.point = current_milestone_;
  status_.command = current_command_;
  if (static_cast<std::size_t>(current_milestone_) >= plan_.NumMilestones()) {
    status_.status.status = ff_msgs::AckStatus::COMPLETED;
  } else {
    status_.status.status = ff_msgs::AckStatus::EXECUTING;
  }
  return status_;
}

}  // namespace sequencer
