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

#include "docking/docker.h"

#include <actionlib/client/simple_action_client.h>

#include <config_reader/config_reader.h>
#include <ff_util/config_client.h>
#include <msg_conversions/msg_conversions.h>

#include <ff_msgs/SetBool.h>
#include <ff_msgs/SetEkfInput.h>

#include <ff_util/ff_names.h>

#include <ff_hw_msgs/DockStateStamped.h>

#include <ff_msgs/MoveAction.h>


#include <deque>
#include <functional>
#include <thread>
#include <chrono>
#include <unordered_map>
#include <cmath>

namespace docking {

constexpr char kConfigName[] = "docker.config";

Docker::Docker()
  : sas_dock_(nh_, ACTION_MOBILITY_DOCK,
              boost::bind(&Docker::ExecDock, this, _1), false),
    sas_undock_(nh_, ACTION_MOBILITY_UNDOCK,
                boost::bind(&Docker::ExecUndock, this, _1), false),
    valid_(false),
    features_count_(kFeatureCountBufferSize),
    docked_valid_(false),
    docked_(false),
    have_goal_(false) {
  ROS_DEBUG("Docker::Docker()");

  config_.AddFile(kConfigName);
  if (!ReadConfig()) {
    ROS_ERROR("unable to read configuration");
    return;
  }

  // subscriptions
  sub_ekf_ = nh_.subscribe(TOPIC_GNC_EKF, 100, &Docker::OnEkfState, this);
  sub_dock_ = nh_.subscribe(TOPIC_MOBILITY_DOCKING_STATE, 10, &Docker::OnDockState, this);

  // start our action servers
  sas_dock_.start();
  sas_undock_.start();
  valid_ = true;
}

void Docker::OnEkfState(const ff_msgs::EkfState &ekf) {
  static size_t index = 0;
  nb_features_ = ekf.ml_count;
  ekf_confidence_ = ekf.confidence;
  /*ROS_WARN("update=%d, features=%d, confidence=%d",
           localization_updates_, nb_features_, ekf_confidence_);*/

  std::unique_lock<std::mutex> pose_lock(pose_mutex_);
  pose_ = ekf.pose;
  pose_valid_ = true;
  pose_lock.unlock();

  features_count_[index] = nb_features_;
  index++;
  if ( index >= kFeatureCountBufferSize ) {
    index = 0;
  }
  localization_updates_++;
}

void Docker::OnDockState(const ff_hw_msgs::DockStateStamped &state) {
  if (state.state == ff_hw_msgs::DockStateStamped::ERROR ||
      state.state == ff_hw_msgs::DockStateStamped::UNKNOWN) {
    docked_mutex_.lock();
    docked_valid_ = false;
    docked_mutex_.unlock();
    docked_cv_.notify_all();
    return;
  }

  docked_mutex_.lock();
  docked_ = (state.state == ff_hw_msgs::DockStateStamped::DOCKED);
  docked_valid_ = true;
  docked_mutex_.unlock();
  docked_cv_.notify_all();
}

bool Docker::EnoughFeatures(uint32_t minimum) {
  const double timeout = 10.0;
  ROS_INFO("check if there are enough features");
  localization_updates_ = 0;
  ros::Rate r(10);
  ros::Time start = ros::Time::now();
  double elapsed = 0;
  while ( localization_updates_ < kFeatureCountBufferSize && elapsed <= timeout ) {
    r.sleep();
    elapsed = ros::Time::now().toSec() - start.toSec();
  }
  if ( elapsed > timeout ) {
    ROS_WARN("Docker::enoughFeatures error");
    ROS_WARN("  did not get enough updates in the allocated time (%f)!",
             timeout);
    return false;
  }
  for ( size_t i = 0; i < features_count_.size(); i++ ) {
    if ( features_count_[i] < minimum ) {
      return false;
    }
  }
  return true;
}

namespace {

// Helper class that auto-unsets a passed boolean when it goes out of scope.
class AutoBool {
 public:
  explicit AutoBool(bool &b, bool set = true)
    : b_(b) {
    if (set) {
      ROS_INFO("Have a goal");
      b_ = true;
    }
  }

  ~AutoBool() {
    b_ = false;
    ROS_INFO("Have no goal anymore");
  }

  void Set(bool val = true) { b_ = val; }

 private:
  bool & b_;
};

// Class to help with cleaning up after a function fails. Think of it as a
// function destructor. (Do you feel sick yet?) (https://youtu.be/Q5UG7ISJfP0)
// Also supports manual execution of any cleanup functions pushed on the
// stack when you absolutely must be clean before returning.
class AutoCleaner {
  using Fn = std::function<void(void)>;
  using FnStack = std::deque<Fn>;

 public:
  AutoCleaner() {}
  ~AutoCleaner() {
    Cleanup();
  }

  void Add(Fn const& f) {
    fns_.push_front(f);
  }

  void Cleanup() {
    while (!fns_.empty()) {
      const Fn& f = fns_.front();
      try {
        f();
      } catch (std::exception &e) {
        // we ignore any exceptions. your problems are not my problems.
        ROS_ERROR("Caught exception running cleanup handler: %s", e.what());
      }
      fns_.pop_front();
    }
  }

  void Clear() {
    fns_.clear();
  }

 private:
  FnStack fns_;
};

using dr = ff_msgs::DockResult;
using df = ff_msgs::DockFeedback;

void UpdateHeader(std_msgs::Header &header, bool first = false) {
  header.frame_id = "world";
  header.seq = (first ? 1 : header.seq + 1);
  header.stamp = ros::Time::now();
}

std::unordered_map<std::int32_t, std::string> kDockStatString = {
  { dr::ACTION_IN_PROGRESS, "Undocking in progress" },
  { dr::ALREADY_DOCKED, "Already docked" },
  { dr::NO_CAMERA_SVC, "No camera enable service available" },
  { dr::CANNOT_ENABLE_CAM, "Unable to enable dock cam" },
  { dr::NO_EKF_SVC, "No EKF service to switch" },
  { dr::CANNOT_SWITCH_EKF, "Unable to switch EKF mode" },
  { dr::NO_MOBILITY_SVC, "No Mobility service available" },
  { dr::NO_PMC_SVC, "No PMC service available" },
  { dr::CANNOT_DISABLE_PMC, "Unable to disable PMCs" },
  { dr::CANCELED, "Canceled/Preempted" },
  { dr::DOCKED, "Docked successfully" }
};

using sas_func = std::function<void(dr const&, std::string const&)>;

double PoseDistance(geometry_msgs::Point const & p1,
                    geometry_msgs::Point const & p2) {
  return std::sqrt(std::pow(p1.x * p2.x, 2) +
                   std::pow(p1.y * p2.y, 2) +
                   std::pow(p1.z * p2.z, 2));
}

}  // namespace

void Docker::ExecDock(ff_msgs::DockGoal::ConstPtr const& goal) {
  ROS_INFO("Executing dock action");

  auto aborted = std::bind(
      &actionlib::SimpleActionServer<ff_msgs::DockAction>::setAborted,
      &sas_dock_, std::placeholders::_1, std::placeholders::_2);
  auto succeeded = std::bind(
      &actionlib::SimpleActionServer<ff_msgs::DockAction>::setSucceeded,
      &sas_dock_, std::placeholders::_1, std::placeholders::_2);
  auto preempted = std::bind(
      &actionlib::SimpleActionServer<ff_msgs::DockAction>::setPreempted,
      &sas_dock_, std::placeholders::_1, std::placeholders::_2);

  // "Destructor" for a failed dock action.
  AutoCleaner vacuum;

  auto finish = [this, &vacuum](std::uint32_t status, sas_func const &f) {
    vacuum.Cleanup();

    dr res;
    UpdateHeader(res.header, true);
    res.status = status;
    f(res, kDockStatString[status]);
  };

  if (have_goal_) {
    return finish(dr::ACTION_IN_PROGRESS, aborted);
  }

  // Auto-set and unset the have_goal_ boolean. This guarantees that we no
  // longer have an active goal when we exit this function.
  AutoBool auto_goal(have_goal_, true);

  ff_msgs::DockFeedback feedback;
  feedback.header.seq = 0;

  // helper lambda for sending feedback, since it's more than a 1-liner
  auto send_progress = [this, &feedback](std::uint32_t status){
    UpdateHeader(feedback.header);
    feedback.status = status;
    sas_dock_.publishFeedback(feedback);
  };

  // 0. Verify that we are not already docked
  {
    // TODO(tfmorse): add a timeout to the condition_var's wait
    std::unique_lock<std::mutex> lk(docked_mutex_);
    if (!docked_valid_) {
      docked_cv_.wait(lk, [this]{ return docked_valid_; });
    }

    if (docked_) {
      return finish(dr::ALREADY_DOCKED, succeeded);
    }
  }

  send_progress(df::PREPARING);

  // Actions to perform
  // 1. Connect and wait for our services
  ros::ServiceClient cam_client =
    nh_.serviceClient<ff_msgs::SetBool>(SERVICE_LOCALIZATION_AR_ENABLE);
  ros::ServiceClient ekf_client =
    nh_.serviceClient<ff_msgs::SetEkfInput>(SERVICE_GNC_EKF_SET_INPUT);
  ros::ServiceClient pmc_client =
    nh_.serviceClient<ff_msgs::SetBool>(SERVICE_HARDWARE_PMC_ENABLE);

  struct svc {
    ros::ServiceClient &client;
    ros::Duration timeout;
    std::int32_t err;
  } svcs[] = { { cam_client, ros::Duration(p_cam_timeout_), dr::NO_CAMERA_SVC },
               { ekf_client, ros::Duration(p_ekf_timeout_), dr::NO_EKF_SVC },
               { pmc_client, ros::Duration(p_pmc_timeout_), dr::NO_PMC_SVC } };
  const ros::Duration zero;
  for (svc & s : svcs) {
    bool found = false;
    ros::Duration d = s.timeout;
    while (d > zero) {
      // XXX: yeah, i know this doesn't work for timeouts with fractional
      // XXX: seconds. maybe i'll fix it... or not.
      if (s.client.waitForExistence(ros::Duration(1, 0))) {
        found = true;
        break;
      }
      d -= ros::Duration(1, 0);

      // check for cancelation
      if (sas_dock_.isPreemptRequested()) {
        return finish(dr::CANCELED, preempted);
      }
    }

    // we're having a bad day
    if (!found) {
      ROS_ERROR("error waiting for service");
      return finish(s.err, aborted);
    }
  }

  // 2. Move to approach point
  // TODO(tfmorse): later...

  if (sas_dock_.isPreemptRequested()) {
    return finish(dr::CANCELED, preempted);
  }
  send_progress(df::SWITCHING_CAMERA);

  // 1. Enable dockcam. technically should already be enabled
  {
    ff_msgs::SetBool::Request req;
    ff_msgs::SetBool::Response resp;

    req.enable = true;
    if (!cam_client.call(req, resp)) {
      return finish(dr::CANNOT_ENABLE_CAM, aborted);
    }

    if (!resp.success) {
      return finish(dr::CANNOT_ENABLE_CAM, aborted);
    }

    // if we fail, there is no cleanup, as we will *still* be disabled.
  }

  if (sas_dock_.isPreemptRequested()) {
    return finish(dr::CANCELED, preempted);
  }
  send_progress(df::SWITCHING_MODE);

  // 2. Switch to AR target mode
  {
    ff_msgs::SetEkfInput::Request req;
    ff_msgs::SetEkfInput::Response resp;
    req.mode = ff_msgs::SetEkfInput::Request::MODE_AR_TAGS;
    if (!ekf_client.call(req, resp)) {
      return finish(dr::CANNOT_SWITCH_EKF, aborted);
    }

    // no real response ¯\_(ツ)_/¯

    // Switch back if we fail or are pre-empted
    vacuum.Add([&ekf_client]{
      ff_msgs::SetEkfInput::Request req;
      ff_msgs::SetEkfInput::Response resp;
      req.mode = ff_msgs::SetEkfInput::Request::MODE_MAP_LANDMARKS;
      ekf_client.call(req, resp);
    });
  }

  if (sas_dock_.isPreemptRequested()) {
    return finish(dr::CANCELED, preempted);
  }
  send_progress(df::WAITING_FOR_LOC);

  // 3. Count number of features
  //    - if < TBD, switch back to sparse mapping + fails
  // if ( !enoughFeatures(10) ) {
  //   ROS_WARN("goal failed because of insufficient features");
  //   result.header.stamp = ros::Time::now();
  //   result.result.status = ff_msgs::DockerResponse::MISSING_FEATURES;
  //   sas_dock_.setAborted(result, "not enough features in AR tags mode");
  //   return result.result.status;
  // }
  // TODO(tfmorse): how the heck do we do this? for now I am just sleeping
  // to ensure that we have a position lock, but this is not a good approach
  for (int i = 0; i < static_cast<int>(p_pos_timeout_); i++) {
    std::this_thread::sleep_for(
        std::chrono::seconds(1));
    if (sas_dock_.isPreemptRequested()) {
      return finish(dr::CANCELED, preempted);
    }
  }

  feedback.progress = 50.0;
  send_progress(df::INGRESSING);

  // 4. Execute the approach
  // 5. At the same time (동시에...), monitor the dock pin
  {
    // TODO(Andrew) Fix these when converting to ff_nodelet / localization manager
    /*
    // Configure the planner (these values will persist until changed)
    ff_util::ConfigClient cfg_planner(SUBSYSTEM_MOBILITY, "planner");
    cfg_planner.Set<bool>("enable_faceforward", false);
    cfg_planner.Set<double>("lim_lin_vel", ingress_.speed);
    if (!cfg_planner.Reconfigure()) {
      ROS_ERROR_STREAM("Could not reconfigure the mobility::planner node!");
      return finish(dr::NO_MOBILITY_SVC, aborted);
    }

    // Configure the choreographer (these values will persist until changed)
    ff_util::ConfigClient cfg_choreographer(SUBSYSTEM_MOBILITY, "choreographer");
    cfg_choreographer.Set<bool>("enable_control", true);    // TODO(Ted)
    cfg_choreographer.Set<bool>("enable_immediate", true);  // TODO(Ted)
    if (!cfg_choreographer.Reconfigure()) {
      ROS_ERROR_STREAM("Could not reconfigure the mobility::choreographer node!");
      return finish(dr::NO_MOBILITY_SVC, aborted);
    }
    */
    // Create the action server
    actionlib::SimpleActionClient<ff_msgs::MoveAction>
        sac(ACTION_MOBILITY_MOVE, true);

    // Create and send a move goal
    ff_msgs::MoveGoal goal;
    goal.flight_mode = "docking";
    geometry_msgs::PoseStamped state;
    state.header.stamp = ros::Time::now();
    state.pose = p_bays_[1].end;
    goal.states.push_back(state);

    // we really care whether it's there or not. without this, it won't be
    // connected and will skip the goal message entirely. thanks, ROS.
    // (╯°□°)╯︵ ┻━┻
    if (!sac.waitForServer(ros::Duration(p_mob_timeout_))) {
      ROS_ERROR("unable to find mobility server");
      return finish(dr::NO_MOBILITY_SVC, aborted);
    }

    std::unique_lock<std::mutex> pose_lock(pose_mutex_, std::defer_lock);

    // we actually don't care abut any of the callbacks, "just let it go"
    // https://youtu.be/moSFlvxnbgk
    sac.sendGoal(goal);
    while (!sac.waitForResult(ros::Duration(60.0))) {
      if (docked_) {
        sac.cancelGoal();
        sac.stopTrackingGoal();
        break;
      }

      pose_lock.lock();
      float distance = static_cast<float>(
          PoseDistance(pose_.position,
                        p_bays_[1].end.position));
      pose_lock.unlock();

      if (sas_dock_.isPreemptRequested()) {
        return finish(dr::CANCELED, preempted);
      }
      feedback.progress = distance;
      send_progress(df::INGRESSING);
    }
  }

  feedback.progress = 0.0;
  send_progress(df::DEACTIVATING_PMC);

  // 6. Turn off propulsion when docked
  {
    ff_msgs::SetBool::Request req;
    ff_msgs::SetBool::Response resp;

    req.enable = false;
    if (!pmc_client.call(req, resp)) {
      return finish(dr::CANNOT_DISABLE_PMC, aborted);
    }

    // always returns true ¯\_(ツ)_/¯
  }

  ROS_INFO("done");
  vacuum.Clear();
  return finish(dr::DOCKED, succeeded);
}

void Docker::ExecUndock(ff_msgs::UndockGoal::ConstPtr const& goal) {
  ff_msgs::UndockFeedback feedback;
  ff_msgs::UndockResult result;

  if (have_goal_) {
    result.header.frame_id = "world";
    result.header.seq = 0;
    result.header.stamp = ros::Time::now();
    result.status = ff_msgs::UndockResult::ACTION_IN_PROGRESS;
    sas_undock_.setAborted(result, "Already docking");
    return;
  }

  have_goal_ = true;

  // Actions to perform
  // 0. Verify that we are docked
  //    - return failure if not docked
  // 1. Enable navcam
  // 2. Switch to Sparse mapping
  // 3. Count number of features
  //    - if < TBD, stop with failure
  // if ( !enoughFeatures(10) ) {
  //   ROS_DEBUG("goal failed because of insufficient features");
  //   result.header.stamp = ros::Time::now();
  //   result.result.status = ff_msgs::DockerResponse::MISSING_FEATURES;
  //   sas_undock_.setAborted(result, "not enough features in Sparse Maping mode");
  //   return result.result.status;
  // }

  // 5. Reset bias + wait
  // 6. Enable the prop module
  // 7. Wait to get correct spin?
  // 8. Execute egress

  for (int i = 1; i <= ff_msgs::UndockFeedback::MAX_STATE; i++) {
    ROS_INFO("publishing dock feedback");
    feedback.header.stamp = ros::Time::now();
    feedback.header.seq++;
    feedback.status = i;
    feedback.progress = 0.0;

    if (i == ff_msgs::UndockFeedback::EGRESSING) {
      feedback.progress = 0.0;
      sas_undock_.publishFeedback(feedback);
      for (int j = 0; j < 3; j++) {
        feedback.header.stamp = ros::Time::now();
        feedback.header.seq++;
        feedback.progress += 1.0;
        sas_undock_.publishFeedback(feedback);
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      continue;
    }

    sas_undock_.publishFeedback(feedback);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  result.header.stamp = ros::Time::now();
  result.status = ff_msgs::UndockResult::UNDOCKED;
  sas_undock_.setSucceeded(result);
  have_goal_ = false;
}

namespace {

bool ReadPose(::config_reader::ConfigReader::Table *t,
              geometry_msgs::Pose *pose) {
  ::config_reader::ConfigReader::Table pos, quat;
  if (!t->GetTable("position", &pos))
    return false;
  if (!t->GetTable("orientation", &quat))
    return false;
  if (!msg_conversions::config_read_vector(&pos, &(pose->position)))
    return false;
  if (!msg_conversions::config_read_quat(&quat, &(pose->orientation)))
    return false;

  return true;
}

bool ReadTwist(::config_reader::ConfigReader::Table *t,
               geometry_msgs::Twist *twist) {
  ::config_reader::ConfigReader::Table linear, angular;
  if (!t->GetTable("linear", &linear) ||
      !t->GetTable("angular", &angular)) {
    return false;
  }

  if (!msg_conversions::config_read_vector(&linear, &(twist->linear)) ||
      !msg_conversions::config_read_vector(&angular, &(twist->angular)))
    return false;
  return true;
}

bool ReadLimits(::config_reader::ConfigReader::Table *t, Limits *l) {
  ::config_reader::ConfigReader::Table vel, accel;
  if (!t->GetTable("vel", &vel) ||
      !t->GetTable("accel", &accel))
    return false;
  if (!ReadTwist(&vel, &(l->velocity)) ||
      !ReadTwist(&accel, &(l->acceleration)))
    return false;
  if (!t->GetReal("speed", &(l->speed)))
    return false;
  return true;
}

}  // namespace

bool Docker::ReadConfig() {
  if (!config_.ReadFiles()) {
    ROS_ERROR("unable to read docker config");
    return false;
  }

  if (!config_.GetReal("position_timeout", &p_pos_timeout_)) {
    ROS_ERROR("no position timeout specified");
    return false;
  }
  if (!config_.GetReal("cam_timeout", &p_cam_timeout_)) {
    ROS_ERROR("no camera service timeout specified");
    return false;
  }
  if (!config_.GetReal("ekf_timeout", &p_ekf_timeout_)) {
    ROS_ERROR("no ekf service timeout specified");
    return false;
  }
  if (!config_.GetReal("mobility_timeout", &p_mob_timeout_)) {
    ROS_ERROR("no mobility action server timeout specified");
    return false;
  }
  if (!config_.GetReal("pmc_timeout", &p_pmc_timeout_)) {
    ROS_ERROR("no PMC service timeout specified");
    return false;
  }

  ::config_reader::ConfigReader::Table ingress;
  if (!config_.GetTable("ingress_limits", &ingress)) {
    ROS_ERROR("no ingress limits sepecified");
    return false;
  }
  if (!ReadLimits(&ingress, &ingress_)) {
    ROS_ERROR("ingress limits specified incorrectly");
    return false;
  }

  /*
  ::config_reader::ConfigReader::Table egress;
  if (!config_.GetTable("egress_limits", &egress)) {
    ROS_ERROR("no egress limits sepecified");
    return false;
  }
  if (!ReadLimits(&egress, &egress_)) {
    ROS_ERROR("egress limits specified incorrectly");
    return false;
  }
  */

  ::config_reader::ConfigReader::Table bays;
  if (!config_.GetTable("dock_bays", &bays)) {
    ROS_ERROR("no docking bay positions specified");
    return false;
  }

  if (bays.GetSize() != 2) {
    ROS_ERROR("invalid number of bays specified");
    return false;
  }

  for (int i = 1; i <= bays.GetSize(); i++) {
    ::config_reader::ConfigReader::Table /* michael */ bay;
    if (!bays.GetTable(i, &bay)) {
      ROS_ERROR("unable to obtain bay %d", i);
      return false;
    }

    ::config_reader::ConfigReader::Table start, end;
    if (!bay.GetTable("initial", &start) ||
        !bay.GetTable("final", &end)) {
      ROS_ERROR("no initial or final positions specified");
      return false;
    }

    if (!ReadPose(&start, &(p_bays_[i-1].start)) ||
        !ReadPose(&end, &(p_bays_[i-1].end))) {
      ROS_ERROR("unable to read initial or final positions");
      return false;
    }
  }

  return true;
}

}  // namespace docking
