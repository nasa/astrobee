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

// Command line flags
#include <gflags/gflags.h>
#include <gflags/gflags_completions.h>

// Include RPOS
#include <ros/ros.h>

// Listen for transforms
#include <tf2_ros/transform_listener.h>

// FSW includes
#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_action.h>
#include <ff_util/config_client.h>

// Primitive actions
#include <ff_msgs/SwitchAction.h>
#include <ff_msgs/MoveAction.h>
#include <ff_msgs/StopAction.h>
#include <ff_msgs/IdleAction.h>
#include <ff_msgs/ExecuteAction.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// C++ STL inclues
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <memory>

// Gflags
DEFINE_string(ns, "", "Robot namespace");
DEFINE_string(loc, "", "Localization pipeline (none, ml, ar, hr)");
DEFINE_string(mode, "nominal", "Flight mode");
DEFINE_string(planner, "trapezoidal", "Path planning algorithm");
DEFINE_bool(ff, false, "Plan in face-forward mode");
DEFINE_double(rate, 1.0, "Segment sampling rate");
DEFINE_double(vel, -1.0, "Desired velocity");
DEFINE_double(accel, -1.0, "Desired acceleration");
DEFINE_double(omega, -1.0, "Desired angular velocity");
DEFINE_double(alpha, -1.0, "Desired angular acceleration");
DEFINE_bool(move, false, "Send move command");
DEFINE_bool(stop, false, "Send stop command");
DEFINE_bool(idle, false, "Send idle command");
DEFINE_bool(novalidate, false, "Don't validate the segment before running");
DEFINE_bool(nocollision, false, "Don't check for collisions during action");
DEFINE_bool(nobootstrap, false, "Don't move to the starting station on execute");
DEFINE_string(rec, "", "Plan and record to this file. Don't execute.");
DEFINE_string(exec, "", "Execute a given segment");
DEFINE_string(pos, "", "Desired position in cartesian format 'X Y Z' (meters)");
DEFINE_string(att, "", "Desired attitude in angle-axis format 'angle X Y Z'");
DEFINE_double(wait, 0.0, "Defer move by given amount in seconds (needs -noimmediate)");
DEFINE_double(connect, 10.0, "Action connect timeout");
DEFINE_double(active, 10.0, "Action active timeout");
DEFINE_double(response, 10.0, "Action response timeout");
DEFINE_double(deadline, -1.0, "Action deadline timeout");

// Avoid sending the command multiple times
bool sent_ = false;

// Generic completion function
void GenericResponseDisplay(int response_action, int mobility_action) {
  switch (response_action) {
  // Result null
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_CONNECT:
    std::cerr << "Timeout on connecting to action" << std::endl;
    break;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE:
    std::cerr << "Timeout on action going active" << std::endl;
    break;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE:
    std::cerr << "Timeout on receiving a response" << std::endl;
    break;
  case ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE:
    std::cerr << "Timeout on result deadline" << std::endl;
    break;
  // Result expected
  case ff_util::FreeFlyerActionState::Enum::SUCCESS:
  case ff_util::FreeFlyerActionState::Enum::PREEMPTED:
  case ff_util::FreeFlyerActionState::Enum::ABORTED: {
    // Print a meningful response
    switch (mobility_action) {
    case ff_msgs::MobilityResult::ALREADY_THERE:
      std::cerr << "Success: already at setpoint" << std::endl; break;
    case ff_msgs::MobilityResult::CALCULATED:
      std::cerr << "Success: solution calculated" << std::endl; break;
    case ff_msgs::MobilityResult::SUCCESS:
      std::cerr << "Success: control applied" << std::endl; break;
    case ff_msgs::MobilityResult::CANCELLED:
      std::cerr << "Cancelled" << std::endl; break;
    case ff_msgs::MobilityResult::PREEMPTED:
      std::cerr << "Preempted" << std::endl; break;
    case ff_msgs::MobilityResult::COULD_NOT_FIND_CONTROL_ENABLE_SERVICE:
      std::cerr << "Error: control service could not be found" << std::endl; break;
    case ff_msgs::MobilityResult::COULD_NOT_CALL_CONTROL_ENABLE_SERVICE:
      std::cerr << "Error: control service could not be called" << std::endl; break;
    case ff_msgs::MobilityResult::ACTION_TIMEOUT_WAITING_FOR_ACTIVE:
      std::cerr << "Error: child action timed out waiting to go active" << std::endl; break;
    case ff_msgs::MobilityResult::ACTION_TIMEOUT_WAITING_FOR_RESPONSE:
      std::cerr << "Error: child action timed out on response" << std::endl; break;
    case ff_msgs::MobilityResult::ACTION_TIMEOUT_WAITING_FOR_DEADLINE:
      std::cerr << "Error: child action timed out on deadline" << std::endl; break;
    case ff_msgs::MobilityResult::BAD_STATE_TRANSITION:
      std::cerr << "Error: choreograpber bad state transition" << std::endl; break;
    case ff_msgs::MobilityResult::OBSTACLE_AND_REPLANNING_DISABLED:
      std::cerr << "Error: obstacle detected and replanning disabled" << std::endl; break;
    case ff_msgs::MobilityResult::OBSTACLE_AND_INSUFFICIENT_REPLAN_TIME:
      std::cerr << "Error: obstacle detected and not enough time to replan" << std::endl; break;
    case ff_msgs::MobilityResult::UNAVOIDABLE_COLLISION_DETECTED:
      std::cerr << "Error: unavoidable obstacle detected" << std::endl; break;
    case ff_msgs::MobilityResult::NOT_YET_INITIALIZED:
      std::cerr << "Error: command received but choreographer not yet initialized" << std::endl; break;
    case ff_msgs::MobilityResult::STOP_CALLED_WITHOUT_CONTROL_ENABLED:
      std::cerr << "Error: stop received with control disabled" << std::endl; break;
    case ff_msgs::MobilityResult::IDLE_CALLED_WITHOUT_CONTROL_ENABLED:
      std::cerr << "Error: idle received with control disabled" << std::endl; break;
    case ff_msgs::MobilityResult::CONTROL_ENABLE_SERVICE_LOST:
      std::cerr << "Error: control enable service disappeared" << std::endl; break;
    case ff_msgs::MobilityResult::NO_FLIGHT_MODE_SPECIFIED:
      std::cerr << "Error: no flight mode specified" << std::endl; break;
    case ff_msgs::MobilityResult::UNEXPECTED_EMPTY_SEGMENT:
      std::cerr << "Error: empty segment" << std::endl; break;
    case ff_msgs::MobilityResult::CONTROL_FAILED:
      std::cerr << "Error: control failed" << std::endl; break;
    case ff_msgs::MobilityResult::SELECTED_PLANNER_DOES_NOT_EXIST:
      std::cerr << "Error: planner does not exist" << std::endl; break;
    case ff_msgs::MobilityResult::PLANNER_FAILED:
      std::cerr << "Error:  plan failed" << std::endl; break;
    case ff_msgs::MobilityResult::VALIDATE_FAILED:
      std::cerr << "Error: validate failed" << std::endl; break;
    case ff_msgs::MobilityResult::CANNOT_QUERY_ROBOT_POSE:
      std::cerr << "Error: cannot query robot pose" << std::endl; break;
    case ff_msgs::MobilityResult::NOT_ON_FIRST_POSE:
      std::cerr << "Error: not on first pose of plan" << std::endl; break;
    default:
      std::cerr << "Error: unknown" << std::endl; break;
    }
  }
  default:
    break;
  }
  ros::shutdown();
}

// Move feedback
void GenericFeedbackDisplay(ff_msgs::MobilityProgress const& progress) {
  std::cout << '\r' << std::flush;
  std::cout << std::fixed << std::setw(11) << std::setprecision(2)
    << "  [ERRORS] POS: " << 1000.00 * progress.control.error_position << " mm "
    << "ATT: " << 57.2958 * progress.control.error_attitude << " deg "
    << "VEL: " << 1000.00 * progress.control.error_velocity << " mm/s "
    << "OMEGA: " << 57.2958 * progress.control.error_omega << " deg/s ";
}

// Move feedback
void MoveFeedbackCallback(ff_msgs::MoveFeedbackConstPtr const& feedback) {
  GenericFeedbackDisplay(feedback->progress);
}

// Move result
void MoveResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
  ff_msgs::MoveResultConstPtr const& result) {
  std::cout << std::endl << "Move result received" << std::endl;
  if (result_code == ff_util::FreeFlyerActionState::Enum::SUCCESS) {
    if (!FLAGS_rec.empty()) {
      if (ff_util::SegmentUtil::Save(FLAGS_rec, result->result.flight_mode, result->result.segment))
        std::cout << "Segment saved to file " << FLAGS_rec << std::endl;
      else
        std::cerr << "Segment not saved to file " << FLAGS_rec << std::endl;
    }
  }
  GenericResponseDisplay(result_code, (result == nullptr ? 0 : result->result.response));
}

void IdleFeedbackCallback(ff_msgs::IdleFeedbackConstPtr const& feedback) {
  GenericFeedbackDisplay(feedback->progress);
}

void IdleResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
  ff_msgs::IdleResultConstPtr const& result) {
  std::cout << std::endl << "Idle result received" << std::endl;
  GenericResponseDisplay(result_code, (result == nullptr ? 0 : result->result.response));
}

void ExecFeedbackCallback(ff_msgs::ExecuteFeedbackConstPtr const& feedback) {
  GenericFeedbackDisplay(feedback->progress);
}

void ExecResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
  ff_msgs::ExecuteResultConstPtr const& result) {
  std::cout << std::endl << "Execute result received" << std::endl;
  GenericResponseDisplay(result_code, (result == nullptr ? 0 : result->result.response));
}

void StopFeedbackCallback(ff_msgs::StopFeedbackConstPtr const& feedback) {
  GenericFeedbackDisplay(feedback->progress);
}

void StopResultCallback(ff_util::FreeFlyerActionState::Enum result_code,
  ff_msgs::StopResultConstPtr const& result) {
  std::cout << std::endl << "Stop result received" << std::endl;
  GenericResponseDisplay(result_code, (result == nullptr ? 0 : result->result.response));
}

// Ignore the feedback
void SwitchFeedbackCallback(ff_msgs::SwitchFeedbackConstPtr const& feedback) {}

// Result triggers
void SwitchResultCallback(ff_util::FreeFlyerActionState::Enum result_code,                        // result
                          ff_msgs::SwitchResultConstPtr const& result,                            // result pointer
                          ff_util::FreeFlyerActionClient < ff_msgs::StopAction > * client_s_,     // mobility::stop
                          ff_util::FreeFlyerActionClient < ff_msgs::IdleAction > * client_i_,     // mobility::idle
                          ff_util::FreeFlyerActionClient < ff_msgs::MoveAction > * client_m_,     // mobility::move
                          ff_util::FreeFlyerActionClient < ff_msgs::ExecuteAction > * client_e_,  // mobility::move
                          tf2_ros::Buffer * tf_buffer_) {                                         // tf2 buffeer
  // Catch errors
  switch (result_code) {
  case ff_util::FreeFlyerActionState::SUCCESS: {
    // Idle command
    if (FLAGS_idle) {
      ff_msgs::IdleGoal idle_goal;
      idle_goal.flight_mode = FLAGS_mode;
      client_i_->SendGoal(idle_goal);
      return;
    }
    // Stop command
    if (FLAGS_stop) {
      ff_msgs::StopGoal stop_goal;
      stop_goal.flight_mode = FLAGS_mode;
      client_s_->SendGoal(stop_goal);
      return;
    }
    // Obtain the current state
    if (FLAGS_move) {
      geometry_msgs::PoseStamped state;
      try {
        std::string ns = FLAGS_ns;
        geometry_msgs::TransformStamped tfs = tf_buffer_->lookupTransform(
          std::string(FRAME_NAME_WORLD),
          (ns.empty() ? "body" : ns + "/" + std::string(FRAME_NAME_BODY)),
          ros::Time(0));
        state.header = tfs.header;
        state.pose.position.x = tfs.transform.translation.x;
        state.pose.position.y = tfs.transform.translation.y;
        state.pose.position.z = tfs.transform.translation.z;
        state.pose.orientation = tfs.transform.rotation;
      } catch (tf2::TransformException &ex) {
        std::cerr << "Could not query the pose of the robot: " << ex.what() << std::endl;
        ros::shutdown();
      }
      // Manipulate timestamp to cause deferral
      state.header.stamp += ros::Duration(FLAGS_wait);
      // Parse and modify the position
      std::string str_p = FLAGS_pos;
      if (!str_p.empty()) {
        std::istringstream iss_p(str_p);
        std::vector<double> vec_p {
          std::istream_iterator<double>(iss_p),
          std::istream_iterator<double>()
        };
        if (vec_p.size() > 0) state.pose.position.x = vec_p[0];
        if (vec_p.size() > 1) state.pose.position.y = vec_p[1];
        if (vec_p.size() > 2) state.pose.position.z = vec_p[2];
      }
      // Parse the attitude - roll, pitch then yaw
      std::string str_a = FLAGS_att;
      if (!str_a.empty()) {
        // Parse double vector from string
        std::istringstream iss_a(str_a);
        std::vector<double> vec_a {
          std::istream_iterator<double>(iss_a),
          std::istream_iterator<double>()
        };
        // Convert the axis angle input to a quaternion
        Eigen::AngleAxisd aa(0.0, Eigen::Vector3d(0.0, 0.0, 0.0));
        if (vec_a.size() == 1) {
          Eigen::Quaterniond q0(state.pose.orientation.w, state.pose.orientation.x,
            state.pose.orientation.y, state.pose.orientation.z);
          Eigen::Vector3d x(1, 0, 0);
          Eigen::Vector3d p = q0.matrix()*x;
          p(2) = 0;
          p.normalize();
          double alpha = vec_a[0] - std::atan2(p(1), p(0));
          Eigen::Quaterniond qz(std::cos(0.5*alpha), 0, 0, std::sin(0.5*alpha));
          Eigen::Quaterniond qd = qz*q0;
          Eigen::Vector3d p_check = qd.matrix()*x;
          p_check(2) = 0;
          Eigen::Vector3d p_check2(std::cos(alpha), std::sin(alpha), 0);
          // End check
          state.pose.orientation.x = qd.x();
          state.pose.orientation.y = qd.y();
          state.pose.orientation.z = qd.z();
          state.pose.orientation.w = qd.w();
        } else if (vec_a.size() == 4) {
          aa.angle() = vec_a[0];
          aa.axis().x() = vec_a[1];
          aa.axis().y() = vec_a[2];
          aa.axis().z() = vec_a[3];
          Eigen::Quaterniond q(aa);
          state.pose.orientation.x = q.x();
          state.pose.orientation.y = q.y();
          state.pose.orientation.z = q.z();
          state.pose.orientation.w = q.w();
        } else if (vec_a.size() > 0) {
          std::cerr << "Invalid axis-angle format passed to -att. Four elements required. Aborting" << std::endl;
          ros::shutdown();
        }
      }
      // Package up and send the move goal
      ff_msgs::MoveGoal move_goal;
      move_goal.flight_mode = FLAGS_mode;
      move_goal.states.push_back(state);
      client_m_->SendGoal(move_goal);
      return;
    }
    // Execute command
    if (!FLAGS_exec.empty()) {
      ff_msgs::ExecuteGoal exec_goal;
      if (!ff_util::SegmentUtil::Load(FLAGS_exec, exec_goal.flight_mode, exec_goal.segment)) {
        std::cerr << "Segment not loaded from file " << FLAGS_exec << std::endl;
        return;
      }
      client_e_->SendGoal(exec_goal);
      return;
    }
  }
  case ff_util::FreeFlyerActionState::PREEMPTED:
    std::cerr << "Error: PREEMPTED" << std::endl;
    break;
  case ff_util::FreeFlyerActionState::ABORTED:
    std::cerr << "Error: ABORTED" << std::endl;
    break;
  case ff_util::FreeFlyerActionState::TIMEOUT_ON_CONNECT:
    std::cerr << "Error: TIMEOUT_ON_CONNECT" << std::endl;
    break;
  case ff_util::FreeFlyerActionState::TIMEOUT_ON_ACTIVE:
    std::cerr << "Error: TIMEOUT_ON_ACTIVE" << std::endl;
    break;
  case ff_util::FreeFlyerActionState::TIMEOUT_ON_RESPONSE:
    std::cerr << "Error: TIMEOUT_ON_RESPONSE" << std::endl;
    break;
  case ff_util::FreeFlyerActionState::TIMEOUT_ON_DEADLINE:
    std::cerr << "Error: TIMEOUT_ON_DEADLINE" << std::endl;
    break;
  default:
    std::cerr << "Error: UNKNOWN" << std::endl;
    break;
  }
  ros::shutdown();
}

// Ensure all clients are connected
void ConnectedCallback(ff_util::FreeFlyerActionClient < ff_msgs::SwitchAction > * client_l_,   // localization_manager
                       ff_util::FreeFlyerActionClient < ff_msgs::StopAction > * client_s_,     // mobility::stop
                       ff_util::FreeFlyerActionClient < ff_msgs::IdleAction > * client_i_,     // mobility::idle
                       ff_util::FreeFlyerActionClient < ff_msgs::MoveAction > * client_m_,     // mobility::move
                       ff_util::FreeFlyerActionClient < ff_msgs::ExecuteAction > * client_e_,  // mobility::execute
                       tf2_ros::Buffer * tf_buffer_) {                                         // TF2 handle
  // Check to see if connected
  if (!client_s_->IsConnected()) return;  // Stop action
  if (!client_i_->IsConnected()) return;  // Idle action
  if (!client_m_->IsConnected()) return;  // Move action
  if (!client_l_->IsConnected()) return;  // Swotch localization
  if (sent_)
    return;
  else
     sent_ = true;
  // Debug
  std::cout << "All three action clients are connected. Sending command" << std::endl;
  // Package up and send the move goal
  if (!FLAGS_loc.empty()) {
    ff_msgs::SwitchGoal switch_goal;
    switch_goal.pipeline = FLAGS_loc;
    client_l_->SendGoal(switch_goal);
    return;
  }
  // Fake a switch result to trigger the releop action
  SwitchResultCallback(ff_util::FreeFlyerActionState::SUCCESS, nullptr,
    client_s_, client_i_, client_m_, client_e_, tf_buffer_);
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Initialize a ros node
  ros::init(argc, argv, "teleop", ros::init_options::AnonymousName);
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun mobility teleop <opts>");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Some simple checks
  uint8_t mode = 0;
  if (!FLAGS_exec.empty()) mode++;
  if (FLAGS_idle) mode++;
  if (FLAGS_stop) mode++;
  if (FLAGS_move) mode++;
  // Check we have specified one of the required switches
  if (FLAGS_loc.empty() && mode == 0) {
    std::cerr << "You must specify one of -loc, -move, -stop, -idle, -exec <segment>" << std::endl;
    return 1;
  }
  if (mode > 1) {
    std::cerr << "You can only specify one of-move, -stop, -idle, or -exec <segment>" << std::endl;
    return 1;
  }
  if (FLAGS_connect <= 0.0) {
    std::cerr << "Your connect timeout must be positive" << std::endl;
    return 1;
  }
  if (FLAGS_active <= 0.0) {
    std::cerr << "Your active timeout must be positive" << std::endl;
    return 1;
  }
  if (FLAGS_response <= 0.0) {
    std::cerr << "Your response timeout must be positive" << std::endl;
    return 1;
  }
  // Action clients
  ff_util::FreeFlyerActionClient < ff_msgs::SwitchAction > client_l_;     // switch localization
  ff_util::FreeFlyerActionClient < ff_msgs::StopAction > client_s_;       // mobility::stop
  ff_util::FreeFlyerActionClient < ff_msgs::IdleAction > client_i_;       // mobility::idle
  ff_util::FreeFlyerActionClient < ff_msgs::MoveAction > client_m_;       // mobility::move
  ff_util::FreeFlyerActionClient < ff_msgs::ExecuteAction > client_e_;    // mobility::execute
  // Create a node handle
  ros::NodeHandle nh(std::string("/") + FLAGS_ns);
  // TF2 Subscriber
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tfListener(tf_buffer_);
  // Perform a reconfiguration of the choreographer based on the input
  ff_util::ConfigClient cfg_choreographer(&nh, NODE_CHOREOGRAPHER);
  // Soft limits
  if (FLAGS_vel > 0)
    cfg_choreographer.Set<double>("desired_vel", FLAGS_vel);
  if (FLAGS_accel > 0)
    cfg_choreographer.Set<double>("desired_accel", FLAGS_accel);
  if (FLAGS_omega > 0)
    cfg_choreographer.Set<double>("desired_omega", FLAGS_omega);
  if (FLAGS_alpha > 0)
    cfg_choreographer.Set<double>("desired_alpha", FLAGS_alpha);
  if (FLAGS_rate > 0)
    cfg_choreographer.Set<double>("desired_rate", FLAGS_rate);
  // Boolean flags
  cfg_choreographer.Set<bool>("enable_collision_checking", !FLAGS_novalidate);
  cfg_choreographer.Set<bool>("enable_validation", !FLAGS_nocollision);
  cfg_choreographer.Set<bool>("enable_bootstrapping", !FLAGS_nobootstrap);
  cfg_choreographer.Set<bool>("enable_immediate", (FLAGS_wait == 0.0));
  cfg_choreographer.Set<bool>("enable_faceforward", FLAGS_ff);
  cfg_choreographer.Set<bool>("enable_control", FLAGS_rec.empty());
  // Planner
  if (!FLAGS_planner.empty())
    cfg_choreographer.Set<std::string>("planner", FLAGS_planner);
  if (!cfg_choreographer.Reconfigure()) {
    std::cerr << "Could not reconfigure the mobility::choreographer node!" << std::endl;
    return 1;
  }
  // Setup SWITCH action
  client_l_.SetConnectedTimeout(FLAGS_connect);
  client_l_.SetActiveTimeout(FLAGS_active);
  client_l_.SetResponseTimeout(FLAGS_response);
  if (FLAGS_deadline > 0)
    client_l_.SetDeadlineTimeout(FLAGS_deadline);
  client_l_.SetFeedbackCallback(std::bind(SwitchFeedbackCallback, std::placeholders::_1));
  client_l_.SetResultCallback(std::bind(SwitchResultCallback, std::placeholders::_1, std::placeholders::_2,
    &client_s_, &client_i_, &client_m_, &client_e_, &tf_buffer_));
  client_l_.SetConnectedCallback(std::bind(ConnectedCallback,
    &client_l_, &client_s_, &client_i_, &client_m_, &client_e_, &tf_buffer_));
  client_l_.Create(&nh, ACTION_LOCALIZATION_MANAGER_SWITCH);
  // Setup MOVE action
  client_m_.SetConnectedTimeout(FLAGS_connect);
  client_m_.SetActiveTimeout(FLAGS_active);
  client_m_.SetResponseTimeout(FLAGS_response);
  if (FLAGS_deadline > 0)
    client_m_.SetDeadlineTimeout(FLAGS_deadline);
  client_m_.SetFeedbackCallback(std::bind(MoveFeedbackCallback, std::placeholders::_1));
  client_m_.SetResultCallback(std::bind(MoveResultCallback, std::placeholders::_1, std::placeholders::_2));
  client_m_.SetConnectedCallback(std::bind(ConnectedCallback,
    &client_l_, &client_s_, &client_i_, &client_m_, &client_e_, &tf_buffer_));
  client_m_.Create(&nh, ACTION_MOBILITY_MOVE);
  // Setup IDLE action
  client_e_.SetConnectedTimeout(FLAGS_connect);
  client_e_.SetActiveTimeout(FLAGS_active);
  client_e_.SetResponseTimeout(FLAGS_response);
  if (FLAGS_deadline > 0)
    client_e_.SetDeadlineTimeout(FLAGS_deadline);
  client_e_.SetFeedbackCallback(std::bind(ExecFeedbackCallback, std::placeholders::_1));
  client_e_.SetResultCallback(std::bind(ExecResultCallback, std::placeholders::_1, std::placeholders::_2));
  client_e_.SetConnectedCallback(std::bind(ConnectedCallback,
    &client_l_, &client_s_, &client_i_, &client_m_, &client_e_, &tf_buffer_));
  client_e_.Create(&nh, ACTION_MOBILITY_EXECUTE);
  // Setup STOP action
  client_s_.SetConnectedTimeout(FLAGS_connect);
  client_s_.SetActiveTimeout(FLAGS_active);
  client_s_.SetResponseTimeout(FLAGS_response);
  if (FLAGS_deadline > 0)
    client_s_.SetDeadlineTimeout(FLAGS_deadline);
  client_s_.SetFeedbackCallback(std::bind(StopFeedbackCallback, std::placeholders::_1));
  client_s_.SetResultCallback(std::bind(StopResultCallback, std::placeholders::_1, std::placeholders::_2));
  client_s_.SetConnectedCallback(std::bind(ConnectedCallback,
    &client_l_, &client_s_, &client_i_, &client_m_, &client_e_, &tf_buffer_));
  client_s_.Create(&nh, ACTION_MOBILITY_STOP);
  // Setup IDLE action
  client_i_.SetConnectedTimeout(FLAGS_connect);
  client_i_.SetActiveTimeout(FLAGS_active);
  client_i_.SetResponseTimeout(FLAGS_response);
  if (FLAGS_deadline > 0)
    client_i_.SetDeadlineTimeout(FLAGS_deadline);
  client_i_.SetFeedbackCallback(std::bind(IdleFeedbackCallback, std::placeholders::_1));
  client_i_.SetResultCallback(std::bind(IdleResultCallback, std::placeholders::_1, std::placeholders::_2));
  client_i_.SetConnectedCallback(std::bind(ConnectedCallback,
    &client_l_, &client_s_, &client_i_, &client_m_, &client_e_, &tf_buffer_));
  client_i_.Create(&nh, ACTION_MOBILITY_IDLE);
  // Synchronous mode
  ros::spin();
  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
