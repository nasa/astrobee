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

// Include ROS
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <ff_msgs/AckCompletedStatus.h>
#include <ff_msgs/AckStamped.h>
#include <ff_msgs/AckStatus.h>
#include <ff_msgs/AgentStateStamped.h>
#include <ff_msgs/CommandArg.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/DockAction.h>
#include <ff_msgs/DockState.h>
#include <ff_msgs/FaultState.h>
#include <ff_msgs/MotionAction.h>
#include <ff_msgs/PerchState.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_names.h>

// Gflags
DEFINE_bool(dock, false, "Send dock command");
DEFINE_bool(get_face_forward, false, "Get face forward mode");
DEFINE_bool(get_faults, false, "Get current faults");
DEFINE_bool(get_op_limits, false, "Get operating limits");
DEFINE_bool(get_planner, false, "Get current planner");
DEFINE_bool(get_pose, false, "Get current robot position");
DEFINE_bool(get_state, false, "Get current robot state");
DEFINE_bool(move, false, "Send move command");
DEFINE_bool(set_op_limits, false, "Send set operating limits");
DEFINE_bool(stop, false, "Send stop command");
DEFINE_bool(undock, false, "Send undock command");
DEFINE_bool(relative, false, "Position is relative to current point");
DEFINE_bool(reset_bias, false, "Send initialize bias command");
DEFINE_bool(reset_ekf, false, "Send reset ekf command");

DEFINE_double(accel, -1.0, "Desired acceleration");
DEFINE_double(alpha, -1.0, "Desired angular acceleration");
DEFINE_double(omega, -1.0, "Desired angular velocity");
DEFINE_double(vel, -1.0, "Desired velocity");
DEFINE_double(collision_distance, -1.0, "Desired collision distance");

DEFINE_int32(berth, 1, "Berth to dock in");

DEFINE_string(att, "", "Desired attitude in angle-axis format 'angle X Y Z'");
DEFINE_string(set_face_forward, "", "Plan in face-forward mode, on or off");
DEFINE_string(mode, "", "Flight mode");
DEFINE_string(ns, "", "Robot namespace");
DEFINE_string(set_planner, "", "Path planning algorithm");
DEFINE_string(pos, "", "Desired position in cartesian format 'X Y Z'");

bool get_face_forward, get_op_limits, get_planner, get_state, get_faults;
bool reset_bias, reset_ekf, set_face_forward, set_planner, set_op_limits;
bool send_mob_command, mob_command_finished;

geometry_msgs::TransformStamped tfs;

ros::Publisher cmd_pub;

uint8_t modeMove = 0, modeGetInfo = 0;

bool Finished() {
  if (!get_face_forward && !get_op_limits && !get_planner && !get_state &&
      !get_faults && !reset_bias && !reset_ekf && !set_face_forward &&
      !set_planner && !set_op_limits && mob_command_finished) {
    return true;
  }

  return false;
}

void AgentStateCallback(ff_msgs::AgentStateStampedConstPtr const& state) {
  if (get_face_forward) {
    get_face_forward = false;
    if (state->holonomic_enabled) {
      std::cout << "\nFace forward mode is disabled.\n";
    } else {
      std::cout << "\nFace forward mode is enabled.\n";
    }
  }

  if (get_op_limits) {
    get_op_limits  = false;

    std::cout << "Flight Mode: " << state->flight_mode << "\n";
    std::cout << "Target Linear Velocity: " << state->target_linear_velocity;
    std::cout << "\nTarget Linear Acceleration: " << state->target_linear_accel;
    std::cout << "\nTarget Angular Velocity: ";
    std::cout << state->target_angular_velocity;
    std::cout << "\nTarget Angular Acceleration: ";
    std::cout << state->target_angular_accel;
    std::cout << "\nCollision Distance: " << state->collision_distance << "\n";
  }

  if (get_planner) {
    get_planner = false;

    std::cout << "\nPlanner is set to " << state->planner << ".\n";
  }

  if (get_state) {
    get_state = false;
    std::string msg = "\nOperating State: ";
    switch (state->operating_state.state) {
      case ff_msgs::OpState::READY:
        msg += "Ready\n";
        break;
      case ff_msgs::OpState::PLAN_EXECUTION:
        msg += "Plan Execution\n";
        break;
      case ff_msgs::OpState::TELEOPERATION:
        msg += "Teleoperation\n";
        break;
      case ff_msgs::OpState::AUTO_RETURN:
        msg += "Auto Return\n";
        break;
      case ff_msgs::OpState::FAULT:
        msg += "Fault\n";
        break;
      default:
        msg += "Unknown\n";
    }

    msg += "Mobility State: ";
    switch (state->mobility_state.state) {
      case ff_msgs::MobilityState::DRIFTING:
        msg += "Drifting";
        break;
      case ff_msgs::MobilityState::STOPPING:
        if (state->mobility_state.sub_state == 0) {
          msg += "Stopped";
        } else {
          msg += "Stopping";
        }
        break;
      case ff_msgs::MobilityState::FLYING:
        msg += "Flying";
        break;
      case ff_msgs::MobilityState::DOCKING:
        if (state->mobility_state.sub_state == ff_msgs::DockState::DOCKED) {
          msg += "Docked";
        } else if (state->mobility_state.sub_state <
                                                  ff_msgs::DockState::DOCKED) {
          msg += "Undocking (on step ";
          msg += std::to_string(state->mobility_state.sub_state * -1);
          msg += " of ";
          msg += std::to_string(((ff_msgs::DockState::UNDOCKING_MAX_STATE * -1)
                                                                          - 1));
          msg += ")";
        } else {
          msg += "Docking (on step ";
          msg += std::to_string((ff_msgs::DockState::DOCKING_MAX_STATE -
                                 state->mobility_state.sub_state));
          msg += " of " + std::to_string(ff_msgs::DockState::DOCKING_MAX_STATE);
          msg += ")";
        }
        break;
      case ff_msgs::MobilityState::PERCHING:
        if (state->mobility_state.sub_state == 0) {
          msg += "Perched";
        } else if (state->mobility_state.sub_state < 0) {
          msg += "Unperching (on step " + (state->mobility_state.sub_state *
                                                                            -1);
          // TODO(Katie) Comment in when perching is implemented
          // msg += " of " + (ff_msgs::PerchState::UNPERCHING_MAX_STATE * -1);
          // msg += ")";
          msg += " of ?)";
        } else {
          msg += "Perching (on step " + state->mobility_state.sub_state;
          // TODO(Katie) Comment in when perching is implemented
          // msg += " of " + ff_msgs::PerchState::PERCHING_MAX_STATE;
          // msg += ")";
          msg += " of ?)";
        }
        break;
      default:
        msg += "Unknown";
    }
    std::cout << msg << "\n";
  }

  if (Finished()) {
    ros::shutdown();
  }
}

void FaultStateCallback(ff_msgs::FaultStateConstPtr const& state) {
  get_faults = false;

  // Output faults
  if (state->faults.size() > 0) {
    std::cout << "\nCurrent faults occuring: \n";
    for (unsigned int i = 0; i < state->faults.size(); i++) {
      std::cout << state->faults[i].msg << std::endl;
    }
  } else {
    std::cout << "There are no faults occurring in the system.\n";
  }

  if (Finished()) {
    ros::shutdown();
  }
}

bool SendMobilityCommand() {
  ff_msgs::CommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.subsys_name = "Astrobee";

  if (FLAGS_dock) {
    cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_DOCK;
    cmd.cmd_id = ff_msgs::CommandConstants::CMD_NAME_DOCK;

    // Dock has one argument
    cmd.args.resize(1);
    cmd.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_INT;
    cmd.args[0].i = FLAGS_berth;
  }

  if (FLAGS_move) {
    cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF;
    cmd.cmd_id = ff_msgs::CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF;

    // Move command has 4 arguements
    cmd.args.resize(4);

    // Set frame
    cmd.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
    if (FLAGS_relative) {
      cmd.args[0].s = "body";
    } else {
      cmd.args[0].s = "world";
    }

    // Set tolerance. Currently not used but needs to be in the command
    cmd.args[2].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
    cmd.args[2].vec3d[0] = 0;
    cmd.args[2].vec3d[1] = 0;
    cmd.args[2].vec3d[2] = 0;

    // Initialize position to be the current position if not a relative move
    cmd.args[1].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
    if (FLAGS_relative) {
      cmd.args[1].vec3d[0] = 0;
      cmd.args[1].vec3d[1] = 0;
      cmd.args[1].vec3d[2] = 0;
    } else {
      cmd.args[1].vec3d[0] = tfs.transform.translation.x;
      cmd.args[1].vec3d[1] = tfs.transform.translation.y;
      cmd.args[1].vec3d[2] = tfs.transform.translation.z;
    }

    // Parse position if it was specified
    std::string str_pos = FLAGS_pos;
    if (!str_pos.empty()) {
      std::istringstream iss_pos(str_pos);
      std::vector<double> vec_pos {
        std::istream_iterator<double>(iss_pos),
        std::istream_iterator<double>()
      };

      if (vec_pos.size() > 0) {
        cmd.args[1].vec3d[0] = vec_pos[0];
      }

      if (vec_pos.size() > 1) {
        cmd.args[1].vec3d[1] = vec_pos[1];
      }

      if (vec_pos.size() > 2) {
        cmd.args[1].vec3d[2] = vec_pos[2];
      }
    }

    // Parse and set the attitude - roll, pitch then yaw
    cmd.args[3].data_type = ff_msgs::CommandArg::DATA_TYPE_MAT33f;
    if (FLAGS_att.empty()) {
      if (FLAGS_relative) {
        cmd.args[3].mat33f[0] = 0;
        cmd.args[3].mat33f[1] = 0;
        cmd.args[3].mat33f[2] = 0;
        cmd.args[3].mat33f[3] = 1;
      } else {
        cmd.args[3].mat33f[0] = tfs.transform.rotation.x;
        cmd.args[3].mat33f[1] = tfs.transform.rotation.y;
        cmd.args[3].mat33f[2] = tfs.transform.rotation.z;
        cmd.args[3].mat33f[3] = tfs.transform.rotation.w;
      }
    } else {
      std::string str_att = FLAGS_att;
      std::istringstream iss_att(str_att);
      std::vector<double> vec_att {
        std::istream_iterator<double>(iss_att),
        std::istream_iterator<double>()
      };

      // Convert the axis angle input to a quaternion
      Eigen::AngleAxisd aa(0.0, Eigen::Vector3d(0.0, 0.0, 0.0));
      if (vec_att.size() == 1) {
        Eigen::Quaterniond q0(tfs.transform.rotation.w,
                              tfs.transform.rotation.x,
                              tfs.transform.rotation.y,
                              tfs.transform.rotation.z);
        Eigen::Vector3d x(1, 0, 0);
        Eigen::Vector3d p = q0.matrix()*x;
        p(2) = 0;
        p.normalize();
        double alpha = vec_att[0] - std::atan2(p(1), p(0));
        Eigen::Quaterniond qz(std::cos(0.5*alpha), 0, 0, std::sin(0.5*alpha));
        Eigen::Quaterniond qd = qz*q0;
        cmd.args[3].mat33f[0] = qd.x();
        cmd.args[3].mat33f[1] = qd.y();
        cmd.args[3].mat33f[2] = qd.z();
        cmd.args[3].mat33f[3] = qd.w();
      } else if (vec_att.size() == 4) {
        aa.angle() = vec_att[0];
        aa.axis().x() = vec_att[1];
        aa.axis().y() = vec_att[2];
        aa.axis().z() = vec_att[3];
        Eigen::Quaterniond q(aa);
        cmd.args[3].mat33f[0] = q.x();
        cmd.args[3].mat33f[1] = q.y();
        cmd.args[3].mat33f[2] = q.z();
        cmd.args[3].mat33f[3] = q.w();
      } else {
        std::cout << "\nInvalid axis-angle format pass to -att. One or four ";
        std::cout << "elements required. Aborting!\n";
        return false;
      }
    }
  }

  if (FLAGS_stop) {
    cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_STOP_ALL_MOTION;
    cmd.cmd_id = ff_msgs::CommandConstants::CMD_NAME_STOP_ALL_MOTION;
  }

  if (FLAGS_undock) {
    cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_UNDOCK;
    cmd.cmd_id = ff_msgs::CommandConstants::CMD_NAME_UNDOCK;
  }

  cmd_pub.publish(cmd);
  // Signify that the main mobility command has been sent
  send_mob_command = false;
  std::cout << "\nStarted " << cmd.cmd_id << " command. It may take some time ";
  std::cout << "to complete.\n";
  return true;
}

bool SendResetBias() {
  ff_msgs::CommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.subsys_name = "Astrobee";

  cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_INITIALIZE_BIAS;
  cmd.cmd_id = ff_msgs::CommandConstants::CMD_NAME_INITIALIZE_BIAS;

  cmd_pub.publish(cmd);
  // Change to false so we don't send the command again
  reset_bias = false;
  std::cout << "\nResetting the bias.\n";
  return true;
}

bool SendResetEkf() {
  ff_msgs::CommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.subsys_name = "Astrobee";

  cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_RESET_EKF;
  cmd.cmd_id = ff_msgs::CommandConstants::CMD_NAME_RESET_EKF;

  cmd_pub.publish(cmd);
  // Change to false so we don't send the command again
  reset_ekf = false;
  std::cout << "\nResetting the ekf.\n";
  return true;
}

bool SendSetFaceForward() {
  ff_msgs::CommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.subsys_name = "Astrobee";

  cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_SET_HOLONOMIC_MODE;
  // For this example only, we will set the command id to the name command
  // This not standard so please don't do it. This should be a unique id,
  // usually a combination of username and timestamp
  cmd.cmd_id = ff_msgs::CommandConstants::CMD_NAME_SET_HOLONOMIC_MODE;

  // Set holonomic has one argument
  cmd.args.resize(1);
  cmd.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_BOOL;
  if (FLAGS_set_face_forward == "on") {
    // Holonomic means don't fly face forward so we need to set holonomic to
    // false when we want to face forward
    cmd.args[0].b = false;
  } else {
    cmd.args[0].b = true;
  }

  cmd_pub.publish(cmd);
  // Change to false so we don't send command again
  set_face_forward = false;
  std::cout << "\nSetting holonomic (face forward) mode.\n";
  return true;
}

bool SendSetOpLimits() {
  ff_msgs::CommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.subsys_name = "Astrobee";

  cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_SET_OPERATING_LIMITS;
  cmd.cmd_id = ff_msgs::CommandConstants::CMD_NAME_SET_OPERATING_LIMITS;

  // Set op limits has seven arguments
  cmd.args.resize(7);

  // Set profile name
  cmd.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  cmd.args[0].s = "user_profile";

  // Set flight mode
  cmd.args[1].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  cmd.args[1].s = FLAGS_mode;

  // Set desired limits
  // Target Linear Velocity
  cmd.args[2].data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  cmd.args[2].f = FLAGS_vel;

  // Target Linear Acceleration
  cmd.args[3].data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  cmd.args[3].f = FLAGS_accel;

  // Target Angular Velocity
  cmd.args[4].data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  cmd.args[4].f = FLAGS_omega;

  // Target Angular Acceleration
  cmd.args[5].data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  cmd.args[5].f = FLAGS_alpha;

  // Set collision distance
  cmd.args[6].data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  cmd.args[6].f = FLAGS_collision_distance;
  if (cmd.args[6].f == -1) {
    cmd.args[6].f = 0.25;
  }

  cmd_pub.publish(cmd);
  // Change set op limits to false so we don't send the command again
  set_op_limits = false;
  std::cout << "\nSetting operating limits.\n";
  return true;
}

bool SendSetPlanner() {
  ff_msgs::CommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.subsys_name = "Astrobee";

  cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_SET_PLANNER;
  cmd.cmd_id = ff_msgs::CommandConstants::CMD_NAME_SET_PLANNER;

  // Set planner has one argument
  cmd.args.resize(1);
  cmd.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  cmd.args[0].s = FLAGS_set_planner;

  cmd_pub.publish(cmd);
  // Change to false so we don't send command again
  set_planner = false;
  std::cout << "\nSetting planner.\n";
  return true;
}

bool SendNextCommand() {
  // Reset bias is the first command we should send as it puts the robot in a
  // stable state
  if (reset_bias) {
    return SendResetBias();
  }

  // Reset ekf is the second command we should send
  if (reset_ekf) {
    return SendResetEkf();
  }

  // Sending the mobility setting is the next thing we should do
  if (set_face_forward) {
    return SendSetFaceForward();
  }

  if (set_planner) {
    return SendSetPlanner();
  }

  if (set_op_limits) {
    return SendSetOpLimits();
  }

  // Finally send the mobility command
  if (send_mob_command) {
    return SendMobilityCommand();
  }

  return true;
}

void AckCallback(ff_msgs::AckStampedConstPtr const& ack) {
  if (ack->completed_status.status == ff_msgs::AckCompletedStatus::NOT) {
    return;
  } else if (ack->completed_status.status == ff_msgs::AckCompletedStatus::OK) {
    std::cout << "\n" << ack->cmd_id << " command completed successfully!\n";
    // Check if the mobility command was sent and if it was, we can say the
    // mobility command was finished since we only send one command at a time
    // and move is the last command to be sent
    if (modeMove > 0 && !send_mob_command) {
      mob_command_finished = true;
    }

    if (!SendNextCommand()) {
      ros::shutdown();
      return;
    }
  } else if (ack->completed_status.status ==
                                        ff_msgs::AckCompletedStatus::CANCELED) {
    std::cout << "\n" << ack->cmd_id << " command was cancelled. This may have";
    std::cout << " been due to a fault in the system or if someone had issued ";
    std::cout << "another command. Aborting!\n";
    ros::shutdown();
    return;
  } else {
    // Command failed due to bad syntax or an actual failure
    std::cout << "\n" << ack->cmd_id << " command failed! " << ack->message;
    std::cout << "\n";
    ros::shutdown();
    return;
  }
  if (Finished()) {
    ros::shutdown();
  }
}

void MoveFeedbackCallback(ff_msgs::MotionActionFeedbackConstPtr const& fb) {
  std::cout << '\r' << std::flush;
  std::cout << std::fixed << std::setprecision(2)
    << "pos: x: " << fb->feedback.progress.setpoint.pose.position.x
    << " y: " << fb->feedback.progress.setpoint.pose.position.y
    << " z: " << fb->feedback.progress.setpoint.pose.position.z
    << " att: x: " << fb->feedback.progress.setpoint.pose.orientation.x
    << " y: " << fb->feedback.progress.setpoint.pose.orientation.y
    << " z: " << fb->feedback.progress.setpoint.pose.orientation.z
    << " w: " << fb->feedback.progress.setpoint.pose.orientation.w;
}

// TODO(Katie) Finish me
void DockFeedbackCallback(ff_msgs::DockActionFeedbackConstPtr const& fb) {
  if (fb->feedback.state.state > ff_msgs::DockState::INITIALIZING) {
    std::cout << "Astrobee failed to un/dock and is trying to recover.\n\n";
  } else if (fb->feedback.state.state < ff_msgs::DockState::DOCKED) {
    std::cout << "Undocking " << (fb->feedback.state.state * -1) << " (of " <<
      ((ff_msgs::DockState::UNDOCKING_MAX_STATE * -1) - 1) << ")\n";
  } else if (fb->feedback.state.state < ff_msgs::DockState::UNKNOWN &&
             fb->feedback.state.state > ff_msgs::DockState::DOCKED) {
    std::cout << "Docking " <<
      std::to_string((ff_msgs::DockState::DOCKING_MAX_STATE -
      fb->feedback.state.state + 1)) << " (of " <<
      ff_msgs::DockState::DOCKING_MAX_STATE << ")\n";
  }
}

// Main entry point for application
int main(int argc, char** argv) {
  // Initialize a ros node
  ros::init(argc, argv, "simple_move", ros::init_options::AnonymousName);

  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun executive simple_move <opts>");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Initialize booleans
  get_face_forward = FLAGS_get_face_forward;
  get_op_limits = FLAGS_get_op_limits;
  get_planner = FLAGS_get_planner;
  get_state = FLAGS_get_state;
  get_faults =  FLAGS_get_faults;
  reset_bias = FLAGS_reset_bias;
  reset_ekf = FLAGS_reset_ekf;
  set_face_forward = false;
  set_planner = false;
  set_op_limits = FLAGS_set_op_limits;
  send_mob_command = false;
  mob_command_finished = true;

  if (FLAGS_dock) modeMove++;
  if (FLAGS_move) modeMove++;
  if (FLAGS_stop) modeMove++;
  if (FLAGS_undock) modeMove++;
  if (FLAGS_get_pose) modeGetInfo++;
  if (FLAGS_get_face_forward) modeGetInfo++;
  if (FLAGS_get_op_limits) modeGetInfo++;
  if (FLAGS_get_planner) modeGetInfo++;
  if (FLAGS_get_state) modeGetInfo++;
  if (FLAGS_get_faults) modeGetInfo++;
  if (FLAGS_reset_bias) modeGetInfo++;
  if (FLAGS_reset_ekf) modeGetInfo++;
  if (FLAGS_set_planner != "") modeGetInfo++;
  if (FLAGS_set_face_forward != "") modeGetInfo++;
  if (FLAGS_set_op_limits) modeGetInfo++;

  // Check we have specified one of the required actions
  if (modeMove == 0 && modeGetInfo == 0) {
    std::cout << "You must specify a -move, -stop, -undock, -dock, -get_pose, ";
    std::cout << "-get_face_forward, -get_op_limits, -get_planner, ";
    std::cout << "-get_state, -get_faults, -reset_bias, or -reset_ekf.\n\n";
    return 1;
  }

  if (modeMove > 1) {
    std::cout << "You can only specify one -move, -stop, -undock, or -dock\n\n";
    return 1;
  }

  if (modeMove == 1) {
    send_mob_command = true;
    mob_command_finished = false;
  }

  if (FLAGS_move && FLAGS_pos.empty() && FLAGS_att.empty()) {
    std::cout << "The move flag must also have a pos / att flag\n\n";
    return 1;
  }

  // Check to see if the user set one of the operating limits
  if (FLAGS_set_op_limits) {
    if (FLAGS_mode == "" || FLAGS_accel == -1 || FLAGS_alpha == -1 ||
        FLAGS_omega == -1 || FLAGS_vel == -1) {
      std::cout << "The set operating limits flag must also have a mode, ";
      std::cout << "accel, alpha, omega, and vel flag.\n\n";
      return 1;
    }
  }

  // Check to see if the user specified face forward mode
  if (FLAGS_set_face_forward != "") {
    if (FLAGS_set_face_forward != "on" && FLAGS_set_face_forward != "off") {
      std::cout << "The face forward flag must be on or off.\n\n";
      return 1;
    }
    set_face_forward = true;
  }

  // Check to see if the user specified a planner
  if (FLAGS_set_planner != "") {
    set_planner = true;
  }

  // Create a node handle
  ros::NodeHandle nh(std::string("/") + FLAGS_ns);

  // TF2 Subscriber
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Initialize publishers
  cmd_pub = nh.advertise<ff_msgs::CommandStamped>(TOPIC_COMMAND, 10, true);

  // Initialize subscribers
  ros::Subscriber ack_sub, agent_state_sub, fault_state_sub, dock_sub, move_sub;
  ack_sub = nh.subscribe(TOPIC_MANAGEMENT_ACK, 10, &AckCallback);

  // Hacky time out
  int count = 0;
  while (ack_sub.getNumPublishers() == 0) {
    ros::Duration(0.2).sleep();
    // Only wait 2 seconds
    if (count == 9) {
      std::cout << "No publisher for acks topics. This tool will not work ";
      std::cout << "without this.\n\n";
      return 1;
    }
    count++;
  }

  // If the user wants to get pose or move, get the current pose of the robot
  if (FLAGS_get_pose || FLAGS_move) {
    std::string ns = FLAGS_ns;
    // Wait for transform listener to start up
    ros::Duration(1.0).sleep();
    try {
      tfs = tf_buffer.lookupTransform(std::string(FRAME_NAME_WORLD),
          (ns.empty() ? "body" : ns + "/" + std::string(FRAME_NAME_BODY)),
          ros::Time(0));
    } catch (tf2::TransformException &ex) {
      std::cout << "Could not query the pose of robot: " << ex.what() << "\n\n";
      return 1;
    }
  }

  if (FLAGS_get_pose) {
    std::cout << "Pose:\nx: " << tfs.transform.translation.x << "\ny: ";
    std::cout << tfs.transform.translation.y << "\nz: ";
    std::cout << tfs.transform.translation.z << "\nRotation:\nx: ";
    std::cout << tfs.transform.rotation.x << "\ny: ";
    std::cout << tfs.transform.rotation.y << "\nz: ";
    std::cout << tfs.transform.rotation.z << "\nw: ";
    std::cout << tfs.transform.rotation.w << "\n\n";

    // If the user only wanted the pose, exit
    if (modeMove == 0 && modeGetInfo == 1) {
      return 1;
    }
  }

  if (FLAGS_get_state || FLAGS_get_face_forward || FLAGS_get_op_limits ||
      FLAGS_get_planner) {
    agent_state_sub = nh.subscribe(TOPIC_MANAGEMENT_EXEC_AGENT_STATE,
                                   10,
                                   &AgentStateCallback);
  }

  if (FLAGS_get_faults) {
    fault_state_sub = nh.subscribe(TOPIC_MANAGEMENT_SYS_MONITOR_STATE,
                                   10,
                                   &FaultStateCallback);
  }

  if (FLAGS_move) {
    std::string topic_name = ACTION_MOBILITY_MOTION;
    topic_name += "/feedback";
    move_sub = nh.subscribe(topic_name, 10, &MoveFeedbackCallback);
  }

  if (FLAGS_dock || FLAGS_undock) {
    std::string topic_name = ACTION_BEHAVIORS_DOCK;
    topic_name += "/feedback";
    dock_sub = nh.subscribe(topic_name, 10, &DockFeedbackCallback);
    // Hacky time out
    int dock_count = 0;
    while (dock_sub.getNumPublishers() == 0) {
      ros::Duration(0.2).sleep();
      // Only wait 2 seconds
      if (dock_count == 9) {
        std::cout << "No publisher for dock feedback. This tool will not work ";
        std::cout << "without this.\n\n";
        return 1;
      }
      dock_count++;
    }
  }

  if (!SendNextCommand()) {
    return 1;
  }

  // Synchronous mode
  ros::spin();

  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  return 0;
}
