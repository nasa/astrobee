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

// FSW includes
#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_serialization.h>
#include <jsonloader/planio.h>

// Primitive actions
#include <ff_msgs/PlanAction.h>
#include <ff_msgs/MotionAction.h>

// For the trapezoidal planner implementation
#include <planner_trapezoidal/planner_trapezoidal.h>

// C++ STL includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <ctime>

// Gflags
// DEFINE_string(planner, "trapezoidal", "Planner name (trapezoidal, qp)");
DEFINE_string(input, "", "Input file with rows of type: x y z roll pitch yaw (in degrees).");
DEFINE_string(output, "", "Output file.");
DEFINE_string(output_type, "fplan", "Output file type. Options: fplan (default), csv and bin. "
              "It will write either a JSON file with .fplan extension to be used in GDS, a .csv file "
              "with only accelerations to be used in MGTF, or a bin file to be used with EXEC.");
// DEFINE_bool(ff, false, "Plan in face-forward mode");
// DEFINE_double(rate, 62.5, "Segment sampling rate");
DEFINE_double(vel, 0.2, "Desired velocity in m/s");
DEFINE_double(accel, 0.0175, "Desired acceleration in m/s^2");
DEFINE_double(omega, 0.1745, "Desired angular velocity in rad/s");
DEFINE_double(alpha, 0.2, "Desired angular acceleration in rad/s^2");
DEFINE_double(tolerance, 0.1, "When two points are close enough.");
DEFINE_string(creator, "astrobee", "The name of the creator of the plan.");
DEFINE_string(rotations_multiplication_order, "yaw-pitch-roll",
              "The order in which the roll, pitch, and yaw matrices will "
              "be multiplied. Options: roll-pitch-yaw and yaw-pitch-roll.");

enum ROTATIONS_MULTIPLICATION_ORDER {ROLL_PITCH_YAW, YAW_PITCH_ROLL};

bool has_only_whitespace_or_comments(const std::string & str) {
  for (std::string::const_iterator it = str.begin(); it != str.end(); it++) {
    if (*it == '#') return true;  // No need to check further
    if (*it != ' ' && *it != '\t' && *it != '\n' && *it != '\r') return false;
  }
  return true;
}

// Convert roll, pitch, yaw specified in degrees to a quaternion.
// The order in which these are applied matters, and we support both of them.
Eigen::Quaterniond roll_pitch_yaw_to_quaternion(ROTATIONS_MULTIPLICATION_ORDER order,
                                                double roll, double pitch, double yaw) {
  double to_rad = M_PI/180.0;
  if (order == ROLL_PITCH_YAW)
    return
      Eigen::AngleAxisd(roll*to_rad,  Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch*to_rad, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw*to_rad,   Eigen::Vector3d::UnitZ());

  return
    Eigen::AngleAxisd(yaw*to_rad,   Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch*to_rad, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll*to_rad,  Eigen::Vector3d::UnitX());
}

// Main entry point for application
int main(int argc, char *argv[]) {
  google::SetUsageMessage("Usage: plangen <options>\n");
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  // Make sure we specify correct parameters
  // Input/Output files
  if (FLAGS_input == "" || FLAGS_output == "") {
    std::cout << "Must specify input and output files via -input and -output. "
              << "Also see: -help." << std::endl;
    return 1;
  }
  // Output file format
  if (FLAGS_output_type != "fplan" && FLAGS_output_type != "csv" && FLAGS_output_type != "bin") {
    std::cout << "The output type must be 'fplan', 'csv', or 'bin'." << std::endl;
    return 1;
  }
  // Define multiplication order
  ROTATIONS_MULTIPLICATION_ORDER order;
  if (FLAGS_rotations_multiplication_order == "roll-pitch-yaw")
    order = ROLL_PITCH_YAW;
  else if (FLAGS_rotations_multiplication_order == "yaw-pitch-roll")
    order = YAW_PITCH_ROLL;
  else
    LOG(FATAL) << "Unknown value for -rotations_multiplication_order: "
               << FLAGS_rotations_multiplication_order;

  // READING ----------------------------------------------------------------------
  std::cout << "Reading: " << FLAGS_input << std::endl;
  std::ifstream ifs(FLAGS_input.c_str());
  // Open the input file
  if (!ifs.is_open()) {
    std::cout << "Could not open file: " << FLAGS_input << std::endl;
    return 1;
  }

  std::vector<Eigen::Affine3d> Tf;
  std::vector<Eigen::VectorXd> Poses;
  std::vector<Eigen::Vector3d> Arms;
  std::string line;
  int arm_init = -1;
  while (getline(ifs, line)) {
    if (has_only_whitespace_or_comments(line)) continue;

    // Arm commands
    std::istringstream is(line);
    Eigen::VectorXd Pose(6);
    bool arm_flag = true;
    Eigen::Vector3d Arm = Eigen::Vector3d::Zero();
    if (!(is >> Pose[0] >> Pose[1] >> Pose[2] >> Pose[3] >> Pose[4] >> Pose[5]
                                                      >> Arm[0] >> Arm[1] >> Arm[2])) {
      arm_flag = false;
      // No arm commands
      std::istringstream is(line);
      if (!(is >> Pose[0] >> Pose[1] >> Pose[2] >> Pose[3] >> Pose[4] >> Pose[5])) {
        std::cout << "Ignoring invalid line: " << line  << std::endl;
        continue;
      }
    }
    // std::cout << Pose[0]  << " " << Pose[1]  << " " << Pose[2] << " " << Pose[3] <<
    // " " << Pose[4] << " " << Pose[5] << " " << Arm[0] << " " << Arm[1] << std::endl;

    Eigen::Affine3d tf;
    tf.translation() = Eigen::Vector3d(Pose[0], Pose[1], Pose[2]);
    tf.linear()      = roll_pitch_yaw_to_quaternion(order, Pose[3],
                                                    Pose[4], Pose[5]).toRotationMatrix();
    Tf.push_back(tf);
    Poses.push_back(Pose);


    // If the arm values were successfull read from the input file
    if (arm_flag) {
      Arms.push_back(Arm);                // Adds the last known Joint Values
      if (arm_init == -1)
        arm_init = Poses.size() - 1;
    } else if (!Arms.empty()) {
      Arms.push_back(Arms.back());        // Adds the last known Joint Values
    }
  }

  // WRITING ----------------------------------------------------------------------
  std::cout << "Writing: " << FLAGS_output << std::endl;
  std::ofstream ofs(FLAGS_output.c_str());
  ofs.precision(15);
  if (FLAGS_output_type == "fplan")
    jsonloader::WritePlanHeader(ofs, FLAGS_vel, FLAGS_accel, FLAGS_omega, FLAGS_alpha, FLAGS_creator);

  // Create motion goal
  ff_msgs::MotionGoal goal;
  goal.command = ff_msgs::MotionGoal::EXEC;
  sensor_msgs::JointState joint;

  if (!Arms.empty()) {
    joint.name.resize(3);
    joint.name[0] = "pan";
    joint.name[1] = "tilt";
    joint.name[2] = "gripper";
    joint.position.resize(3);
  }

  ros::Time station_time(0);  // Start at this time
  ros::Time old_station_time(0);  // Start at this time
  double time_percentage;
  for (size_t id = 0; id < Poses.size(); id++) {
    old_station_time = station_time;
    if (FLAGS_output_type == "fplan")
      jsonloader::WriteStation(ofs, Poses[id], FLAGS_tolerance, id);

    if (id + 1 == Poses.size())
      break;  // Finished writing the last station

    // Write the segment connecting to the next station
    if (FLAGS_output_type == "fplan" || FLAGS_output_type == "csv")
      station_time = ros::Time(0);  // Start at this time
    ff_util::Segment segment;
    double dt = 0, min_control_period = 1.0, epsilon = 0.001;
    planner_trapezoidal::InsertTrapezoid(segment,       // output
                                         station_time,  // this will be incremented
                                         dt, Tf[id], Tf[id+1],
                                         FLAGS_vel, FLAGS_omega, FLAGS_accel, FLAGS_alpha,
                                         min_control_period, epsilon);

    // Export the segment to a more plain format
    ff_util::Segment::const_iterator it;
    if (FLAGS_output_type == "fplan") {
      std::vector<Eigen::VectorXd> SegVec;
      for (it = segment.begin(); it != segment.end(); it++) {
        Eigen::VectorXd S(20);
        S << (it->when).toSec(), it->pose.position.x, it->pose.position.y, it->pose.position.z,
          it->twist.linear.x, it->twist.linear.y, it->twist.linear.z,
          it->accel.linear.x, it->accel.linear.y, it->accel.linear.z,
          it->pose.orientation.x, it->pose.orientation.y,
          it->pose.orientation.z, it->pose.orientation.w,
          it->twist.angular.x, it->twist.angular.y, it->twist.angular.z,
          it->accel.angular.x, it->accel.angular.y, it->accel.angular.z;
        SegVec.push_back(S);
      }
      jsonloader::WriteSegment(ofs, SegVec, FLAGS_vel, FLAGS_accel, FLAGS_omega, FLAGS_alpha, id);
    } else if (FLAGS_output_type == "csv") {
      // Write accelerations only
      for (it = segment.begin(); it != segment.end(); it++) {
        ofs << (it->when - segment.begin()->when).toSec()
            << "," << it->accel.linear.x
            << "," << it->accel.linear.y
            << "," << it->accel.linear.z
            << "," << it->accel.angular.x
            << "," << it->accel.angular.y
            << "," << it->accel.angular.z
            << std::endl;
      }
    } else if (FLAGS_output_type == "bin") {
    std::cout << "SEGMENT " << id << std::endl;
      // Add segment to plan
      for (it = segment.begin(); it != segment.end(); it++) {
        goal.segment.push_back(*it);
        if ((!Arms.empty()) && (arm_init <= static_cast<int>(id))) {
          time_percentage = (it->when.toSec() - old_station_time.toSec())/
                                        (station_time.toSec() - old_station_time.toSec());
          joint.header.stamp = it->when;
          joint.position[0] = Arms[id-arm_init](0)-(time_percentage)*(Arms[id-arm_init](0) - Arms[id+1-arm_init](0));
          joint.position[1] = Arms[id-arm_init](1)-(time_percentage)*(Arms[id-arm_init](1) - Arms[id+1-arm_init](1));
          joint.position[2] = Arms[id-arm_init](2)-(time_percentage)*(Arms[id-arm_init](2) - Arms[id+1-arm_init](2));
          goal.arm.push_back(joint);
          // std::cout << it->when.toSec()  << "\t " << joint.position[0]  << "\t " << joint.position[1] << std::endl;
        }
      }
      // Pause in between segments
      station_time += ros::Duration(2);
    }
  }

  // Let the plan name be the output file name without the dot and dir name
  std::string plan_name = FLAGS_output;
  std::size_t dot_pos = plan_name.rfind(".");
  if (dot_pos != std::string::npos)
    plan_name = plan_name.substr(0, dot_pos);
  std::size_t slash_pos = plan_name.rfind("/");
  if (slash_pos != std::string::npos)
    plan_name = plan_name.substr(slash_pos + 1, std::string::npos);
  if (plan_name == "")
    plan_name = FLAGS_output;  // if we really fail

  size_t id = Poses.size();
  if (FLAGS_output_type == "fplan")
    jsonloader::WritePlanFooter(ofs, plan_name, id);

  if (ofs.is_open()) {
    ofs.close();
  } else {
    std::cout << "Could not write: " << FLAGS_output << std::endl;
  }

  // Write ros message using the stadard function
  if (FLAGS_output_type == "bin") {
    if (ff_util::Serialization::WriteFile(FLAGS_output, goal))
      std::cout << std::endl << "Segment saved to " << FLAGS_output << "\n";
    else
      std::cout << std::endl << "Segment not saved\n";
  }

  return 0;
}
