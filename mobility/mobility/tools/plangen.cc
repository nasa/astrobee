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
#include <jsonloader/planio.h>

// Primitive actions
#include <ff_msgs/PlanAction.h>

// For the trapezoidal planner implementation
#include <planner_trapezoidal/planner_trapezoidal.h>

// C++ STL inclues
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <ctime>

// Gflags
// DEFINE_string(planner, "trapezoidal", "Planner name (trapezoidal, qp)");
DEFINE_string(input, "", "Input file with rows of type: x y z roll pitch yaw (in degrees).");
DEFINE_string(output, "", "Output file.");
DEFINE_string(output_type, "fplan", "Output file type. Options: fplan (default) and csv. "
              "It will write either a JSON file with .fplan extension to be used in GDS, or a .csv file "
              "with only accelerations to be used in MGTF.");
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
  common::InitFreeFlyerApplication(&argc, &argv);

  if (FLAGS_input == "" || FLAGS_output == "") {
    std::cout << "Must specify input and output files via -input and -output. "
              << "Also see: -help." << std::endl;
    return 1;
  }

  if (FLAGS_output_type != "fplan" && FLAGS_output_type != "csv") {
    std::cout << "The output type must be 'fplan' or 'csv'." << std::endl;
    return 1;
  }

  ROTATIONS_MULTIPLICATION_ORDER order;
  if (FLAGS_rotations_multiplication_order == "roll-pitch-yaw")
    order = ROLL_PITCH_YAW;
  else if (FLAGS_rotations_multiplication_order == "yaw-pitch-roll")
    order = YAW_PITCH_ROLL;
  else
    LOG(FATAL) << "Unknown value for -rotations_multiplication_order: "
               << FLAGS_rotations_multiplication_order;

  // Read the input file and create the affine transforms among which we will
  // insert trapezoids.

  std::cout << "Reading: " << FLAGS_input << std::endl;
  std::ifstream ifs(FLAGS_input.c_str());
  if (!ifs.is_open()) {
    std::cout << "Could not open file: " << FLAGS_input << std::endl;
    return 1;
  }

  std::vector<Eigen::Affine3d> Tf;
  std::vector<Eigen::VectorXd> Poses;
  std::string line;
  while (getline(ifs, line)) {
    if (has_only_whitespace_or_comments(line)) continue;

    std::istringstream is(line);
    Eigen::VectorXd Pose(6);
    if (!(is >> Pose[0] >> Pose[1] >> Pose[2] >> Pose[3] >> Pose[4] >> Pose[5])) {
      std::cout << "Ignoring invalid line: " << line  << std::endl;
      continue;
    }

    Eigen::Affine3d tf;
    tf.translation() = Eigen::Vector3d(Pose[0], Pose[1], Pose[2]);
    tf.linear()      = roll_pitch_yaw_to_quaternion(order, Pose[3],
                                                    Pose[4], Pose[5]).toRotationMatrix();
    Tf.push_back(tf);
    Poses.push_back(Pose);
  }

  std::cout << "Writing: " << FLAGS_output << std::endl;
  std::ofstream ofs(FLAGS_output.c_str());
  ofs.precision(15);
  if (FLAGS_output_type == "fplan")
    jsonloader::WritePlanHeader(ofs, FLAGS_vel, FLAGS_accel, FLAGS_omega, FLAGS_alpha, FLAGS_creator);

  for (size_t id = 0; id < Poses.size(); id++) {
    if (FLAGS_output_type == "fplan")
      jsonloader::WriteStation(ofs, Poses[id], FLAGS_tolerance, id);

    if (id + 1 == Poses.size())
      break;  // Finished writing the last station

    // Write the segment connecting to the next station
    ros::Time station_time(0);  // Start at this time
    ff_util::Segment segment;
    double dt = 0, min_control_period = 1.0, epsilon = 0.001;
    planner_trapezoidal::InsertTrapezoid(segment,       // output
                                         station_time,  // this will be incremented
                                         dt, Tf[id], Tf[id+1],
                                         FLAGS_vel, FLAGS_omega, FLAGS_accel, FLAGS_alpha,
                                         min_control_period, epsilon);

    // Export the segment to a more plain format
    std::vector<Eigen::VectorXd> SegVec;
    ff_util::Segment::const_iterator it;
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
    if (FLAGS_output_type == "fplan") {
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

  return 0;
}
