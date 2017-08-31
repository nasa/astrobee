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

#ifndef FF_UTIL_FF_FLIGHT_H_
#define FF_UTIL_FF_FLIGHT_H_

// ROS includes
#include <ros/ros.h>

// Message includes
#include <ff_msgs/EkfState.h>
#include <ff_msgs/ControlState.h>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// STL includes
#include <vector>
#include <string>
#include <map>

namespace ff_util {

// Convenience declarations
typedef ff_msgs::ControlState Setpoint;
typedef ff_msgs::EkfState Estimate;
typedef std::vector < Setpoint > Segment;

class State {
 public:
  // Empty state
  State();
  // Create a state from a ff_msgs::ControlState message
  explicit State(Setpoint const& msg);
  // Create a state from a ff_msgs::ControlState message
  explicit State(Estimate const& msg);
  // Destructor
  virtual ~State();
  // Assignment operator
  void operator=(State const& right);
  // Minus operator
  State operator-(State const& right);
  // Propagate the state forward by a dome time step
  State Propagate(double dt);
  // Convert the state to a setpoint
  Setpoint ToSetpoint();
  // Return the magnitude of a rotation
  double QuaternionMagnitude();
    // Useful helper functions
  static double QuaternionMagnitude(Eigen::Quaterniond const& iq);
  static double VectorMagnitude(Eigen::Vector3d const& iv);

 protected:
  // Generate a strapdown equation for a given quaternion
  Eigen::Matrix4d Omega(Eigen::Vector3d const& vec);

 public:
  double t;               // Time
  Eigen::Quaterniond q;   // Orientation
  Eigen::Vector3d p;      // Position
  Eigen::Vector3d w;      // Omega
  Eigen::Vector3d v;      // Velocity
  Eigen::Vector3d b;      // Alpha
  Eigen::Vector3d a;      // Acceleration
};

//////////////////////////////////////
// CONFIG FILE FUNCTIONS FOR FLIGHT //
//////////////////////////////////////

// Flight mode
struct FlightMode {
  std::string name;
  // GNC gains specific to the flight mode
  Eigen::Vector3d att_kp;
  Eigen::Vector3d att_ki;
  Eigen::Vector3d omega_kd;
  Eigen::Vector3d pos_kp;
  Eigen::Vector3d pos_ki;
  Eigen::Vector3d vel_kd;
  // Kinematic limits
  double hard_limit_omega;
  double hard_limit_vel;
  double hard_limit_alpha;
  double hard_limit_accel;
  // Tolerances
  double tolerance_time;
  double tolerance_att;
  double tolerance_pos;
  double tolerance_omega;
  double tolerance_vel;
};

// Vector of flight modes
typedef std::map < std::string, FlightMode > FlightModeMap;

// Inertia configuration
class InertiaConfig {
 public:
  InertiaConfig() : mass(-1.0) {}
  double mass;                       // Mass
  Eigen::Matrix3d inertia_matrix;    // Inertia matrix
  Eigen::Vector3d center_of_mass;    // Center of mass
};

// General configuration
class GeneralConfig {
 public:
  GeneralConfig() : default_flight_mode("") {}
  std::string default_flight_mode;   // The current flight mode
  double min_control_rate;           // Minimum acceptable control period
  double time_sync_threshold;        // Threshold for detecting time sync issues
};

// Class to read flight mode information
class ConfigUtil {
 public:
  // Get all general configuration (LUA config file is queried only once)
  static bool GetGeneralConfig(GeneralConfig & gc_out);
  // Get all inertia configuration (LUA config file is queried only once)
  static bool GetInertiaConfig(InertiaConfig & ic_out);
  // Get data for a given flight mode
  static bool GetFlightMode(std::string const& name, FlightMode &data);
};

////////////////////////////////////////
// SEGMENT CHECKING UTILITY FUNCTIONS //
////////////////////////////////////////

// Segment check bitmask
enum SegmentCheckMask : unsigned int {
  CHECK_NONE                      = 0x000,
  CHECK_MINIMUM_FREQUENCY         = 0x001,
  CHECK_STATIONARY_ENDPOINT       = 0x002,
  CHECK_FIRST_TIMESTAMP_IN_PAST   = 0x004,
  CHECK_MINIMUM_NUM_SETPOINTS     = 0x008,
  CHECK_LIMITS_VEL                = 0x010,
  CHECK_LIMITS_ACCEL              = 0x020,
  CHECK_LIMITS_OMEGA              = 0x040,
  CHECK_LIMITS_ALPHA              = 0x080,
  CHECK_ALL_BASIC                 = 0x100,
  CHECK_ALL_LIMITS                = 0x200,
  CHECK_ALL_CONSISTENCY           = 0x400,
  CHECK_ALL                       = 0x800
};

// Segment result bitmask
enum SegmentResult : int {
  SUCCESS                         =  0,
  ERROR_MINIMUM_FREQUENCY         = -1,
  ERROR_STATIONARY_ENDPOINT       = -2,
  ERROR_MINIMUM_NUM_SETPOINTS     = -3,
  ERROR_TIME_RUNS_BACKWARDS       = -4,
  ERROR_LIMITS_VEL                = -5,
  ERROR_LIMITS_ACCEL              = -6,
  ERROR_LIMITS_OMEGA              = -7,
  ERROR_LIMITS_ALPHA              = -8,
  ERROR_INVALID_FLIGHT_MODE       = -9,
  ERROR_INVALID_GENERAL_CONFIG    = -10
};

// Class to read flight mode information
class SegmentUtil {
 public:
  // Validate if an upper limit has been breached
  static double ValidateUpperLimit(const double limit, double val);
  // Validate if a lower limit has been breached
  static double ValidateLowerLimit(const double limit, double val);
  // Perform a given set of checks on a segment for validity, based on a flight mode
  static SegmentResult Check(SegmentCheckMask mask, Segment const& segment, std::string const& fm);
  // Resample a segment to the minimum control frequency
  static SegmentResult Resample(Segment const& in, Segment & out, double rate = -1.0);
  // Print a human-readable description of the error code
  static std::string GetDescription(SegmentResult result);
  // Logical comparison of two setpoints to evaluate equality within defined EPSILON value
  static bool Equal(Setpoint const& left, Setpoint const& right, size_t degree = 2);
  // Load a flight mode and segment from a file
  static bool Load(std::string const& fname, std::string & flight_mode, Segment & segment);
  // Save a flight mode and segment from to file
  static bool Save(std::string const& fname, std::string const& flight_mode, Segment const& segment);
};

}  // end namespace ff_util

#endif  // FF_UTIL_FF_FLIGHT_H_
