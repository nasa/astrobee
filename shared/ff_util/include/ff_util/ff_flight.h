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

// General information
#include <geometry_msgs/Inertia.h>

// Message includes
#include <ff_msgs/EkfState.h>
#include <ff_msgs/ControlState.h>
#include <ff_msgs/FlightMode.h>

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
typedef std::vector<Setpoint> Segment;

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

  // Convert the state to a setpoint
  Setpoint ToSetpoint();

  // Return the magnitude of a rotation
  double QuaternionMagnitude();

  // Useful helper functions
  static double QuaternionMagnitude(Eigen::Quaterniond const& iq);
  static double VectorMagnitude(Eigen::Vector3d const& iv);

 public:
  double t;               // Time
  Eigen::Quaterniond q;   // Orientation
  Eigen::Vector3d p;      // Position
  Eigen::Vector3d w;      // Omega
  Eigen::Vector3d v;      // Velocity
  Eigen::Vector3d b;      // Alpha
  Eigen::Vector3d a;      // Acceleration
};

////////////////////////////////////////
// SEGMENT CHECKING UTILITY FUNCTIONS //
////////////////////////////////////////

// Segment check bitmask
enum SegmentCheckMask : uint32_t {
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
enum SegmentResult : int32_t {
  SUCCESS                         =  0,
  ERROR_MINIMUM_FREQUENCY         = -1,
  ERROR_STATIONARY_ENDPOINT       = -2,
  ERROR_MINIMUM_NUM_SETPOINTS     = -3,
  ERROR_TIME_RUNS_BACKWARDS       = -4,
  ERROR_LIMITS_VEL                = -5,
  ERROR_LIMITS_ACCEL              = -6,
  ERROR_LIMITS_OMEGA              = -7,
  ERROR_LIMITS_ALPHA              = -8
};

// Class to read flight mode information
class FlightUtil {
 public:
  // Vector of flight modes
  typedef std::map<std::string, ff_msgs::FlightMode> FlightModeMap;

  // Minimum acceptable control rate
  static constexpr double MIN_CONTROL_RATE = 1.0;

  // Get all inertia configuration (LUA config file is queried only once)
  static bool GetInertiaConfig(geometry_msgs::Inertia & data);

  // Get data for a given flight mode (leave empty for default value)
  static bool GetFlightMode(
    ff_msgs::FlightMode &data, std::string const& name = "");

  // Get the intial flight mode
  static bool GetInitialFlightMode(ff_msgs::FlightMode &data);

  // Validate if an upper limit has been breached
  static double ValidateUpperLimit(const double limit, double val);

  // Validate if a lower limit has been breached
  static double ValidateLowerLimit(const double limit, double val);

  // Perform checks on a segment for validity, based on a flight mode
  static SegmentResult Check(SegmentCheckMask mask, Segment const& segment,
    ff_msgs::FlightMode const& flight_mode, bool faceforward);

  // Resample a segment to the minimum control frequency
  static SegmentResult Resample(Segment const& in, Segment & out,
    double rate = -1.0);

  // Print a human-readable description of the error code
  static std::string GetDescription(SegmentResult result);

  // Logical comparison of two setpoints within defined EPSILON value
  static bool Equal(Setpoint const& left, Setpoint const& right,
    size_t degree = 2);

  // Check that the first pose is consistent with the segment/flight mode
  static bool WithinTolerance(ff_msgs::FlightMode const& fm,
    geometry_msgs::Pose const& a, geometry_msgs::Pose const& b);
};

}  // end namespace ff_util

#endif  // FF_UTIL_FF_FLIGHT_H_
