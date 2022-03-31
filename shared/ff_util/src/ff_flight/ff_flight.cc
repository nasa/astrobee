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

#include <ff_util/ff_flight.h>

#include <unsupported/Eigen/MatrixFunctions>

#include <msg_conversions/msg_conversions.h>

// STL includes
#include <fstream>
#include <string>

// Constants
#define EPSILON            1e-6
#define THRESHOLD_METERS   0.01
#define THRESHOLD_RADIAN   0.01
#define INVALID_VALUE      -1.0

namespace ff_util {

  //////////////////////////////////////////////////////////////////////
  // MODELLING AND SECOND ORDER KINEMATIC MODEL FOR A 6DoF RIGID BODY //
  //////////////////////////////////////////////////////////////////////

  State::State() {}

  State::State(Setpoint const& msg) {
    t = msg.when.toSec();
    q = msg_conversions::ros_to_eigen_quat(msg.pose.orientation);
    p = msg_conversions::ros_point_to_eigen_vector(msg.pose.position);
    w = msg_conversions::ros_to_eigen_vector(msg.twist.angular);
    v = msg_conversions::ros_to_eigen_vector(msg.twist.linear);
    b = msg_conversions::ros_to_eigen_vector(msg.accel.angular);
    a = msg_conversions::ros_to_eigen_vector(msg.accel.linear);
  }

  State::State(Estimate const& msg)  {
    t = msg.header.stamp.toSec();
    q = msg_conversions::ros_to_eigen_quat(msg.pose.orientation);
    p = msg_conversions::ros_point_to_eigen_vector(msg.pose.position);
    w = msg_conversions::ros_to_eigen_vector(msg.omega);
    v = msg_conversions::ros_to_eigen_vector(msg.velocity);
  }

  State::~State() {}

  void State::operator=(State const& right) {
    t = right.t;
    q = right.q;
    p = right.p;
    w = right.w;
    v = right.v;
    b = right.b;
    a = right.a;
  }

  State State::operator-(State const& right) {
    State left;
    left.t = t - right.t;
    left.q = right.q * q.inverse();
    left.p = p - right.p;
    left.w = w - right.w;
    left.v = v - right.v;
    left.b = b - right.b;
    left.a = a - right.a;
    return left;
  }

  // Convert the state to a setpoint
  Setpoint State::ToSetpoint() {
    Setpoint sp;
    sp.when = ros::Time(t);
    sp.pose.orientation = msg_conversions::eigen_to_ros_quat(q);
    sp.pose.position = msg_conversions::eigen_to_ros_point(p);
    sp.twist.angular = msg_conversions::eigen_to_ros_vector(w);
    sp.twist.linear = msg_conversions::eigen_to_ros_vector(v);
    sp.accel.angular = msg_conversions::eigen_to_ros_vector(b);
    sp.accel.linear = msg_conversions::eigen_to_ros_vector(a);
    return sp;
  }

  // Return the magnitude of the rotation
  double State::QuaternionMagnitude() {
    return QuaternionMagnitude(q);
  }

  // Return the magnitude of the rotation
  double State::QuaternionMagnitude(Eigen::Quaterniond const& iq) {
    double x = Eigen::AngleAxisd(iq.normalized()).angle();
    x = fmod(x + M_PI, 2.0 * M_PI);
    if (x < 0)
      x += 2.0 * M_PI;
    x -= M_PI;
    return fabs(x);
  }

  // Return the magnitude of the rotation
  double State::VectorMagnitude(Eigen::Vector3d const& iv) {
    return iv.norm();
  }

  /////////////////////////////////////////////////////////////////////
  // FLIGHT UTILITIES SHARED BETWEEN THE PLANNING AND GNC SUBSYSTEMS //
  /////////////////////////////////////////////////////////////////////

  // Get and cache default inertia configuration
  bool FlightUtil::GetInertiaConfig(geometry_msgs::InertiaStamped &data) {
    static geometry_msgs::InertiaStamped inertia;
    if (inertia.inertia.m == 0) {
      config_reader::ConfigReader cfg;
      config_reader::ConfigReader::Table com;
      float inertia_matrix[9];
      cfg.AddFile("flight.config");
      cfg.ReadFiles();
      if (   !cfg.GetReal("inertia_mass", &inertia.inertia.m)
          || !cfg.GetStr("inertia_name", &inertia.header.frame_id)
          || !msg_conversions::config_read_matrix(&cfg, "inertia_matrix", 3, 3,
                inertia_matrix)
          || !cfg.GetTable("inertia_com", &com)
          || !msg_conversions::config_read_vector(&com, &inertia.inertia.com))
        return false;
      inertia.inertia.ixx = inertia_matrix[0];
      inertia.inertia.ixy = inertia_matrix[1];
      inertia.inertia.ixz = inertia_matrix[2];
      inertia.inertia.iyy = inertia_matrix[4];
      inertia.inertia.iyz = inertia_matrix[5];
      inertia.inertia.izz = inertia_matrix[8];
    }
    // Copy the data and return successful
    data = inertia;
    return true;
  }

  // Get and cache data for a given flight mode
  bool FlightUtil::GetFlightMode(ff_msgs::FlightMode &data,
    std::string const& name) {
    // make sure we have the flight modes
    static FlightModeMap fm;
    static std::string default_flight_mode = "";
    if (fm.empty()) {
      config_reader::ConfigReader cfg;
      cfg.AddFile("flight.config");
      cfg.ReadFiles();
      // Get the default flight mode
      if (!cfg.GetStr("default_flight_mode", &default_flight_mode))
        return false;
      // Get the list of flight modes
      config_reader::ConfigReader::Table flight_modes, mode, tmp;
      if (!cfg.GetTable("flight_modes", &flight_modes))
        return false;
      for (int j = 0; j < flight_modes.GetSize(); j++) {
        static ff_msgs::FlightMode info;
        unsigned int speed;
        bool control_enabled;
        if (   !flight_modes.GetTable(j + 1, &mode)
            || !mode.GetStr("name", &info.name)
            || !mode.GetBool("control_enabled", &control_enabled)
            || !mode.GetReal("hard_limit_omega", &info.hard_limit_omega)
            || !mode.GetReal("hard_limit_vel", &info.hard_limit_vel)
            || !mode.GetReal("hard_limit_alpha", &info.hard_limit_alpha)
            || !mode.GetReal("hard_limit_accel", &info.hard_limit_accel)
            || !mode.GetReal("tolerance_pos_endpoint", &info.tolerance_pos_endpoint)
            || !mode.GetReal("tolerance_pos", &info.tolerance_pos)
            || !mode.GetReal("tolerance_att", &info.tolerance_att)
            || !mode.GetReal("tolerance_omega", &info.tolerance_omega)
            || !mode.GetReal("tolerance_vel", &info.tolerance_vel)
            || !mode.GetReal("tolerance_time", &info.tolerance_time)
            || !mode.GetUInt("speed", &speed)
            || !mode.GetTable("att_kp", &tmp)
            || !msg_conversions::config_read_vector(&tmp, &info.att_kp)
            || !mode.GetTable("att_ki", &tmp)
            || !msg_conversions::config_read_vector(&tmp, &info.att_ki)
            || !mode.GetTable("omega_kd", &tmp)
            || !msg_conversions::config_read_vector(&tmp, &info.omega_kd)
            || !mode.GetTable("pos_kp", &tmp)
            || !msg_conversions::config_read_vector(&tmp, &info.pos_kp)
            || !mode.GetTable("pos_ki", &tmp)
            || !msg_conversions::config_read_vector(&tmp, &info.pos_ki)
            || !mode.GetTable("vel_kd", &tmp)
            || !msg_conversions::config_read_vector(&tmp, &info.vel_kd))
          return false;
        // Check for out of bound speed gain value
        if (speed < ff_msgs::FlightMode::SPEED_MIN
         || speed > ff_msgs::FlightMode::SPEED_MAX) return false;
        // Copy over boolean
        info.control_enabled = control_enabled;
        // Should be safe to cast the speed gain now
        info.speed = static_cast<uint8_t>(speed);
        // Save the flight mode info
        fm[info.name] = info;
      }
      // Check that the default flight mode exists
      if (fm.find(default_flight_mode) == fm.end())
        return false;
    }
    // If no name was specified, grab the default parameters
    if (name.empty()) {
      data = fm[default_flight_mode];
      return true;
    }
    // Check that this flight mode exists
    if (fm.find(name) == fm.end())
      return false;
    // Extract the flight mode info
    data = fm[name];
    return true;
  }

  // Get the intial flight mode
  bool FlightUtil::GetInitialFlightMode(ff_msgs::FlightMode &data) {
    std::string name = "";
    config_reader::ConfigReader cfg;
    cfg.AddFile("flight.config");
    cfg.ReadFiles();
    if (!cfg.GetStr("initial_flight_mode", &name))
      return false;
    return GetFlightMode(data, name);
  }

  // Validate if an upper limit has been breached
  double FlightUtil::ValidateUpperLimit(const double limit, const double val) {
    // Special case: negative numbers imply saturation
    if (val < 0)
      return limit;
    // Are we less than limit + epsilon?
    return (val - limit < EPSILON ? val : INVALID_VALUE);
  }

  // Validate if a lower limit has been breached
  double FlightUtil::ValidateLowerLimit(const double limit, const double val) {
    // Special case: negative numbers imply saturation
    if (val < 0)
      return limit;
    // Are we less than limit + epsilon?
    return (limit - val < EPSILON ? val : INVALID_VALUE);
  }

  // The code to o
  SegmentResult FlightUtil::Check(SegmentCheckMask mask, Segment const& segment,
    ff_msgs::FlightMode const& flight_mode, bool faceforward) {
    ////////////////////////////////////////////////////////////////////////////
    // Check 1 : Always check that we have the minimum number of set points   //
    //           If we didn't check this, other tests would fail              //
    ////////////////////////////////////////////////////////////////////////////
    if (segment.size() < 1)
      return ERROR_MINIMUM_NUM_SETPOINTS;
    // Iterate over all segments
    for (Segment::const_iterator it = segment.begin();
      it != segment.end(); it++) {
      State c_s(*it);
      //////////////////////////////////////////////////////////////////////////
      // Check 2 : Check that the final setpoint in the segment has a zero    //
      //           velocity. If it doesn't, then the controller will drift!   //
      //////////////////////////////////////////////////////////////////////////
      if (it + 1 == segment.end()) {
        if (mask & CHECK_STATIONARY_ENDPOINT) {
          if (State::VectorMagnitude(c_s.w) != 0.0
           || State::VectorMagnitude(c_s.v) != 0.0
           || State::VectorMagnitude(c_s.a) != 0.0
           || State::VectorMagnitude(c_s.b) != 0.0)
            return ERROR_STATIONARY_ENDPOINT;
        }
        break;
      }
      ///////////////////////////////////////////////////////////////////////////
      // Check 3 : Check limits on acceleration and velocities                 //
      ///////////////////////////////////////////////////////////////////////////
      double divider = 1.0;
      if (!faceforward)
        divider = 2.0;
      if ((mask & CHECK_LIMITS_OMEGA)
        && ValidateUpperLimit(flight_mode.hard_limit_omega,
          State::VectorMagnitude(c_s.w)) < 0)
        return ERROR_LIMITS_OMEGA;
      if ((mask & CHECK_LIMITS_VEL)
        && ValidateUpperLimit(flight_mode.hard_limit_vel,
          State::VectorMagnitude(c_s.v)) < 0)
        return ERROR_LIMITS_VEL;
      if ((mask & CHECK_LIMITS_ALPHA)
        && ValidateUpperLimit(flight_mode.hard_limit_alpha / divider,
          State::VectorMagnitude(c_s.b)) < 0)
        return ERROR_LIMITS_ALPHA;
      if ((mask & CHECK_LIMITS_ACCEL)
        && ValidateUpperLimit(flight_mode.hard_limit_accel / divider,
          State::VectorMagnitude(c_s.a)) < 0)
        return ERROR_LIMITS_ACCEL;
      //////////////////////////////////////////////////////////////////////////
      // Check 4 : Check that we have a delta t in the range [0, timeout]     //
      //////////////////////////////////////////////////////////////////////////
      State n_s(*(it + 1));
      State diff = n_s - c_s;
      if (diff.t < -EPSILON)
        return ERROR_TIME_RUNS_BACKWARDS;
      if (diff.t - MIN_CONTROL_RATE > EPSILON)
        return ERROR_MINIMUM_FREQUENCY;
      if (diff.t < EPSILON)
        continue;
    }
    // This segment is valid
    return SUCCESS;
  }

  // Resample a segment to a minimum control rate
  SegmentResult FlightUtil::Resample(Segment const& in, Segment & out,
    double rate) {
    // Check that we have enough setpoints
    if (in.size() < 2) {
      out = in;
      return SUCCESS;
    }
    // If the rate was not specified, try and get it from the general config
    if (rate < MIN_CONTROL_RATE)
      rate = MIN_CONTROL_RATE;
    // Clear the output
    out.clear();
    // Perform the resample of the segment - we are expecting as an input the
    // four points of trapezoid. What we want as an output is the segment
    Segment::const_iterator it;
    Segment::const_iterator jt;
    // ROS_INFO_STREAM("***");
    for (Segment::const_iterator it = in.begin(); it != in.end(); it++) {
      // Convert the angular velocity and angular acceleration from the
      // world to the body frame, avoiding a GDS / FSW incompatibility
      State is(*it);
      // Do we have an end setpoint?
      jt = std::next(it);
      if (jt == in.end()) {
        out.push_back(is.ToSetpoint());
      } else {
        // State at start end of setpoint (we just need the timestamp)
        State js(*jt);
        // Debug for clarity
        /*
        ROS_INFO_STREAM("---");
        ROS_INFO_STREAM("PREV: "
                          << " t: "  << is.t
                          << " qw: " << is.q.w()
                          << " qx: " << is.q.x()
                          << " qy: " << is.q.y()
                          << " qz: " << is.q.z()
                          << " wx: " << is.w[0]
                          << " wy: " << is.w[1]
                          << " wz: " << is.w[2]
                          << " bx: " << is.b[0]
                          << " by: " << is.b[1]
                          << " bz: " << is.b[2]);
        ROS_INFO_STREAM("NEXT: "
                          << " t: "  << js.t
                          << " qw: " << js.q.w()
                          << " qx: " << js.q.x()
                          << " qy: " << js.q.y()
                          << " qz: " << js.q.z()
                          << " wx: " << js.w[0]
                          << " wy: " << js.w[1]
                          << " wz: " << js.w[2]
                          << " bx: " << js.b[0]
                          << " by: " << js.b[1]
                          << " bz: " << js.b[2]);
        */
        // Get the discrete time step
        double tsamp = 1.0 / rate;    // Trajectory sampling rate 1Hz
        double nsamp = 0.01;          // Numerical sampling rate 100Hz
        // Add the first
        State state = is;
        /*
        ROS_INFO_STREAM("SAMP: "
                          << " t: "  << state.t
                          << " qw: " << state.q.w()
                          << " qx: " << state.q.x()
                          << " qy: " << state.q.y()
                          << " qz: " << state.q.z()
                          << " wx: " << state.w[0]
                          << " wy: " << state.w[1]
                          << " wz: " << state.w[2]
                          << " bx: " << state.b[0]
                          << " by: " << state.b[1]
                          << " bz: " << state.b[2]);
        */
        out.push_back(state.ToSetpoint());
        // Now resample
        while (state.t < js.t) {
          double tend = (state.t + tsamp < js.t ? state.t + tsamp : js.t);
          while (state.t < tend) {
            double dt = (state.t + nsamp < tend ? nsamp : tend - state.t);
            Eigen::Matrix4d omega;
            omega << 0.0,        state.w[2], -state.w[1], state.w[0],
                    -state.w[2], 0.0,         state.w[0], state.w[1],
                     state.w[1], -state.w[0], 0.0,        state.w[2],
                    -state.w[0], -state.w[1], -state.w[2], 0.0;
            Eigen::Vector4d qdiff(state.q.x(), state.q.y(), state.q.z(), state.q.w());
            qdiff = omega * qdiff * 0.5;
            state.q.x() += qdiff[0] * dt;
            state.q.y() += qdiff[1] * dt;
            state.q.z() += qdiff[2] * dt;
            state.q.w() += qdiff[3] * dt;
            state.q.normalize();
            state.p += state.v * dt;
            state.w += state.b * dt;
            state.v += state.a * dt;
            state.t += dt;
          }
          /*
          ROS_INFO_STREAM("SAMP: "
                            << " t: "  << state.t
                            << " qw: " << state.q.w()
                            << " qx: " << state.q.x()
                            << " qy: " << state.q.y()
                            << " qz: " << state.q.z()
                            << " wx: " << state.w[0]
                            << " wy: " << state.w[1]
                            << " wz: " << state.w[2]
                            << " bx: " << state.b[0]
                            << " by: " << state.b[1]
                            << " bz: " << state.b[2]);
          */
          out.push_back(state.ToSetpoint());
        }
      }
    }
    // Success
    return SUCCESS;
  }

  // Print a human-readable description of the error code
  std::string FlightUtil::GetDescription(SegmentResult result) {
    switch (result) {
    case SUCCESS:
      return std::string("SUCCESS");
    case ERROR_MINIMUM_FREQUENCY:
      return std::string("ERROR_MINIMUM_FREQUENCY");
    case ERROR_STATIONARY_ENDPOINT:
      return std::string("ERROR_STATIONARY_ENDPOINT");
    case ERROR_MINIMUM_NUM_SETPOINTS:
      return std::string("ERROR_MINIMUM_NUM_SETPOINTS");
    case ERROR_TIME_RUNS_BACKWARDS:
      return std::string("ERROR_TIME_RUNS_BACKWARDS");
    case ERROR_LIMITS_VEL:
      return std::string("ERROR_LIMITS_VEL");
    case ERROR_LIMITS_ACCEL:
      return std::string("ERROR_LIMITS_ACCEL");
    case ERROR_LIMITS_OMEGA:
      return std::string("ERROR_LIMITS_OMEGA");
    case ERROR_LIMITS_ALPHA:
      return std::string("ERROR_LIMITS_ALPHA");
    }
    return std::string("ERROR_NOT_FOUND");
  }

  // Logical comparison of two setpoints to evaluate equality
  bool FlightUtil::Equal(Setpoint const& left, Setpoint const& right,
    size_t degree) {
    Eigen::Quaterniond q, qr, ql;
    Eigen::Vector3d v;
    switch (degree) {
    case 2:
      v = msg_conversions::ros_to_eigen_vector(right.accel.angular)
        - msg_conversions::ros_to_eigen_vector(left.accel.angular);
      if (State::VectorMagnitude(v) > EPSILON)
        return false;
      v = msg_conversions::ros_to_eigen_vector(right.accel.linear)
        - msg_conversions::ros_to_eigen_vector(left.accel.linear);
      if (State::VectorMagnitude(v) > EPSILON)
        return false;
    case 1:
      v = msg_conversions::ros_to_eigen_vector(right.twist.angular)
        - msg_conversions::ros_to_eigen_vector(left.twist.angular);
      if (State::VectorMagnitude(v) > EPSILON)
        return false;
      v = msg_conversions::ros_to_eigen_vector(right.twist.linear)
        - msg_conversions::ros_to_eigen_vector(left.twist.linear);
      if (State::VectorMagnitude(v) > EPSILON)
        return false;
    case 0:
      qr = msg_conversions::ros_to_eigen_quat(right.pose.orientation);
      ql = msg_conversions::ros_to_eigen_quat(left.pose.orientation);
      q = qr * ql.inverse();
      if (State::QuaternionMagnitude(q) > EPSILON)
        return false;
      v = msg_conversions::ros_point_to_eigen_vector(right.pose.position)
        - msg_conversions::ros_point_to_eigen_vector(left.pose.position);
      if (State::VectorMagnitude(v) > EPSILON)
        return false;
    }
    return true;
  }

  // Check that the first pose is consistent with the segment/flight mode
  bool FlightUtil::WithinTolerance(geometry_msgs::Pose const& a,
    geometry_msgs::Pose const& b, double tolerance_pos, double tolerance_att) {
    // Get the desired and actual positions / attitudes
    static Eigen::Quaterniond qd, qa;
    static Eigen::Vector3d vd, va;
    qa = msg_conversions::ros_to_eigen_quat(a.orientation);
    va = msg_conversions::ros_point_to_eigen_vector(a.position);
    qd = msg_conversions::ros_to_eigen_quat(b.orientation);
    vd = msg_conversions::ros_point_to_eigen_vector(b.position);
    // We are not tolerant with respect to our intital attitude
    double att_err = State::QuaternionMagnitude(qd * qa.inverse());
    if (att_err > tolerance_att) {
      ROS_DEBUG_STREAM("Attitude violation: " << att_err);
      return false;
    }
    // We are not tolerant with respect to our initial position
    double pos_err = State::VectorMagnitude(va - vd);
    if (pos_err > tolerance_pos) {
      ROS_DEBUG_STREAM("Position violation: " << pos_err);
      return false;
    }
    // Yes, we are consistent
    return true;
  }

  // Tolerance check using default flight mode values
  bool FlightUtil::WithinTolerance(ff_msgs::FlightMode const& flight_mode,
    geometry_msgs::Pose const& a, geometry_msgs::Pose const& b) {
    return WithinTolerance(a, b,
      flight_mode.tolerance_pos, flight_mode.tolerance_att);
  }
}  // namespace ff_util
