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

#include <ff_msgs/MobilityResult.h>

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

  // Generate a strapdown equation for a given quaternion
  Eigen::Matrix4d State::Omega(Eigen::Vector3d const& vec) {
    Eigen::Matrix4d mat;
    mat(0, 0) =  0.0;     mat(0, 1) =  vec(2);  mat(0, 2) = -vec(1);  mat(0, 3) =  vec(0);
    mat(1, 0) = -vec(2);  mat(1, 1) =  0.0;     mat(1, 2) =  vec(0);  mat(1, 3) =  vec(1);
    mat(2, 0) =  vec(1);  mat(2, 1) = -vec(0);  mat(2, 2) =  0.0;     mat(2, 3) =  vec(2);
    mat(3, 0) = -vec(0);  mat(3, 1) = -vec(1);  mat(3, 2) = -vec(2);  mat(3, 3) =  0.0;
    return mat;
  }

  // Propagate the state forward by a bit
  State State::Propagate(double dt) {
    State state;
    // Quaternion integration must be done numerically, as there is no closed form
    // solution for alpha > 0. See Jesse and Robert's notes.
    if (dt > 0) {
      Eigen::Vector4d qvec;
      qvec(0) = q.x();
      qvec(1) = q.y();
      qvec(2) = q.z();
      qvec(3) = q.w();
      Eigen::Matrix4d ow = Omega(w);
      Eigen::Matrix4d ob = Omega(b);
      Eigen::Matrix4d ok = ow + 0.5 * ob * dt;
      qvec = ((0.5 * ok * dt).exp() + 1.0 / 48.0 * (ob * ow - ow * ob) * dt * dt *dt) * qvec;
      state.q.x() = qvec(0);
      state.q.y() = qvec(1);
      state.q.z() = qvec(2);
      state.q.w() = qvec(3);
      state.p = p + (v + 0.5 * a * dt) * dt;
      state.w = w + b * dt;
      state.v = v + a * dt;
      state.b = b;
      state.a = a;
      state.t = t + dt;
    } else {
      state.q = q;
      state.p = p;
      state.w = w;
      state.v = v;
      state.b = b;
      state.a = a;
      state.t = t;
    }
    return state;
  }

  // Convert the state to a setpoint
  Setpoint State::ToSetpoint() {
    Setpoint sp;
    sp.when              = ros::Time(t);
    sp.pose.orientation  = msg_conversions::eigen_to_ros_quat(q);
    sp.pose.position     = msg_conversions::eigen_to_ros_point(p);
    sp.twist.angular     = msg_conversions::eigen_to_ros_vector(w);
    sp.twist.linear      = msg_conversions::eigen_to_ros_vector(v);
    sp.accel.angular     = msg_conversions::eigen_to_ros_vector(b);
    sp.accel.linear      = msg_conversions::eigen_to_ros_vector(a);
    return sp;
  }

  // Return the magnitude of the rotation
  double State::QuaternionMagnitude() {
    return QuaternionMagnitude(q);
  }

  // Return the magnitude of the rotation
  double State::QuaternionMagnitude(Eigen::Quaterniond const& iq) {
    return 2.0 * atan2(iq.vec().norm(), iq.w());
  }

  // Return the magnitude of the rotation
  double State::VectorMagnitude(Eigen::Vector3d const& iv) {
    return iv.norm();
  }

  /////////////////////////////////////////////////////////////////////
  // FLIGHT UTILITIES SHARED BETWEEN THE PLANNING AND GNC SUBSYSTEMS //
  /////////////////////////////////////////////////////////////////////

  bool ConfigUtil::GetGeneralConfig(GeneralConfig & gc_out) {
    static GeneralConfig gc;
    if (gc.default_flight_mode.empty()) {
      config_reader::ConfigReader cfg;
      cfg.AddFile("flight.config");
      cfg.ReadFiles();
      if (   !cfg.GetStr("default_flight_mode", &gc.default_flight_mode)
          || !cfg.GetReal("time_sync_threshold", &gc.time_sync_threshold)
          || !cfg.GetReal("min_control_rate", &gc.min_control_rate))
        return false;
    }
    gc_out = gc;
    return true;
  }

  // Get all inertia configuration (LUA config file is queried only once)
  bool ConfigUtil::GetInertiaConfig(InertiaConfig & ic_out) {
    static InertiaConfig ic;
    if (ic.mass < 0) {
      config_reader::ConfigReader cfg;
      config_reader::ConfigReader::Table tmp;
      float inertia_matrix[9];
      cfg.AddFile("flight.config");
      cfg.ReadFiles();
      if (   !cfg.GetReal("inertia_mass", &ic.mass)
          || !msg_conversions::config_read_matrix(&cfg, "inertia_matrix", 3, 3, inertia_matrix)
          || !cfg.GetTable("inertia_com", &tmp)
          || !msg_conversions::config_read_vector(&tmp, &ic.center_of_mass))
        return false;
      ic.inertia_matrix(0, 0) = inertia_matrix[0];
      ic.inertia_matrix(0, 1) = inertia_matrix[1];
      ic.inertia_matrix(0, 2) = inertia_matrix[2];
      ic.inertia_matrix(1, 0) = inertia_matrix[3];
      ic.inertia_matrix(1, 1) = inertia_matrix[4];
      ic.inertia_matrix(1, 2) = inertia_matrix[5];
      ic.inertia_matrix(2, 0) = inertia_matrix[6];
      ic.inertia_matrix(2, 1) = inertia_matrix[7];
      ic.inertia_matrix(2, 2) = inertia_matrix[8];
    }
    ic_out = ic;
    return true;
  }

  // Get data for a given flight mode
  bool ConfigUtil::GetFlightMode(std::string const& name, FlightMode &data) {
    // make sure we have the flight modes
    static FlightModeMap fm;
    if (fm.empty()) {
      config_reader::ConfigReader cfg;
      cfg.AddFile("flight.config");
      cfg.ReadFiles();
      config_reader::ConfigReader::Table flight_modes, mode, tmp;
      if (!cfg.GetTable("flight_modes", &flight_modes))
        return false;
      for (int j = 0; j < flight_modes.GetSize(); j++) {
        FlightMode info;
        if (   !flight_modes.GetTable(j + 1, &mode)
            || !mode.GetStr("name", &info.name)
            || !mode.GetReal("hard_limit_omega", &info.hard_limit_omega)
            || !mode.GetReal("hard_limit_vel", &info.hard_limit_vel)
            || !mode.GetReal("hard_limit_alpha", &info.hard_limit_alpha)
            || !mode.GetReal("hard_limit_accel", &info.hard_limit_accel)
            || !mode.GetReal("tolerance_pos", &info.tolerance_pos)
            || !mode.GetReal("tolerance_att", &info.tolerance_att)
            || !mode.GetReal("tolerance_omega", &info.tolerance_omega)
            || !mode.GetReal("tolerance_vel", &info.tolerance_vel)
            || !mode.GetReal("tolerance_time", &info.tolerance_time)
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
        fm[info.name] = info;
      }
    }
    // Check that this flight mode exists
    if (fm.find(name) == fm.end())
      return false;
    // Extract the flight mode info
    data = fm[name];
    return true;
  }

  // Validate if an upper limit has been breached
  double SegmentUtil::ValidateUpperLimit(const double limit, const double val) {
    // Special case: negative numbers imply saturation
    if (val < 0)
      return limit;
    // Are we less than limit + epsilon?
    return (val - limit < EPSILON ? val : INVALID_VALUE);
  }

  // Validate if a lower limit has been breached
  double SegmentUtil::ValidateLowerLimit(const double limit, const double val) {
    // Special case: negative numbers imply saturation
    if (val < 0)
      return limit;
    // Are we less than limit + epsilon?
    return (limit - val < EPSILON ? val : INVALID_VALUE);
  }

  // The code to o
  SegmentResult SegmentUtil::Check(SegmentCheckMask mask, Segment const& segment, std::string const& fm) {
    // ROS_WARN_STREAM("HERE " << fm);
    /////////////////////////////////////////////////////////////////////////////
    // Check 1 : That we have loaded the flight mode table and the mode exists //
    /////////////////////////////////////////////////////////////////////////////
    FlightMode flight_mode;
    if (!ConfigUtil::GetFlightMode(fm, flight_mode))
      return ERROR_INVALID_FLIGHT_MODE;
    GeneralConfig general_config;
    if (!ConfigUtil::GetGeneralConfig(general_config))
      return ERROR_INVALID_GENERAL_CONFIG;
    ///////////////////////////////////////////////////////////////////////////
    // Check 2 : Always check that we have the minimum number of set points  //
    //           If we didn't check this, other tests would fail             //
    ///////////////////////////////////////////////////////////////////////////
    if (segment.size() < 2)
      return ERROR_MINIMUM_NUM_SETPOINTS;
    // Iterate over all segments
    for (Segment::const_iterator it = segment.begin(); it != segment.end(); it++) {
      State c_s(*it);
      ///////////////////////////////////////////////////////////////////////////
      // Check 3 : Check that the final setpoint in the segment has a zero     //
      //           velocity. If it doesn't, then the controller will drift!    //
      ///////////////////////////////////////////////////////////////////////////
      if (it + 1 == segment.end()) {
        if (mask & CHECK_STATIONARY_ENDPOINT) {
          if (State::VectorMagnitude(c_s.w) == 0 || State::VectorMagnitude(c_s.v) == 0
           || State::VectorMagnitude(c_s.a) == 0 || State::VectorMagnitude(c_s.b) == 0)
            return ERROR_STATIONARY_ENDPOINT;
        }
        break;
      }
      // ROS_WARN_STREAM("PASSED4");
      ///////////////////////////////////////////////////////////////////////////
      // Check 5 : Check limits on acceleration and velocities                 //
      ///////////////////////////////////////////////////////////////////////////
      if ((mask & CHECK_LIMITS_OMEGA)
        && ValidateUpperLimit(flight_mode.hard_limit_omega, State::VectorMagnitude(c_s.w)) < 0)
        return ERROR_LIMITS_OMEGA;
      if ((mask & CHECK_LIMITS_VEL)
        && ValidateUpperLimit(flight_mode.hard_limit_vel, State::VectorMagnitude(c_s.v)) < 0)
        return ERROR_LIMITS_VEL;
      if ((mask & CHECK_LIMITS_ALPHA)
        && ValidateUpperLimit(flight_mode.hard_limit_alpha, State::VectorMagnitude(c_s.b)) < 0)
        return ERROR_LIMITS_ALPHA;
      if ((mask & CHECK_LIMITS_ACCEL)
        && ValidateUpperLimit(flight_mode.hard_limit_accel, State::VectorMagnitude(c_s.a)) < 0)
        return ERROR_LIMITS_ACCEL;
      ///////////////////////////////////////////////////////////////////////////
      // Check 5 : Check that we have a delta t in the range [0, timeout]      //
      ///////////////////////////////////////////////////////////////////////////
      State n_s(*(it + 1));
      State diff = n_s - c_s;
      if (diff.t < -EPSILON)
        return ERROR_TIME_RUNS_BACKWARDS;
      if (diff.t - general_config.min_control_rate > EPSILON)
        return ERROR_MINIMUM_FREQUENCY;
      if (diff.t < EPSILON)
        continue;
    }
    // This segment is valid
    return SUCCESS;
  }

  // Resample a segment to a minimum control rate
  SegmentResult SegmentUtil::Resample(Segment const& in, Segment & out, double rate) {
    // Check that we have enough setpoints
    if (in.size() < 2)
      return ERROR_MINIMUM_NUM_SETPOINTS;
    // If the rate was not specified, try and get it from the general config
    if (rate <= 0.0) {
      GeneralConfig general_config;
      if (!ConfigUtil::GetGeneralConfig(general_config))
        return ERROR_INVALID_GENERAL_CONFIG;
      rate = general_config.min_control_rate;
    }
    // Peform the resample of the segment
    for (Segment::const_iterator it = in.begin(); it != in.end(); it++) {
      // Convert the current setpoint to a state
      State state(*it);
      Segment::const_iterator jt = std::next(it);
      if (jt == in.end()) {
        out.push_back(state.ToSetpoint());
      } else {
        // Propagate the state forward adding setpoints at a given rate, until
        // we reach the next setpoint. Then stop, and move onto the next setpoint.
        do {
          out.push_back(state.ToSetpoint());
          state = state.Propagate(rate);
        } while (state.t < jt->when.toSec());
      }
    }
    // Success
    return SUCCESS;
  }

  // Print a human-readable description of the error code
  std::string SegmentUtil::GetDescription(SegmentResult result) {
    switch (result) {
    case SUCCESS:                      return std::string("SUCCESS");
    case ERROR_MINIMUM_FREQUENCY:      return std::string("ERROR_MINIMUM_FREQUENCY");
    case ERROR_STATIONARY_ENDPOINT:    return std::string("ERROR_STATIONARY_ENDPOINT");
    case ERROR_MINIMUM_NUM_SETPOINTS:  return std::string("ERROR_MINIMUM_NUM_SETPOINTS");
    case ERROR_TIME_RUNS_BACKWARDS:    return std::string("ERROR_TIME_RUNS_BACKWARDS");
    case ERROR_LIMITS_VEL:             return std::string("ERROR_LIMITS_VEL");
    case ERROR_LIMITS_ACCEL:           return std::string("ERROR_LIMITS_ACCEL");
    case ERROR_LIMITS_OMEGA:           return std::string("ERROR_LIMITS_OMEGA");
    case ERROR_LIMITS_ALPHA:           return std::string("ERROR_LIMITS_ALPHA");
    case ERROR_INVALID_FLIGHT_MODE:    return std::string("ERROR_INVALID_FLIGHT_MODE");
    case ERROR_INVALID_GENERAL_CONFIG: return std::string("ERROR_INVALID_GENERAL_CONFIG");
    }
    return std::string("ERROR_NOT_FOUND");
  }

  // Logical comparison of two setpoints to evaluate equality within defined EPSILON value
  bool SegmentUtil::Equal(Setpoint const& left, Setpoint const& right, size_t degree) {
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

  // Load a flight mode and segment from a file
  bool SegmentUtil::Load(std::string const& fname, std::string & flight_mode, Segment & segment) {
    std::ifstream ifs(fname, std::ios::in | std::ios::binary);
    if (!ifs.is_open())
      return false;
    ff_msgs::MobilityResult msg;
    // Read the data fromt eh file
    ifs.seekg(0, std::ios::end);
    std::streampos end = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    std::streampos begin = ifs.tellg();
    uint32_t file_size = end - begin;
    boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
    ifs.read(reinterpret_cast < char* > (ibuffer.get()), file_size);
    ros::serialization::IStream istream(ibuffer.get(), file_size);
    ros::serialization::deserialize(istream, msg);
    ifs.close();
    // Copy the data
    flight_mode = msg.flight_mode;
    segment = msg.segment;
    // Succes
    return true;
  }
  // Save a flight mode and segment from to file
  bool SegmentUtil::Save(std::string const& fname, std::string const& flight_mode, Segment const& segment) {
    std::ofstream ofs(fname, std::ios::out | std::ios::binary);
    if (!ofs.is_open())
      return false;
    // Assemble the data
    ff_msgs::MobilityResult msg;
    msg.flight_mode = flight_mode;
    msg.segment = segment;
    // Read the data fromt eh file
    uint32_t serial_size = ros::serialization::serializationLength(msg);
    boost::shared_array < uint8_t > obuffer(new uint8_t[serial_size]);
    ros::serialization::OStream ostream(obuffer.get(), serial_size);
    ros::serialization::serialize(ostream, msg);
    ofs.write(reinterpret_cast < char* > (obuffer.get()), serial_size);
    ofs.close();
    // Success!
    return true;
  }

}  // namespace ff_util
