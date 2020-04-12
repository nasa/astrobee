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

// Standard includes
#include <ros/ros.h>

// FSW includes
#include <ff_util/ff_names.h>

// For the trapezoidal planner implementation
#include <planner_trapezoidal/planner_trapezoidal.h>

// C++ includes
#include <vector>
#include <algorithm>
#include <cmath>

/**
 * \ingroup planner
 */
namespace planner_trapezoidal {

  //////////////////////////////////////////////////////////////////////////////
  // Smear a linear or angular displacement (d) over some time (t) taking     //
  // into account a maximum acceleration (a) and velocity (v). The algorithm  //
  // finds resultant time (r) needed  to accelerate to / decelerate from      //
  // cruise time (c) at a constant velocity (h).                              //
  //////////////////////////////////////////////////////////////////////////////
  void FairRamp(double t,     // Time
                double d,     // Distance
                double v,     // Max velocity
                double a,     // Max acceleration
                double &r,    // Ramp time
                double &c,    // Time at constant velocity
                double &h,    // Constant velocity
                double epsilon) {
    if (t < epsilon ||
        d < epsilon)         // This doesn't work for small / negative numbers
      return;
    // If a triangle ramp-up results in correct displacement, then pyramidal
    if (fabs(t * t * a / 4.0 - d) < epsilon) {
      h = sqrt(a * d);
      r = t / 2.0;
      c = 0.0;
      // In the case of a trapezoidal ramp, things are more complex to calculate
    } else {
      h = (a * t - sqrt(a * a * t * t - 4.0 * d * a)) / 2.0;
      r = h / a;
      c = (d - r * h) / h;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Given some linear or angular displacement (d) and a maximum acceleration //
  // (a) and velocity (v), return the time taken (t) to achieve the motion as //
  // quickly as possible. Also return the time (r) needed to accelerate to or //
  // decelerate from the cruise phase, which lasts a given time (c) at a      //
  // constant velocity (h).                                                   //
  //////////////////////////////////////////////////////////////////////////////
  double GreedyRamp(double d,     // Distance
                    double v,     // Max velocity
                    double a,     // Max acceleration
                    double &r,    // Ramp time
                    double &c,    // Time at constant velocity
                    double &h,    // Constant velocity
                    double epsilon) {
    if (d < epsilon)             // Doesn't work for small / negative numbers
      return 0.0;
    h = sqrt(a * d);              // The max vel required if one had zero dwell
    if (h > v) {                  // If the required velocity is too high
      h = v;                      // Clamp the velocity to maximum
      r = h / a;                  // Time taken to ramp up and down to max vel
      c = (d - h * h / a) / h;    // Dwell time at maxmimum velocity
    } else {                      // If we don't need to achieve max velocity
      r = d / h;                  // Time taken to ramp up/down to right vel
      c = 0.0;                    // Time at constant velocity
    }
    return (r * 2.0 + c);         // Minimum time required to complete action
  }

  // Insert a trapezoid between two poses
  void InsertTrapezoid(ff_util::Segment &segment, ros::Time & offset, double dt,
                       const Eigen::Affine3d & p0, const Eigen::Affine3d & p1,
                       double lin_v, double rot_v, double lin_a, double rot_a,
                       double min_control_period, double epsilon) {
    // Find the delta transform between the two poses
    Eigen::AngleAxisd error_ang(p0.rotation().inverse()* p1.rotation());
    Eigen::Vector3d error_vec(p1.translation() - p0.translation());
    // Make sure that we are taking the minimum angle of rotatin
    error_ang.angle() = fmod(error_ang.angle(), 2.0 * M_PI);
    if (error_ang.angle() > M_PI)
      error_ang.angle() -= 2.0 * M_PI;
    if (error_ang.angle() < -M_PI)
      error_ang.angle() += 2.0 * M_PI;
    if (error_ang.angle() < 0) {
      error_ang.angle() *= -1.0;
      error_ang.axis() *= -1.0;
    }
    // Get the linear and rotational directions
    Eigen::Vector3d lin_d = error_vec.normalized();
    Eigen::Vector3d rot_d = error_ang.axis().normalized();
    // Get the linear and rotational magnitudes
    double lin_m = error_vec.norm();
    double rot_m = error_ang.angle();
    // Special case: the beginning and end poses are the same
    if (lin_m < epsilon && rot_m < epsilon)
      return;
    // Greedy phase - calculate the linear and rotational ramps separately
    double lin_t = 0.0, lin_r = 0.0, lin_c = 0.0, lin_h = 0.0;
    lin_t = GreedyRamp(lin_m, lin_v, lin_a, lin_r, lin_c, lin_h, epsilon);
    double rot_t = 0.0, rot_r = 0.0, rot_c = 0.0, rot_h = 0.0;
    rot_t = GreedyRamp(rot_m, rot_v, rot_a, rot_r, rot_c, rot_h, epsilon);
    // Now determine whether the angular or linear component dominates...
    double tmin = (lin_t > rot_t ? lin_t : rot_t);
    if (dt > 0 && tmin < dt)
      tmin = dt;
    // Now recalculate the ramps using the dominating time
    FairRamp(tmin, lin_m, lin_v, lin_a, lin_r, lin_c, lin_h, epsilon);
    FairRamp(tmin, rot_m, rot_v, rot_a, rot_r, rot_c, rot_h, epsilon);
    // Create a vector of timestamps
    std::vector < double > ts;
    if (lin_m > epsilon) {
      ts.push_back(lin_r);              // End of ramp-up
      if (lin_c > epsilon)
        ts.push_back(lin_r + lin_c);    // End of cruise-phase
    }
    if (rot_m > epsilon) {
      ts.push_back(rot_r);              // End of ramp-up
      if (rot_c > epsilon)
        ts.push_back(rot_r + rot_c);    // End of cruise-phase
    }
    for (double t = 0.0; t < tmin; t += min_control_period / 2.0)
      ts.push_back(t);
    ts.push_back(tmin);
    // Sort and remove any duplicate timestamps within min_control_period
    std::sort(ts.begin(), ts.end());
    std::vector < double > :: iterator it;
    for (it = ts.begin(); it != ts.end(); it++) {
      // Get the time and delta t
      ff_util::State state;
      state.t = *it;
      state.p = p0.translation();
      state.v = Eigen::Vector3d(0.0, 0.0, 0.0);
      state.a = Eigen::Vector3d(0.0, 0.0, 0.0);
      state.q = p0.rotation();
      state.w = Eigen::Vector3d(0.0, 0.0, 0.0);
      state.b = Eigen::Vector3d(0.0, 0.0, 0.0);
      // Deal with the linear component
      if (lin_m > epsilon) {
        // End of ramp
        if (state.t >= tmin) {
          state.a = Eigen::Vector3d(0.0, 0.0, 0.0);
          state.v = Eigen::Vector3d(0.0, 0.0, 0.0);
          state.p = p0.translation()
                  + lin_d * (lin_a * lin_r * lin_r + lin_h * lin_c);
        // Ramp-down
        } else if (state.t >= lin_r + lin_c) {
          double t = state.t - lin_r - lin_c;
          state.a = -lin_a * lin_d;
          state.v = lin_h * lin_d - lin_a * lin_d * t;
          state.p = p0.translation()
            + lin_d * (0.5 * lin_a * lin_r * lin_r            // Ramp up
            + lin_h * lin_c                                   // Cruise phase
            + lin_h * t - 0.5 * lin_a * t * t);               // Ramp down
        // Cruise-phase
        } else if (state.t >= lin_r && lin_c > epsilon) {
          double t = state.t - lin_r;
          state.a = Eigen::Vector3d(0.0, 0.0, 0.0);
          state.v = lin_h * lin_d;
          state.p = p0.translation()
                  + lin_d * (0.5 * lin_a * lin_r * lin_r      // Ramp up
                  + lin_h * t);                               // Cruise phase
        // Ramp-up
        } else if (state.t >= 0.0) {
          double t = state.t;
          state.a = lin_a * lin_d;
          state.v = lin_a * lin_d * t;
          state.p = p0.translation()
                  + lin_d * (0.5 * lin_a * t * t);            // Ramp up
        }
      }
      // Deal with the angular component
      if (rot_m > epsilon) {
        // End of ramp
        if (state.t >= tmin) {
          state.b = Eigen::Vector3d(0.0, 0.0, 0.0);
          state.w = Eigen::Vector3d(0.0, 0.0, 0.0);
          state.q = p0.rotation() * Eigen::Quaterniond(
            Eigen::AngleAxisd(rot_a * rot_r * rot_r + rot_h * rot_c, rot_d));
        // Ramp-down
        } else if (state.t >= rot_r + rot_c) {
          double t = state.t - rot_r - rot_c;
          state.b = -rot_a * rot_d;
          state.w = rot_h * rot_d - rot_a * rot_d * t;
          state.q = p0.rotation() * Eigen::Quaterniond(
            Eigen::AngleAxisd(0.5 * rot_a * rot_r * rot_r     // Ramp up
              + rot_h * rot_c                                 // Cruise phase
              + rot_h * t - 0.5 * rot_a * t * t, rot_d));     // Ramp down
        // Cruise phase
        } else if (state.t >= rot_r && rot_c > epsilon) {
          double t = state.t - rot_r;
          state.b = Eigen::Vector3d(0.0, 0.0, 0.0);
          state.w = rot_h * rot_d;
          state.q = p0.rotation() * Eigen::Quaterniond(
            Eigen::AngleAxisd(0.5 * rot_a * rot_r * rot_r     // Ramp up
                + rot_h * t, rot_d));                         // Cruise phase
        // Ramp-up
        } else if (state.t >= 0.0) {
          double t = state.t;
          state.b = rot_a * rot_d;
          state.w = rot_a * rot_d * t;
          state.q = p0.rotation() * Eigen::Quaterniond(
            Eigen::AngleAxisd(0.5 * rot_a * t * t, rot_d));   // Ramp up
        }
      }
      // Add the setpoint to the segment
      ff_util::Setpoint sp;
      sp.when = offset + ros::Duration(state.t);
      sp.pose.position = msg_conversions::eigen_to_ros_point(state.p);
      sp.pose.orientation = msg_conversions::eigen_to_ros_quat(state.q);
      sp.twist.linear = msg_conversions::eigen_to_ros_vector(state.v);
      sp.twist.angular = msg_conversions::eigen_to_ros_vector(state.w);
      sp.accel.linear = msg_conversions::eigen_to_ros_vector(state.a);
      sp.accel.angular = msg_conversions::eigen_to_ros_vector(state.b);
      segment.push_back(sp);
    }
    // Increment the offset by the total time
    offset += ros::Duration(tmin);
  }


}  // namespace planner_trapezoidal
