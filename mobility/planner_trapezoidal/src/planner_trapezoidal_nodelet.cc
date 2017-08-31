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
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// FSW includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_names.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>
#include <msg_conversions/msg_conversions.h>

// For the planner implementation API
#include <choreographer/planner.h>

// C++ includes
#include <vector>
#include <algorithm>
#include <cmath>

/**
 * \ingroup planner
 */
namespace planner_trapezoidal {

using RESPONSE = ff_msgs::PlanResult;

// Structure to store the trapezoid
struct trapezoid {
  double v;   // Velocity limit
  double a;   // Acceleration limit
  double h;   // Constant velocity
  double r;   // Ramp time
  double c;   // Constant velocity time
};

class PlannerTrapezoidalNodelet : public planner::PlannerImplementation {
 public:
  PlannerTrapezoidalNodelet() : planner::PlannerImplementation("trapezoidal", "Trapezoidal path planner") {}
  ~PlannerTrapezoidalNodelet() {}

 protected:
  bool InitializePlanner(ros::NodeHandle *nh) {
    // Grab some configuration parameters for this node from the LUA config reader
    cfg_.Initialize(GetPrivateHandle(), "mobility/planner_trapezoidal.config");
    cfg_.Listen(boost::bind(&PlannerTrapezoidalNodelet::ReconfigureCallback, this, _1));
    // Setup a timer to forward diagnostics
    timer_d_ = nh->createTimer(ros::Duration(ros::Rate(DEFAULT_DIAGNOSTICS_RATE)),
      &PlannerTrapezoidalNodelet::DiagnosticsCallback, this, false, true);
    // Save the epsilon value
    epsilon_ = cfg_.Get<double>("epsilon");
      // Notify initialization complete
    NODELET_DEBUG_STREAM("Initialization complete");
    // Success
    return true;
  }

  void DiagnosticsCallback(const ros::TimerEvent &event) {
    SendDiagnostics(cfg_.Dump());
  }

  bool ReconfigureCallback(dynamic_reconfigure::Config &config) {
    if (!cfg_.Reconfigure(config))
      return false;
    epsilon_ = cfg_.Get<double>("epsilon");
    return true;
  }

  void PlanCallback(ff_msgs::PlanGoal const& goal) {
    // Do some basic error checks
    ff_msgs::PlanResult plan_result;
    plan_result.response = RESPONSE::SUCCESS;
    if (goal.states.size() < 2) plan_result.response = RESPONSE::NOT_ENOUGH_STATES;
    if (goal.check_obstacles)   plan_result.response = RESPONSE::OBSTACLES_NOT_SUPPORTED;
    if (plan_result.response < 0)
      return PlanResult(plan_result);
    // Save the information
    desired_vel_ = goal.desired_vel;
    desired_omega_ = goal.desired_omega;
    desired_accel_ = goal.desired_accel;
    desired_alpha_ = goal.desired_alpha;
    min_control_period_ = 1.0 / goal.desired_rate;
    // Setup header and keep track of time as we generate trapezoidal ramps
    plan_result.segment.clear();
    // Generate the trapezoidal ramp
    ros::Time offset = goal.states.front().header.stamp;
    for (size_t i = 1; i < goal.states.size(); i++) {
      // Get the requested duration and offset
      double dt = (goal.states[i].header.stamp - goal.states[i-1].header.stamp).toSec();
      // Extract affine transformations for the two poses
      Eigen::Affine3d tf_1 = msg_conversions::ros_pose_to_eigen_transform(goal.states[i-1].pose);
      Eigen::Affine3d tf_4 = msg_conversions::ros_pose_to_eigen_transform(goal.states[i].pose);
      // Advanced case: generate a forward-only segment
      if (goal.faceforward) {
        // Get the current vector representing the forward direction
        Eigen::Affine3d delta = tf_4 * tf_1.inverse();
        Eigen::Vector3d vfwd = delta.translation().normalized();
        Eigen::Vector3d vdown(0.0, 0.0, 1.0);
        Eigen::Vector3d vright = vdown.cross(vfwd);
        vdown = vfwd.cross(vright);
        if (vdown.z() < 0)
          vdown = vfwd.cross(-vright);
        Eigen::Matrix3d dcm;
        dcm << vfwd.x(), vright.x(), vdown.x(),
               vfwd.y(), vright.y(), vdown.y(),
               vfwd.z(), vright.z(), vdown.z();
        Eigen::Quaterniond q(dcm);
        // Intermediary transforms
        Eigen::Affine3d tf_2;
        tf_2.translation() = tf_1.translation();
        tf_2.linear() = q.toRotationMatrix();
        Eigen::Affine3d tf_3;
        tf_3.translation() = tf_4.translation();
        tf_3.linear() = q.toRotationMatrix();
        // Initial rotation into direction orientation
        InsertTrapezoid(plan_result.segment, offset, dt, tf_1, tf_2);
        InsertTrapezoid(plan_result.segment, offset, dt, tf_2, tf_3);
        InsertTrapezoid(plan_result.segment, offset, dt, tf_3, tf_4);
      } else {
        // Fully-holonomic smear of delta across all axes
        InsertTrapezoid(plan_result.segment, offset, dt, tf_1, tf_4);
      }
    }
    // Make sure to set the flight mode!
    plan_result.flight_mode = goal.flight_mode;
    // Special case: we might already be there
    if (plan_result.segment.size() < 2)
      plan_result.response = RESPONSE::ALREADY_THERE;
    // Callback with the result
    return PlanResult(plan_result);
  }

  // Called to interrupt the process
  virtual void CancelCallback() {}

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Greedy ramp calculation - given some linear or angular displacement (d) and a maximum acceleration (a) and       //
  //                           velocity (v), return the time taken (t) to achieve the motion as quickly as possible.  //
  //                           Also return the time (r) needed to accelerate to / decelerate from the cruise          //
  //                           phase, which lasts a given time (c) at a constant velocity (h).                        //
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  double GreedyRamp(double d,             // Distance
                    double v,             // Max velocity
                    double a,             // Max acceleration
                    double &r,            // Ramp time
                    double &c,            // Time at constant velocity
                    double &h) {          // Constant velocity
    if (d < epsilon_)                     // This doesn't work for small / negative numbers
      return 0.0;
    h = sqrt(a * d);                      // The max velocity required if one had zero dwell time
    if (h > v) {                          // If the required velocity is too high
      h = v;                              // Clamp the velocity to maximum
      r = h / a;                          // Time taken to ramp up and down to max velocity
      c = (d - h * h / a) / h;            // Dwell time at maxmimum velocity
    } else {                              // If we don't need to achieve max velocity
      r = d / h;                          // Time taken to ramp up and down to sufficient velocity
      c = 0.0;                            // Time at constant velocity
    }
    return (r * 2.0 + c);                 // Minimum time required to complete the action
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Fair ramp calculation -- smear a linear or angular displacement (d) over some time (t) taking into account       //
  //                          a maximum acceleration (a) and velocity (v). Algorithm finds resultant time (r) needed  //
  //                          to accelerate to / decelerate from cruise time (c) at a constant velocity (h).          //
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void FairRamp(double t,                 // Time
                double d,                 // Distance
                double v,                 // Max velocity
                double a,                 // Max acceleration
                double &r,                // Ramp time
                double &c,                // Time at constant velocity
                double &h) {              // Constant velocity
    if (t < epsilon_ || d < epsilon_)     // This doesn't work for small / negative numbers
      return;
    // If a triangle ramp-up results in the correct displacement, then its a triangle
    if (fabs(t * t * a / 4.0 - d) < epsilon_) {
      h = sqrt(a * d);
      r = t / 2.0;
      c = 0.0;
    // In the case of a trapezoidal ramp up, things are a bit more complex to calculate
    } else {
      h = (a * t - sqrt(a * a * t * t - 4.0 * d * a)) / 2.0;
      r = h / a;
      c = (d - r * h) / h;
    }
  }

  // Insert a trapezoid between two poses
  void InsertTrapezoid(ff_util::Segment &segment, ros::Time & offset, double dt,
    const Eigen::Affine3d & p0, const Eigen::Affine3d & p1) {
    // Default speeds are set by the planner configuration
    double lin_v = desired_vel_;
    double rot_v = desired_omega_;
    double lin_a = desired_accel_;
    double rot_a = desired_alpha_;
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
    if (lin_m < epsilon_ && rot_m < epsilon_)
      return;
    // Greedy phase - calculate the linear and rotational ramps separately
    double lin_t = 0.0, lin_r = 0.0, lin_c = 0.0, lin_h = 0.0;
    lin_t = GreedyRamp(lin_m, lin_v, lin_a, lin_r, lin_c, lin_h);
    double rot_t = 0.0, rot_r = 0.0, rot_c = 0.0, rot_h = 0.0;
    rot_t = GreedyRamp(rot_m, rot_v, rot_a, rot_r, rot_c, rot_h);
    // Now determine whether the angular or linear component dominates...
    double tmin = (lin_t > rot_t ? lin_t : rot_t);
    if (dt > 0 && tmin < dt)
      tmin = dt;
    // Now recalculate the ramps using the dominating time
    FairRamp(tmin, lin_m, lin_v, lin_a, lin_r, lin_c, lin_h);
    FairRamp(tmin, rot_m, rot_v, rot_a, rot_r, rot_c, rot_h);
    // Create a vector of timestamps
    std::vector < double > ts;
    if (lin_m > epsilon_) {
      ts.push_back(lin_r);              // End of ramp-up
      if (lin_c > epsilon_)
        ts.push_back(lin_r + lin_c);    // End of cruise-phase
    }
    if (rot_m > epsilon_) {
      ts.push_back(rot_r);              // End of ramp-up
      if (rot_c > epsilon_)
        ts.push_back(rot_r + rot_c);    // End of cruise-phase
    }
    for (double t = 0.0; t < tmin; t += min_control_period_ / 2.0)
      ts.push_back(t);
    ts.push_back(tmin);
    // Sort and remove any duplicate timestamps within pc.min_control_period
    std::sort(ts.begin(), ts.end());
    for (std::vector < double > :: iterator it = ts.begin(); it != ts.end(); it++) {
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
      if (lin_m > epsilon_) {
        // End of ramp
        if (state.t >= tmin) {
          state.a = Eigen::Vector3d(0.0, 0.0, 0.0);
          state.v = Eigen::Vector3d(0.0, 0.0, 0.0);
          state.p = p0.translation() + lin_d * (lin_a * lin_r * lin_r + lin_h * lin_c);
        // Ramp-down
        } else if (state.t >= lin_r + lin_c) {
          double t = state.t - lin_r - lin_c;
          state.a = -lin_a * lin_d;
          state.v = lin_h * lin_d - lin_a * lin_d * t;
          state.p = p0.translation() + lin_d * (0.5 * lin_a * lin_r * lin_r   // Ramp up
                  + lin_h * lin_c                                             // Cruise
                  + lin_h * t - 0.5 * lin_a * t * t);                         // Ramp down
        // Cruise-phase
        } else if (state.t >= lin_r && lin_c > epsilon_) {
          double t = state.t - lin_r;
          state.a = Eigen::Vector3d(0.0, 0.0, 0.0);
          state.v = lin_h * lin_d;
          state.p = p0.translation() + lin_d * (0.5 * lin_a * lin_r * lin_r   // Ramp up
                  + lin_h * t);                                               // Cruise
        // Ramp-up
        } else if (state.t >= 0.0) {
          double t = state.t;
          state.a = lin_a * lin_d;
          state.v = lin_a * lin_d * t;
          state.p = p0.translation() + lin_d * (0.5 * lin_a * t * t);         // Ramp up
        }
      }
      // Deal with the angular component
      if (rot_m > epsilon_) {
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
            Eigen::AngleAxisd(0.5 * rot_a * rot_r * rot_r                   // Ramp up
                              + rot_h * rot_c                               // Cruise phase
                              + rot_h * t - 0.5 * rot_a * t * t, rot_d));   // Ramp down
        // Cruise phase
        } else if (state.t >= rot_r && rot_c > epsilon_) {
          double t = state.t - rot_r;
          state.b = Eigen::Vector3d(0.0, 0.0, 0.0);
          state.w = rot_h * rot_d;
          state.q = p0.rotation() * Eigen::Quaterniond(
            Eigen::AngleAxisd(0.5 * rot_a * rot_r * rot_r                   // Ramp up
                              + rot_h * t, rot_d));                         // Cruise phase
        // Ramp-up
        } else if (state.t >= 0.0) {
          double t = state.t;
          state.b = rot_a * rot_d;
          state.w = rot_a * rot_d * t;
          state.q = p0.rotation() * Eigen::Quaterniond(
            Eigen::AngleAxisd(0.5 * rot_a * t * t, rot_d));                 // Ramp up
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

 protected:
  ff_util::ConfigServer cfg_;
  ros::Timer timer_d_;
  double desired_vel_;
  double desired_omega_;
  double desired_accel_;
  double desired_alpha_;
  double min_control_period_;
  double epsilon_;
};

PLUGINLIB_DECLARE_CLASS(planner_trapezoidal, PlannerTrapezoidalNodelet,
                        planner_trapezoidal::PlannerTrapezoidalNodelet, nodelet::Nodelet);

}  // namespace planner_trapezoidal
