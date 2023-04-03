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
#include <ff_common/ff_ros.h>
#include <tf2_ros/buffer.h>

// FSW includes
#include <ff_util/ff_component.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_timer.h>
#include <ff_common/ff_names.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>
#include <msg_conversions/msg_conversions.h>

// For the planner implementation API
#include <choreographer/planner.h>

// Actions
#include <ff_msgs/action/motion.hpp>
#include <ff_msgs/action/control.hpp>
namespace ff_msgs {
typedef action::Motion Motion;
typedef action::Control Control;
}  // namespace ff_msgs

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

FF_DEFINE_LOGGER("planner_trapezoidal");

using RESPONSE = ff_msgs::Plan::Result;

class PlannerTrapezoidalComponent : public planner::PlannerImplementation {
 public:
  explicit PlannerTrapezoidalComponent(const rclcpp::NodeOptions & options) :
    planner::PlannerImplementation(options, "trapezoidal", "Trapezoidal path planner") {}
  ~PlannerTrapezoidalComponent() {}

 protected:
  bool InitializePlanner(NodeHandle &nh) {
    // Grab some configuration parameters for this component
    cfg_.AddFile("mobility/planner_trapezoidal.config");
    if (!cfg_.Initialize(nh)) {
      return false;
    }

    // Notify initialization complete
    FF_DEBUG_STREAM("Initialization complete");
    // Success
    return true;
  }

  double GetEpsilon() {
    return cfg_.Get<double>("epsilon");
  }

  void PlanCallback(ff_msgs::Plan::Goal const& goal) {
    // Do some basic error checks
    ff_msgs::Plan::Result plan_result;
    plan_result.response = RESPONSE::SUCCESS;
    if (goal.states.size() < 2)
      plan_result.response = RESPONSE::NOT_ENOUGH_STATES;
    if (goal.check_obstacles)
      plan_result.response = RESPONSE::OBSTACLES_NOT_SUPPORTED;
    if (plan_result.response < 0)
      return PlanResult(std::make_shared<ff_msgs::Plan::Result>(plan_result));

    // Save the information
    desired_vel_   = goal.desired_vel;
    desired_omega_ = goal.desired_omega;
    desired_accel_ = goal.desired_accel;
    desired_alpha_ = goal.desired_alpha;
    min_control_period_ = 1.0 / goal.desired_rate;
    // Setup header and keep track of time as we generate trapezoidal ramps
    plan_result.segment.clear();
    // Generate the trapezoidal ramp
    rclcpp::Time offset = goal.states.front().header.stamp;
    for (size_t i = 1; i < goal.states.size(); i++) {
      // Get the requested duration and offset
      double dt = (rclcpp::Time(goal.states[i].header.stamp) - rclcpp::Time(goal.states[i - 1].header.stamp)).seconds();
      // Extract affine transformations for the two poses
      Eigen::Affine3d tf_1 =
        msg_conversions::ros_pose_to_eigen_transform(goal.states[i-1].pose);
      Eigen::Affine3d tf_4 =
        msg_conversions::ros_pose_to_eigen_transform(goal.states[i].pose);
      Eigen::Vector3d delta = tf_4.translation() - tf_1.translation();
      // Generate a forward-only segment when translations are non-zero
      if (goal.faceforward &&
        delta.norm() > cfg_.Get<double>("faceforward_shim")) {
        // Get the current vector representing the forward direction
        Eigen::Vector3d vfwd = delta.normalized();
        Eigen::Vector3d vdown(0.0, 0.0, 1.0);
        Eigen::Vector3d vright(0.0, 1.0, 0.0);
        // Check that the direction of motion is not along the Z axis. In this
        // case the approach of taking the cross product with the world Z will
        // fail and we need to choose a different axis
        if (fabs(vdown.dot(vfwd)) < 1.0 - GetEpsilon()) {
          vright = vdown.cross(vfwd);
          vdown = vfwd.cross(vright);
          if (vdown.z() < 0) {
            vright = -vright;
            vdown = vfwd.cross(vright);
          }
        } else {
          vdown = vfwd.cross(vright);
          vright = vdown.cross(vfwd);
          if (vright.y() < 0) {
            vdown = -vdown;
            vright = vdown.cross(vfwd);
          }
        }
        // Make sure all vectors are normalized
        vfwd = vfwd.normalized();
        vright = vright.normalized();
        vdown = vdown.normalized();
        // Construct a rotation matrix
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
        // Default speeds are set by the planner configuration
        InsertTrapezoid(plan_result.segment, offset, dt, tf_1, tf_2,
                        desired_vel_, desired_omega_, desired_accel_, desired_alpha_,
                        min_control_period_, GetEpsilon());
        InsertTrapezoid(plan_result.segment, offset, dt, tf_2, tf_3,
                        desired_vel_, desired_omega_, desired_accel_, desired_alpha_,
                        min_control_period_, GetEpsilon());
        InsertTrapezoid(plan_result.segment, offset, dt, tf_3, tf_4,
                        desired_vel_, desired_omega_, desired_accel_, desired_alpha_,
                        min_control_period_, GetEpsilon());
      } else {
        // Fully-holonomic smear of delta across all axes
        InsertTrapezoid(plan_result.segment, offset, dt, tf_1, tf_4,
                        desired_vel_, desired_omega_, desired_accel_, desired_alpha_,
                        min_control_period_, GetEpsilon());
      }
    }
    // Special case: we might already be there
    if (plan_result.segment.size() < 2)
      plan_result.response = RESPONSE::ALREADY_THERE;
    // Callback with the result
    return PlanResult(std::make_shared<ff_msgs::Plan::Result>(plan_result));
  }

  // Called to interrupt the process
  void CancelCallback() {}

  void Error(std::string s) {
    FF_ERROR_STREAM(s);
  }
  void Warn(std::string s) {
    FF_WARN_STREAM(s);
  }
  void Debug(std::string s) {
    FF_DEBUG_STREAM(s);
  }

 protected:
  ff_util::ConfigServer cfg_;
  ff_util::FreeFlyerTimer timer_d_;
  double desired_vel_;
  double desired_omega_;
  double desired_accel_;
  double desired_alpha_;
  double min_control_period_;
};
}  // namespace planner_trapezoidal

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(planner_trapezoidal::PlannerTrapezoidalComponent)
