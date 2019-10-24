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

// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>

// Generic arm control messages
#include <sensor_msgs/JointState.h>

// Specific arm services
#include <ff_hw_msgs/SetJointMaxVelocity.h>

// STL includes
#include <string>

namespace gazebo {

/* The perching arm has two DYNAMIXEL XM 430 intelligent motors. They
   run an internal PID control loop to achieve velocity or position.
   Default velocity controller values:
    P : 100    -> 0.78125
    I : 1920   -> 0.029296875
    D : 0      -> 0
   Default position controller values
    P : 800    -> 6.25
    I : 0      -> 0
    D : 0      -> 0
  the control axis points along -Z when stowed. In the stowed position
  when looked from above as a Y shape, the 'left' and 'right' are
  correctly oriented. For this reason:
  JOINT                             OPEN  | CLOSED
    o gripper_left_proximal_joint    1.0  | 0.0
    o gripper_left_distal_joint      1.0  | 0.0
    o gripper_right_proximal_joint   0.0  | 1.0
    o gripper_right_distal_joint     0.0  | 1.0
  The incoming position must be mapped from input range to ratio
*/
class GazeboModelPluginPerchingArm : public FreeFlyerModelPlugin {
 public:
  enum Type { POSITION, VELOCITY, EFFORT };
  // Constructor
  GazeboModelPluginPerchingArm() :
    FreeFlyerModelPlugin("perching_arm", "", true),
    rate_(10.0), bay_(""),
    pid_prox_p_(6.25, 0.0, 0.1),
    pid_dist_p_(6.25, 0.0, 0.1),
    pid_gl_prox_p_(6.25, 0.0, 0.0),
    pid_gl_dist_p_(6.25, 0.0, 0.0),
    pid_gr_prox_p_(6.25, 0.0, 0.0),
    pid_gr_dist_p_(6.25, 0.0, 0.0) {}

  // Destructor
  ~GazeboModelPluginPerchingArm() {}

 protected:
  // Hard limits of the system
  static constexpr double PROXIMAL_DEPLOYED     = -1.57079;
  static constexpr double PROXIMAL_STOWED       = 1.57079;
  static constexpr double PROXIMAL_MIN_TILT     = -2.09440;
  static constexpr double PROXIMAL_MAX_TILT     = -1.04718;
  static constexpr double DISTAL_DEPLOYED       = 0.0;
  static constexpr double DISTAL_STOWED         = 0.0;
  static constexpr double DISTAL_MIN_PAN        = -1.57079;
  static constexpr double DISTAL_MAX_PAN        = 1.57079;
  static constexpr double GRIPPER_OPEN          = 100.0;
  static constexpr double GRIPPER_CLOSED        = 0.0;
  static constexpr double RPM_TO_RADS_PER_S     = 0.1047198;

  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh, physics::ModelPtr model,
    sdf::ElementPtr sdf) {
    // Get parameters
    if (sdf->HasElement("bay"))
      bay_ = sdf->Get<std::string>("bay");
    if (sdf->HasElement("rate"))
      rate_ = sdf->Get<double>("rate");
    double prox = PROXIMAL_STOWED;
    if (sdf->HasElement("proximal"))
      prox = sdf->Get<double>("proximal");
    double dist = DISTAL_STOWED;
    if (sdf->HasElement("distal"))
      dist = sdf->Get<double>("distal");
    double grip = GRIPPER_CLOSED;
    if (sdf->HasElement("gripper"))
      grip = sdf->Get<double>("gripper");

    // Set the initial joint positions
    model->GetJoint(bay_+"_arm_proximal_joint")->SetPosition(0, prox);
    model->GetJoint(bay_+"_arm_distal_joint")->SetPosition(0, dist);
    SetGripperPosition(grip);

    // Setup the eight PID controllers used in this driver
    model->GetJointController()->SetPositionPID(GetModel()->GetJoint(
      bay_+"_arm_proximal_joint")->GetScopedName(), pid_prox_p_);
    model->GetJointController()->SetPositionPID(GetModel()->GetJoint(
      bay_+"_arm_distal_joint")->GetScopedName(), pid_dist_p_);
    model->GetJointController()->SetPositionPID(GetModel()->GetJoint(
      bay_+"_gripper_left_proximal_joint")->GetScopedName(), pid_gl_prox_p_);
    model->GetJointController()->SetPositionPID(GetModel()->GetJoint(
      bay_+"_gripper_left_distal_joint")->GetScopedName(), pid_gl_dist_p_);
    model->GetJointController()->SetPositionPID(GetModel()->GetJoint(
      bay_+"_gripper_right_proximal_joint")->GetScopedName(), pid_gr_prox_p_);
    model->GetJointController()->SetPositionPID(GetModel()->GetJoint(
      bay_+"_gripper_right_distal_joint")->GetScopedName(), pid_gr_dist_p_);

    // Set the default joint controller values equal to the initial state
    model->GetJointController()->SetPositionTarget(GetModel()->GetJoint(
      bay_+"_arm_proximal_joint")->GetScopedName(), prox);
    model->GetJointController()->SetPositionTarget(GetModel()->GetJoint(
      bay_+"_arm_distal_joint")->GetScopedName(), dist);

    // Set the composite gripper goal
    SetGripperGoal(grip);

    // We're going to publish all joint states plus one composite state. The
    // gripper states are used by rviz to print how the gripper appears. The
    // composite state is used by the high level driver to monitor progress.
    joints_.push_back(GetModel()->GetJoint(
      bay_+"_arm_proximal_joint"));
    joints_.push_back(GetModel()->GetJoint(
      bay_+"_arm_distal_joint"));
    joints_.push_back(GetModel()->GetJoint(
      bay_+"_gripper_left_proximal_joint"));
    joints_.push_back(GetModel()->GetJoint(
      bay_+"_gripper_left_distal_joint"));
    joints_.push_back(GetModel()->GetJoint(
      bay_+"_gripper_right_proximal_joint"));
    joints_.push_back(GetModel()->GetJoint(
      bay_+"_gripper_right_distal_joint"));

    // Avoid resizing in each callback
    msg_.header.frame_id =  GetModel()->GetName();
    msg_.name.resize(joints_.size() + 1);
    msg_.position.resize(joints_.size() + 1);
    msg_.velocity.resize(joints_.size() + 1);
    msg_.effort.resize(joints_.size() + 1);

    // Create a joint state publisher for the arm
    pub_ = nh->advertise<sensor_msgs::JointState>("joint_states", 100, true);

    // Now register to be called back every time FAM has new wrench
    sub_ = nh->subscribe("joint_goals", 1,
      &GazeboModelPluginPerchingArm::GoalCallback, this);

    // Set the distal velocity
    srv_p_ = nh->advertiseService(SERVICE_HARDWARE_PERCHING_ARM_DIST_VEL,
      &GazeboModelPluginPerchingArm::SetDistVelCallback, this);

    // Set the proximal velocity
    srv_t_ = nh->advertiseService(SERVICE_HARDWARE_PERCHING_ARM_PROX_VEL,
      &GazeboModelPluginPerchingArm::SetProxVelCallback, this);

    // Periodic timer to send feedback to executive to avoid timeout
    timer_ = nh->createTimer(ros::Rate(rate_),
      &GazeboModelPluginPerchingArm::TimerCallback, this, false, true);
  }

  // Called on simulation reset
  void Reset() {}

  // GET THE VIRTUAL GRIPPER JOINT STATE

  // Get the position and veloctity of a virtual "gripper" joint, which
  // the callee can use to determine progress of gripper opening. Returns
  // the proportion of the angle (0 - 1) from low-stop to high-stop
  double GetGripperJointState(std::string const& name, Type type) {
    physics::JointPtr joint = GetModel()->GetJoint(name);
    // Get the joint limits
    double value = 0.0;
    switch (type) {
    case POSITION: {
      double lower = joint->GetLowerLimit(0).Radian();
      double upper = joint->GetUpperLimit(0).Radian();
      value = (joint->GetAngle(0).Radian() - lower) / (upper - lower);
      break;
    }
    case VELOCITY:
      value = joint->GetVelocity(0);
      break;
    case EFFORT:
      value = joint->GetForce(0);
      break;
    }
    // Work out the position
    return value;
  }

  // SET THE GRIPPER PID SETPOINTS

  // Set the joint angle based on a gripper position from 0 to 100
  void SetGripperJointGoal(std::string const& name, double position) {
    physics::JointPtr joint = GetModel()->GetJoint(name);
    // Get the joint limits
    double lower = joint->GetLowerLimit(0).Radian();
    double upper = joint->GetUpperLimit(0).Radian();
    double value = lower + position * (upper - lower);
    // Calculate the correct joint angle based on the position (0 - 100)
    GetModel()->GetJointController()->SetPositionTarget(
      joint->GetScopedName(), value);
  }

  // Control 4 PIDs on the gripper proximal and distal joints.
  void SetGripperGoal(double position) {
    // Throw out junk values not in range
    if (position < GRIPPER_CLOSED || position > GRIPPER_OPEN)
      return;
    // Save the gripper state
    gripper_ = position;
    // All other cases are actual requests
    double r = (position - GRIPPER_CLOSED) / (GRIPPER_OPEN - GRIPPER_CLOSED);
    SetGripperJointGoal(bay_+"_gripper_left_proximal_joint", r);
    SetGripperJointGoal(bay_+"_gripper_left_distal_joint", r);
    SetGripperJointGoal(bay_+"_gripper_right_proximal_joint", 1.0 - r);
    SetGripperJointGoal(bay_+"_gripper_right_distal_joint", 1.0 - r);
  }

  // SET THE ACTUAL GRIPPER JOINT POSITIONS

  // Set the joint angle based on a gripper position from 0 to 100
  void SetGripperJointPosition(std::string const& name, double position) {
    physics::JointPtr joint = GetModel()->GetJoint(name);
    double lower = joint->GetLowerLimit(0).Radian();
    double upper = joint->GetUpperLimit(0).Radian();
    double value = lower + position * (upper - lower);
    joint->SetPosition(0, value);
  }

  // Set the four gripper PIDs fromt the single virtual gripper position
  void SetGripperPosition(double position) {
    double r = (position - GRIPPER_CLOSED) / (GRIPPER_OPEN - GRIPPER_CLOSED);
    SetGripperJointPosition(bay_+"_gripper_left_proximal_joint", r);
    SetGripperJointPosition(bay_+"_gripper_left_distal_joint", r);
    SetGripperJointPosition(bay_+"_gripper_right_proximal_joint", 1.0 - r);
    SetGripperJointPosition(bay_+"_gripper_right_distal_joint", 1.0 - r);
  }

  // CALLBACKS

  // Called whenever a new control command is available
  void GoalCallback(sensor_msgs::JointState const& msg) {
    for (size_t i = 0; i < msg.name.size(); i++) {
      // Special case: virtual joint "gripper" actually controls for PIDs
      // in parallel to simulate a complete gripper opening and closing. We
      // abuse the sensor_msgs::JointState slightly by allowing the gripper
      // position to be specified alongside joint velocities. Apologies.
      if (msg.name[i] == bay_+"_gripper_joint") {
        if (msg.position.size() > i)
          SetGripperGoal(msg.position[i]);
        else
          NODELET_WARN("Gripper: only position control is supported");
      // The only other two states that are supported are the proximal and
      // distal joints of the arm.
      } else if (msg.name[i] == bay_+"_arm_proximal_joint"
              || msg.name[i] == bay_+"_arm_distal_joint") {
        if (msg.position.size() > i)
          GetModel()->GetJointController()->SetPositionTarget(GetModel()
            ->GetJoint(msg.name[i])->GetScopedName(), msg.position[i]);
        else
          NODELET_WARN("Joint: only position control is supported");
      // Catch all invalid joint states
      }
    }
  }

  // Called on every discrete time tick in the simulated world
  void TimerCallback(ros::TimerEvent const& event) {
    // Package all joint states, inclusind the left and right proximal
    // and distal joints of the gripper (for visualization reasons)
    msg_.header.stamp = ros::Time::now();
    size_t i = 0;
    for (; i < joints_.size(); i++) {
      msg_.name[i] = joints_[i]->GetName();
      msg_.position[i] = joints_[i]->GetAngle(0).Radian();
      msg_.velocity[i] = joints_[i]->GetVelocity(0);
      msg_.effort[i] = joints_[i]->GetForce(0);
    }
    // Set the virtual gripper state manually as the last element
    msg_.name[i] = bay_+"_gripper_joint";
    msg_.position[i] = gripper_;
    msg_.velocity[i] = 0;
    msg_.effort[i] = 0;
    // Publish the joint state
    pub_.publish(msg_);
  }

  // Set the pan velocity
  bool SetDistVelCallback(ff_hw_msgs::SetJointMaxVelocity::Request  &req,
                          ff_hw_msgs::SetJointMaxVelocity::Response &res) {
    // Set the velocity limit for the joint
    GetModel()->GetJoint(bay_+"_arm_distal_joint")
      ->SetVelocityLimit(0, req.rpm * RPM_TO_RADS_PER_S);
    // Success!
    res.success = true;
    res.status_message = "Success";
    return true;
  }

  // Set the tilt velocity
  bool SetProxVelCallback(ff_hw_msgs::SetJointMaxVelocity::Request  &req,
                          ff_hw_msgs::SetJointMaxVelocity::Response &res) {
    // Set the velocity limit for the joint
    GetModel()->GetJoint(bay_+"_arm_proximal_joint")
      ->SetVelocityLimit(0, req.rpm * RPM_TO_RADS_PER_S);
    // Success!
    res.success = true;
    res.status_message = "Success";
    return true;
  }

 private:
  double rate_;                   // Rate of joint state update
  std::string bay_;               // Prefix to avoid name collisions
  ros::Timer timer_;              // Timer for sending updates
  ros::Publisher pub_;            // Joint state publisher
  ros::Subscriber sub_;           // Joint goal subscriber
  ros::ServiceServer srv_p_;      // Set max pan velocity
  ros::ServiceServer srv_t_;      // Set max tilt velcoity
  physics::Joint_V joints_;       // List of joints in system
  sensor_msgs::JointState msg_;   // Joint state message
  common::PID pid_prox_p_;        // PID : arm proximal position
  common::PID pid_dist_p_;        // PID : arm distal position
  common::PID pid_gl_prox_p_;     // PID : gripper left proximal position
  common::PID pid_gl_dist_p_;     // PID : gripper left distal position
  common::PID pid_gr_prox_p_;     // PID : gripper right proximal position
  common::PID pid_gr_dist_p_;     // PID : gripper right distal position
  double gripper_;                // Gripper value
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginPerchingArm)

}   // namespace gazebo
