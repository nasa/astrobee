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

// Astrobee simulation API
#include <astrobee_gazebo/astrobee_gazebo.h>

// Standard messages
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>

// Finite state machine for tracking dock state
#include <ff_util/ff_fsm.h>

// Messages
#include <ff_hw_msgs/EpsBatteryLocation.h>
#include <ff_hw_msgs/EpsChannelState.h>
#include <ff_hw_msgs/EpsDockStateStamped.h>

// Services
#include <ff_hw_msgs/Reset.h>
#include <ff_hw_msgs/ConfigureSystemLeds.h>
#include <ff_hw_msgs/ConfigurePayloadPower.h>
#include <ff_hw_msgs/ConfigureAdvancedPower.h>
#include <ff_hw_msgs/GetBoardInfo.h>
#include <ff_hw_msgs/ClearTerminate.h>
#include <ff_hw_msgs/RingBuzzer.h>
#include <ff_hw_msgs/SetEnabled.h>
#include <ff_hw_msgs/GetBatteryStatus.h>
#include <ff_hw_msgs/GetTemperatures.h>
#include <ff_hw_msgs/Undock.h>

// STL includes
#include <string>
#include <thread>

namespace gazebo {

using FSM = ff_util::FSM;

/*
  Provides a simple EPS plugin for helping with the the docking procedure.
  Like the EPS, it publishes a dock state. To do this is it first determines
  the static location of the berths. When the freeflyer is "close to" the
  berth it synthetically adds a small force to replicate the effect of the
  docking magnets on the final approach. Once close enough that pin contact
  would be established, it moved to a "docking" state. After a threshold
  amount of time it finally moves to a "docked" state. If pushed off the
  dock, this event moves the state back to undocked.
*/

class GazeboModelPluginEps : public FreeFlyerModelPlugin {
 public:
  // All possible states the EPS node can be in
  enum : FSM::State {
    UNKNOWN       = 1,       // We don't know if we are docked or undocked
    UNDOCKED      = 2,       // We are undocked
    DOCKING       = 3,       // We are busy docking (see nearest_ for berth)
    DOCKED        = 4,       // We are docked (see nearest_ for current berth)
    UNDOCKING     = 5        // We are pushing off the dock
  };

  // All possible events that can occur
  enum : FSM::Event {
    SENSE_NEAR    = (1<<0),  // We are near to a berth (see nearest_ for berth)
    SENSE_FAR     = (1<<1),  // We are far from any berth
    UNDOCK        = (1<<2),  // We received an undock command
    TIMEOUT       = (1<<3)   // We can now move from the docking to docked state
  };

  // Constructor
  GazeboModelPluginEps() : FreeFlyerModelPlugin("eps_driver", "", true),
    fsm_(UNKNOWN, std::bind(&GazeboModelPluginEps::StateCallback,
      this, std::placeholders::_1, std::placeholders::_2)),
    rate_(10.0), distance_(0.05), delay_(5.0), lock_(false) {
      // In an unknown state, if we are sensed to be near or far from a berth
      // then update to either a docked or undocked state.
      fsm_.Add(UNKNOWN, SENSE_NEAR | SENSE_FAR,
        [this](FSM::Event const& event) -> FSM::State {
          if (event == SENSE_FAR) {
            // Destroy a virtual link
            Lock(false);
            return UNDOCKED;
          }
          // Destroy a virtual link
          Lock(true);
          // We are now docked (and magnetised)
          return DOCKED;
      });
      // In an undocked state, if we come near to some berth then we need
      // to start the docking process. This means creating a magnetic force
      // that pulls the freeflyer towards the dock and start a timer to
      // replicate the several second connecting delay.
      fsm_.Add(UNDOCKED, SENSE_NEAR,
        [this](FSM::Event const& event) -> FSM::State {
          // Setup a timer to expire after a givent delay
          timer_delay_.start();
          // Create a virtual link
          Lock(true);
          // We are now in the process of docking
          return DOCKING;
        });
      // In a DOCKING state if we receive a TIMEOUT event then we move to
      // a docked state. The magnetic force persists...
      fsm_.Add(DOCKING, TIMEOUT,
        [this](FSM::Event const& event) -> FSM::State {
          // We are not docked
          return DOCKED;
        });
      // In a DOCKED state if we receive a FAR message then we have undocked,
      // in an uncontrolled way. That is, we have pushed off dock. The key here
      // is that we don't release the magnets, so we might redock :)
      fsm_.Add(DOCKED, SENSE_FAR,
        [this](FSM::Event const& event) -> FSM::State {
          // Destroy a virtual link
          Lock(false);
          // Stop the timer that signals successful connection
          timer_delay_.stop();
          // Start a timer to prevent magnets from kicking back in
          return UNDOCKED;
      });
      // In a DOCKED state if we receive a UNDOCK message then we must undock
      // in an uncontrolled way. That is, we need to release the magnets, so
      // that the propulsion system can drive us off the dock.
      fsm_.Add(DOCKED, UNDOCK,
        [this](FSM::Event const& event) -> FSM::State {
          // Destroy a virtual link
          Lock(false);
          // Start a timer to prevent magnets from kicking back in
          timer_delay_.start();
          // We are now undocking
          return UNDOCKING;
      });
      // In an UNDOCKING state we are sensed as being far from any berth
      // then we can move to an UNDOCKED state.
      fsm_.Add(UNDOCKING, SENSE_FAR,
        [this](FSM::Event const& event) -> FSM::State {
          // Destroy a virtual link
          Lock(false);
          // Stop the magnetic timeout
          timer_delay_.stop();
          // Invalidate the berth
          nearest_ = berths_.end();
          // We are now undocked
          return UNDOCKED;
      });
      // In an UNDOCKING state if the timeout expires then we didn't manage to
      // actually disconnect from the dock. In this case the magnets must be
      // turned on to pull us back into the dock.
      fsm_.Add(UNDOCKING, TIMEOUT,
        [this](FSM::Event const& event) -> FSM::State {
          // Setup a timer to expire after a given delay
          timer_delay_.start();
          // We are now docking
          return DOCKING;
      });
    }

  // Destructor
  ~GazeboModelPluginEps() {
    // Gazebo 7.x -> 9.x migration
    // event::Events::DisconnectWorldUpdateEnd(connection_);
      connection_.reset();
    // end Gazebo 7.x -> 9.x migration
  }

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh,
    physics::ModelPtr model, sdf::ElementPtr sdf) {
    // Get parameters
    if (sdf->HasElement("rate"))
      rate_ = sdf->Get<double>("rate");
    if (sdf->HasElement("distance"))
      distance_ = sdf->Get<double>("distance");
    if (sdf->HasElement("delay"))
      delay_ = sdf->Get<double>("delay");
    // Setup telemetry publishers
    pub_ = nh->advertise<ff_hw_msgs::EpsDockStateStamped>(
      TOPIC_HARDWARE_EPS_DOCK_STATE, 1);
    // Provide an undock service to call to release the robot from the dock
    srv_ = nh->advertiseService(SERVICE_HARDWARE_EPS_UNDOCK,
      &GazeboModelPluginEps::UndockCallback, this);
    // Once we have berth locations start timer for checking dock status
    timer_delay_ = nh->createTimer(ros::Duration(delay_),
      &GazeboModelPluginEps::DelayCallback, this, true, false);
      // Once we have berth locations start timer for checking dock status
    timer_update_ = nh->createTimer(ros::Rate(rate_),
      &GazeboModelPluginEps::UpdateCallback, this, false, false);
    // Defer the extrinsics setup to allow plugins to load
    connection_ = event::Events::ConnectWorldUpdateEnd(
      std::bind(&GazeboModelPluginEps::BerthCallback, this));
  }

  // When the FSM state changes we get a callback here, so that we can send
  // feedback to any active client, print debug info and publish our state
  void StateCallback(FSM::State const& state, FSM::FSM::Event const& event) {
    // Debug events
    std::string str = "UNKNOWN";
    switch (event) {
    case SENSE_NEAR:              str = "NEAR";               break;
    case SENSE_FAR:               str = "FAR";                break;
    case TIMEOUT:                 str = "TIMEOUT";            break;
    case UNDOCK:                  str = "UNDOCK";             break;
    }
    NODELET_DEBUG_STREAM("Received event " << str);
    // Debug state changes
    switch (state) {
    case UNKNOWN:                 str = "UNKNOWN";            break;
    case UNDOCKED:                str = "UNDOCKED";           break;
    case DOCKING:                 str = "DOCKING";            break;
    case DOCKED:                  str = "DOCKED";             break;
    case UNDOCKING:               str = "UNDOCKING";          break;
    }
    NODELET_DEBUG_STREAM("State changed to " << str);
  }

  // Manage the extrinsics based on the sensor type
  void BerthCallback() {
    // Create a buffer and listener for TF2 transforms
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener listener(buffer);
    static geometry_msgs::TransformStamped tf;
    // Get extrinsics from framestore
    try {
      // Lookup the transform for dock/berth1
      tf = buffer.lookupTransform(
        "world", "dock/berth1/complete", ros::Time(0));
      // Handle the transform for all sensor types
      berths_["dock/berth1/complete"] = ignition::math::Pose3d(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z,
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z);
      // Lookup the transform for dock/berth2
      tf = buffer.lookupTransform(
        "world", "dock/berth2/complete", ros::Time(0));
      // Handle the transform for all sensor types
      berths_["dock/berth2/complete"] = ignition::math::Pose3d(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z,
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z);
      // Kill the connection when we have a dock pose
      // Gazebo 7.x -> 9.x migration
      // event::Events::DisconnectWorldUpdateEnd(connection_);
      connection_.reset();
      // end Gazebo 7.x -> 9.x migration
      // Once we have berth locations start timer for checking dock status
      timer_update_.start();
    // If we have an exception we need to quietly wait for transform(s)
    } catch (tf2::TransformException &ex) {}
  }

  // Create a virtual joint to lock the freeflyer to the berth
  void Lock(bool enable) {
    if (enable) {
      // We are not guaranteed to have a dock yet, so we need to check to see
      // that the model pointer is valid. If it is valid, then we to quietly
      // ignore locking for the time being.
      // Gazebo 7.x -> 9.x migration
      // physics::ModelPtr dock = GetWorld()->GetModel("dock");
      physics::ModelPtr dock = GetWorld()->ModelByName("dock");
      // end Gazebo 7.x -> 9.x migration
      if (dock == nullptr)
        return;
      // By this point we are guaranteed to have a dock
      // Gazebo 7.x -> 9.x migration
      // joint_ = GetWorld()->GetPhysicsEngine()->CreateJoint("fixed", GetModel());
      joint_ = GetWorld()->Physics()->CreateJoint("fixed", GetModel());
      // end Gazebo 7.x -> 9.x migration
      joint_->Attach(GetModel()->GetLink(), dock->GetLink());
    } else if (joint_) {
      joint_->Detach();
      joint_->Fini();
    }
  }

  // Called when we have berth transforms populated
  void UpdateCallback(const ros::TimerEvent& event) {
    // Iterate over the berths to check if any are within a threshold distance.
    // There are smarter ways to do this sort of search (kNN) but this we are
    // only expecting fewer than 6 berths, it seems like needless optimization.
    for (nearest_ = berths_.begin(); nearest_ != berths_.end(); nearest_++) {
      // Gazebo 7.x -> 9.x migration
      // if (GetModel()->GetWorldPose().Ign().Pos().Distance(
      //   nearest_->second.Pos()) > distance_) continue;
      if (GetModel()->WorldPose().Pos().Distance(
        nearest_->second.Pos()) > distance_) continue;
      // end Gazebo 7.x -> 9.x migration
      // Now, send an event to the FSM to signal that we are close!
      fsm_.Update(SENSE_NEAR);
      // There should always only be one dock that we are close to
      break;
    }
    // Send an FAR event if we aren't close to any berth
    if (nearest_ == berths_.end())
      fsm_.Update(SENSE_FAR);
    // Whatever the result, publish the dock state to be used by other entites
    // in the system, and in particular the dock procedure.
    ff_hw_msgs::EpsDockStateStamped msg;
    msg.header.frame_id = GetPlatform();
    msg.header.stamp = ros::Time::now();
    switch (fsm_.GetState()) {
    case UNDOCKED:
    case UNDOCKING:
      msg.state = ff_hw_msgs::EpsDockStateStamped::UNDOCKED;    break;
    case DOCKING:
      msg.state = ff_hw_msgs::EpsDockStateStamped::CONNECTING;  break;
    case DOCKED:
      msg.state = ff_hw_msgs::EpsDockStateStamped::DOCKED;      break;
    default:
      msg.state = ff_hw_msgs::EpsDockStateStamped::UNKNOWN;     break;
    }
    pub_.publish(msg);
  }

  // Called when we have berth transforms populated
  void DelayCallback(const ros::TimerEvent& event) {
    fsm_.Update(TIMEOUT);
  }

  // Callback for the undock service
  bool UndockCallback(ff_hw_msgs::Undock::Request &req,
                      ff_hw_msgs::Undock::Response &res) {
    // We cannot undock in certain states
    switch (fsm_.GetState()) {
    case UNKNOWN:
    default:
      res.value = ff_hw_msgs::Undock::Response::CANNOT_QUERY_STATE;
      return true;
    case DOCKING:
    case UNDOCKED:
    case UNDOCKING:
      res.value = ff_hw_msgs::Undock::Response::NOT_DOCKED;
      return true;
    case DOCKED:
      break;
    }
    // Send an undock command to the FSM
    fsm_.Update(UNDOCK);
    res.value = ff_hw_msgs::Undock::Response::SUCCESS;
    return true;
  }

 private:
  ff_util::FSM fsm_;
  double rate_, distance_, delay_;
  event::ConnectionPtr connection_;
  std::map<std::string, ignition::math::Pose3d> berths_;
  std::map<std::string, ignition::math::Pose3d>::iterator nearest_;
  ignition::math::Vector3d force_;
  physics::JointPtr joint_;
  ros::Timer timer_update_, timer_delay_;
  ros::Publisher pub_;
  ros::ServiceServer srv_;
  bool lock_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginEps)

}   // namespace gazebo
