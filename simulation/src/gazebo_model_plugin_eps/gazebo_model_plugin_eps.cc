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
#include <ff_hw_msgs/EpsHousekeeping.h>
#include <ff_hw_msgs/EpsPowerState.h>
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
  the static location of the berths. When the astrobee is "close to" the
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
    rate_(10.0), distance_(0.05), delay_(5.0), lock_(false),
    battery_capacity_(3.4), battery_charge_(3.0),
    battery_discharge_rate_(0.005) {
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
          timer_delay_.stop();
          timer_delay_.start();
          ros::Duration(1.0).sleep();
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
      // is that we don't release the magnets, so we might redock.
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
          timer_delay_.stop();
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
          timer_delay_.stop();
          timer_delay_.start();
          // We are now docking
          return DOCKING;
      });
    }

  // Destructor
  ~GazeboModelPluginEps() {
    #if GAZEBO_MAJOR_VERSION > 7
    update_.reset();
    #else
    event::Events::DisconnectWorldUpdateEnd(update_);
    #endif
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
    pub_dock_state_ = nh->advertise<ff_hw_msgs::EpsDockStateStamped>(
      TOPIC_HARDWARE_EPS_DOCK_STATE, 1);
    pub_housekeeping_ = nh->advertise<ff_hw_msgs::EpsHousekeeping>(
      TOPIC_HARDWARE_EPS_HOUSEKEEPING, 1);
    pub_power_ = nh->advertise<ff_hw_msgs::EpsPowerState>(
      TOPIC_HARDWARE_EPS_POWER_STATE, 1);
    battery_state_pub_tl_ = nh->advertise<sensor_msgs::BatteryState>(
      TOPIC_HARDWARE_EPS_BATTERY_STATE_TL, 5);
    battery_state_pub_tr_ = nh->advertise<sensor_msgs::BatteryState>(
      TOPIC_HARDWARE_EPS_BATTERY_STATE_TR, 5);
    battery_state_pub_bl_ = nh->advertise<sensor_msgs::BatteryState>(
      TOPIC_HARDWARE_EPS_BATTERY_STATE_BL, 5);
    battery_state_pub_br_ = nh->advertise<sensor_msgs::BatteryState>(
      TOPIC_HARDWARE_EPS_BATTERY_STATE_BR, 5);
    // Provide an undock service to call to release the robot from the dock
    srv_undock_ = nh->advertiseService(
      SERVICE_HARDWARE_EPS_UNDOCK,
        &GazeboModelPluginEps::UndockCallback, this);
    // Enable payload power toggling
    srv_payload_ = nh->advertiseService(
      SERVICE_HARDWARE_EPS_CONF_PAYLOAD_POWER,
        &GazeboModelPluginEps::PayloadConfigureCallback, this);
    // Enable advanced power toggling
    srv_power_ = nh->advertiseService(
      SERVICE_HARDWARE_EPS_CONF_ADVANCED_POWER,
        &GazeboModelPluginEps::PowerConfigureCallback, this);
    srv_led_ = nh->advertiseService(SERVICE_HARDWARE_EPS_CONF_LED_STATE,
      &GazeboModelPluginEps::LedsConfigureCallback, this);
    // Once we have berth locations start timer for checking dock status
    timer_delay_ = nh->createTimer(ros::Duration(delay_),
      &GazeboModelPluginEps::DelayCallback, this, true, false);
      // Once we have berth locations start timer for checking dock status
    timer_update_ = nh->createTimer(ros::Rate(rate_),
      &GazeboModelPluginEps::UpdateCallback, this, false, false);
    // Create timer to publish battery states
    telem_timer_= nh->createTimer(ros::Duration(5),
      &GazeboModelPluginEps::TelemetryCallback, this, false, true);
     // Defer the extrinsics setup to allow plugins to load
    update_ = event::Events::ConnectWorldUpdateEnd(
      std::bind(&GazeboModelPluginEps::BerthCallback, this));
    // Initialize battery states
    state_tl_.header.stamp = ros::Time::now();
    state_tl_.location = ff_hw_msgs::EpsBatteryLocation::TOP_LEFT;
    state_tl_.present = sdf->Get<bool>("battery_top_left");
    state_tl_.capacity = battery_capacity_;
    state_tl_.charge = battery_charge_;
    state_tl_.percentage = state_tl_.charge / state_tl_.capacity;
    battery_state_pub_tl_.publish(state_tl_);
    state_tr_.header.stamp = ros::Time::now();
    state_tr_.location = ff_hw_msgs::EpsBatteryLocation::TOP_RIGHT;
    state_tr_.present = sdf->Get<bool>("battery_top_right");
    state_tr_.capacity = battery_capacity_;
    state_tr_.charge = battery_charge_;
    state_tr_.percentage = state_tr_.charge / state_tl_.capacity;
    battery_state_pub_tr_.publish(state_tr_);
    state_bl_.header.stamp = ros::Time::now();
    state_bl_.location = ff_hw_msgs::EpsBatteryLocation::BOTTOM_LEFT;
    state_bl_.present = sdf->Get<bool>("battery_bottom_left");
    state_bl_.capacity = battery_capacity_;
    state_bl_.charge = battery_charge_;
    state_bl_.percentage = state_bl_.charge / state_bl_.capacity;
    battery_state_pub_bl_.publish(state_bl_);
    state_br_.header.stamp = ros::Time::now();
    state_br_.location = ff_hw_msgs::EpsBatteryLocation::BOTTOM_RIGHT;
    state_br_.present = sdf->Get<bool>("battery_bottom_right");
    state_br_.capacity = battery_capacity_;
    state_br_.charge = battery_charge_;
    state_br_.percentage = state_br_.charge /state_br_.capacity;
    battery_state_pub_br_.publish(state_br_);
    // Set default values for housekeeping
    housekeeping_["AGND1_V"] = 0.0000;
    housekeeping_["SUPPLY_IN_V"] = 0.4640;
    housekeeping_["PAYLOAD_PWR3_I"] = 0.0030;
    housekeeping_["SUBSYS1_1_PWR_V"] = 14.9820;
    housekeeping_["SUBSYS1_2_PWR_V"] = 14.9660;
    housekeeping_["UNREG_V"] = 15.0150;
    housekeeping_["SYSTEM_I"] = 1.0960;
    housekeeping_["BAT4V_V"] = 0.0;
    housekeeping_["BAT3V_V"] = 0.0;
    housekeeping_["BAT2V_V"] = 0.0;
    housekeeping_["BAT1V_V"] = 0.0;
    housekeeping_["SUPPLY_I"] = 0.1790;
    housekeeping_["5VLIVE_V"] = 4.9740;
    housekeeping_["AGND2_V"] =  4.9800;
    housekeeping_["FAN_PWR_I"] = 0.3150;
    housekeeping_["AUX_PWR_I"] = 0.0120;
    housekeeping_["PAYLOAD_PWR4_I"] = 0.0;
    housekeeping_["PAYLOAD_PWR2_I"] = 0.0;
    housekeeping_["PAYLOAD_PWR1_I"] = 0.0;
    housekeeping_["5A_REG1_PWR_I"] = 0.4410;
    housekeeping_["MOTOR1_I"] = 0.0770;
    housekeeping_["SUBSYS2_PWR_V"] = 14.9820;
    housekeeping_["MOTOR2_I"] = 0.0740;
    housekeeping_["5A_REG2_PWR_I"] = 0.3750;
    housekeeping_["5A_REG3_PWR_I"] = 0.0100;
    housekeeping_["MAIN5_PWR_I"] = 0.5720;
    housekeeping_["AUO_PWR_I"] = 0.0050;
    housekeeping_["HLP_I"] = 0.1930;
    housekeeping_["USB_PWR_I"] = 0.7830;
    housekeeping_["LLP_I"] = 0.4000;
    housekeeping_["MLP_I"] = 0.1900;
    housekeeping_["ENET_PWR_I"] = 0.1290;
    // Set sdefault power channel states
    power_states_["LLP_EN"] = true;
    power_states_["MLP_EN"] = true;
    power_states_["HLP_EN"] = true;
    power_states_["USB_PWR_EN"] = true;
    power_states_["AUX_PWR_EN"] = true;
    power_states_["ENET_PWR_EN"] = true;
    power_states_["FAN_EN"] = true;
    power_states_["SPEAKER_EN"] = true;
    power_states_["PAYLOAD_EN_TOP_AFT"] = false;
    power_states_["PAYLOAD_EN_BOT_AFT"] = false;
    power_states_["PAYLOAD_EN_BOT_FRONT"] = false;
    power_states_["PAYLOAD_EN_TOP_FRONT"] = true;
    power_states_["MOTOR_EN1"] = true;
    power_states_["MOTOR_EN2"] = true;
    power_states_["RESERVED0"] = false;
    power_states_["RESERVED1"] = false;
    power_states_["STATUSA2_LED"] = true;
    power_states_["STATUSA1_LED"] = true;
    power_states_["STATUSB2_LED"] = true;
    power_states_["STATUSB1_LED"] = true;
    power_states_["STATUSC2_LED"] = true;
    power_states_["STATUSC1_LED"] = true;
    power_states_["RESERVED2"] = false;
    power_states_["RESERVED3"] = false;
    power_states_["VIDEO_LED"] = true;
    power_states_["AUDIO_LED"] = true;
    power_states_["LIVE_LED"] = true;
    power_states_["RESERVED4"] = false;
    power_states_["RESERVED5"] = false;
    power_states_["RESERVED6"] = false;
    power_states_["RESERVED7"] = false;
    power_states_["RESERVED8"] = false;
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
      #if GAZEBO_MAJOR_VERSION > 7
      update_.reset();
      #else
      event::Events::DisconnectWorldUpdateEnd(update_);
      #endif
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
      #if GAZEBO_MAJOR_VERSION > 7
      physics::ModelPtr dock = GetWorld()->ModelByName("dock");
      #else
      physics::ModelPtr dock = GetWorld()->GetModel("dock");
      #endif
      if (dock == nullptr)
        return;
      // By this point we are guaranteed to have a dock
      #if GAZEBO_MAJOR_VERSION > 7
      joint_ = GetWorld()->Physics()->CreateJoint("fixed", GetModel());
      #else
      joint_ = GetWorld()->GetPhysicsEngine()->CreateJoint("fixed", GetModel());
      #endif
      joint_->Attach(GetModel()->GetLink(), dock->GetLink());
      // If we have an air carriage, stop colliding with anything
      physics::LinkPtr link = GetModel()->GetLink("body");
      if (link)
        link->SetCollideMode("none");
    } else if (joint_) {
      joint_->Detach();
      joint_->Fini();
      // If we have an air carriage, start colliding with everything
      physics::LinkPtr link = GetModel()->GetLink("body");
      if (link)
        link->SetCollideMode("all");
    }
  }

  // Called when we have berth transforms populated
  void UpdateCallback(const ros::TimerEvent& event) {
    // Iterate over the berths to check if any are within a threshold distance.
    // There are smarter ways to do this sort of search (kNN) but this we are
    // only expecting fewer than 6 berths, it seems like needless optimization.
    bool near = false;
    for (nearest_ = berths_.begin(); nearest_ != berths_.end(); nearest_++) {
      #if GAZEBO_MAJOR_VERSION > 7
      if (GetModel()->WorldPose().Pos().Distance(
          nearest_->second.Pos()) > distance_) continue;
      #else
      if (GetModel()->GetWorldPose().Ign().Pos().Distance(
        nearest_->second.Pos()) > distance_) continue;
      #endif
      // Now, send an event to the FSM to signal that we are close!
      fsm_.Update(SENSE_NEAR);
      // There should always only be one dock that we are close to
      near = true;
      break;
    }

    // Send an FAR event if we aren't close to any berth
    if (!near)
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
    pub_dock_state_.publish(msg);
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

  // Callback for setting the power state
  bool PayloadConfigureCallback(
      ff_hw_msgs::ConfigurePayloadPower::Request &req,
      ff_hw_msgs::ConfigurePayloadPower::Response &res) {
    // Batch the request
    uint8_t const &on = ff_hw_msgs::ConfigurePayloadPower::Request::ON;
    uint8_t const &off = ff_hw_msgs::ConfigurePayloadPower::Request::OFF;
    // Top front
    if (req.top_front == on)
      power_states_["PAYLOAD_EN_TOP_FRONT"] = true;
    if (req.top_front == off)
      power_states_["PAYLOAD_EN_TOP_FRONT"] = false;
    // Top aft
    if (req.top_aft == on)
      power_states_["PAYLOAD_EN_TOP_AFT"] = true;
    if (req.top_aft == off)
      power_states_["PAYLOAD_EN_TOP_AFT"] = false;
    // Bottom aft
    if (req.bottom_aft == on)
      power_states_["PAYLOAD_EN_BOT_AFT"] = true;
    if (req.bottom_aft == off)
      power_states_["PAYLOAD_EN_BOT_AFT"] = false;
    // Bottom front
    if (req.bottom_front == on)
      power_states_["PAYLOAD_EN_BOT_FRONT"] = true;
    if (req.bottom_front == off)
      power_states_["PAYLOAD_EN_BOT_FRONT"] = false;
    // Success!
    res.success = true;
    res.status = "All payload power set sucessfully";
    return true;
  }

  // Callback for setting the power channels
  bool PowerConfigureCallback(
      ff_hw_msgs::ConfigureAdvancedPower::Request &req,
      ff_hw_msgs::ConfigureAdvancedPower::Response &res) {
    // Batch the request
    uint8_t const &on = ff_hw_msgs::ConfigureAdvancedPower::Request::ON;
    uint8_t const &off = ff_hw_msgs::ConfigureAdvancedPower::Request::OFF;
    // Set the states
    if (req.usb == on)
      power_states_["USB_PWR_EN"] = true;
    if (req.usb == off)
      power_states_["USB_PWR_EN"] = false;
    if (req.aux == on)
      power_states_["AUX_PWR_EN"] = true;
    if (req.aux == off)
      power_states_["AUX_PWR_EN"] = false;
    if (req.pmc1 == on)
      power_states_["MOTOR_EN1"] = true;
    if (req.pmc1 == off)
      power_states_["MOTOR_EN1"] = false;
    if (req.pmc2 == on)
      power_states_["MOTOR_EN2"] = true;
    if (req.pmc2 == off)
      power_states_["MOTOR_EN2"] = false;
    // Success!
    res.success = true;
    res.status = "All advanced power set sucessfully";
    return true;
  }

  bool LedsConfigureCallback(ff_hw_msgs::ConfigureSystemLeds::Request &req,
                             ff_hw_msgs::ConfigureSystemLeds::Response &res) {
    // TODO(?) Maybe have actual leds on the simulated robot that turn on
    res.success = true;
    res.status = "All LED set successfully";
    return true;
  }

  // Callback for telemetry broadcast
  void TelemetryCallback(const ros::TimerEvent &event) {
    // Set a header for all telemetry items
    static std_msgs::Header header;
    header.frame_id = GetPlatform();
    header.stamp = ros::Time::now();

    // Send battery
    if (state_tl_.present) {
      state_tl_.charge -= battery_discharge_rate_;
      if (state_tl_.charge < 0) {
        state_tl_.charge = battery_charge_;
      }
      state_tl_.percentage = state_tl_.charge / state_tl_.capacity;
    }
    // Always publish every battery state regardless if it is present or not
    // since the actual eps driver does this
    state_tl_.header = header;
    battery_state_pub_tl_.publish(state_tl_);

    if (state_tr_.present) {
      state_tr_.charge -= battery_discharge_rate_;
      if (state_tr_.charge < 0) {
        state_tr_.charge = battery_charge_;
      }
      state_tr_.percentage = state_tr_.charge / state_tr_.capacity;
    }
    state_tr_.header = header;
    battery_state_pub_tr_.publish(state_tr_);

    if (state_bl_.present) {
      state_bl_.charge -= battery_discharge_rate_;
      if (state_bl_.charge < 0) {
        state_bl_.charge = battery_charge_;
      }
      state_bl_.percentage = state_bl_.charge / state_bl_.capacity;
    }
    state_bl_.header = header;
    battery_state_pub_bl_.publish(state_bl_);

    if (state_br_.present) {
      state_br_.charge -= battery_discharge_rate_;
      if (state_br_.charge < 0) {
        state_br_.charge = battery_charge_;
      }
      state_br_.percentage = state_br_.charge / state_br_.capacity;
    }
    state_br_.header.stamp = ros::Time::now();
    battery_state_pub_br_.publish(state_br_);

    // Send housekeeping
    static ff_hw_msgs::EpsHousekeeping msg_housekeeping;
    msg_housekeeping.header = header;
    housekeeping_["BAT4V_V"] = state_tr_.present ? 14.9 : 0.0;  // CHECK!!!
    housekeeping_["BAT3V_V"] = state_tr_.present ? 14.9 : 0.0;  // CHECK!!!
    housekeeping_["BAT2V_V"] = state_bl_.present ? 14.9 : 0.0;  // CHECK!!!
    housekeeping_["BAT1V_V"] = state_br_.present ? 14.9 : 0.0;  // CHECK!!!
    housekeeping_["PAYLOAD_PWR4_I"] =
      power_states_["PAYLOAD_EN_TOP_AFT"] ? 1.0 : 0.0;  // CHECK!!!
    housekeeping_["PAYLOAD_PWR2_I"] =
      power_states_["PAYLOAD_EN_BOT_AFT"] ? 1.0 : 0.0;  // CHECK!!!
    housekeeping_["PAYLOAD_PWR1_I"] =
      power_states_["PAYLOAD_EN_BOT_FRONT"] ? 1.0 : 0.0;  // CHECK!!!
    msg_housekeeping.values.clear();
    for (auto it = housekeeping_.begin(); it != housekeeping_.end(); it++) {
      static ff_hw_msgs::EpsHousekeepingValue kv;
      kv.name = it->first;
      kv.value = it->second;
      msg_housekeeping.values.push_back(kv);
    }
    pub_housekeeping_.publish(msg_housekeeping);

    // Send power
    static ff_hw_msgs::EpsPowerState msg_power;
    msg_power.header = header;
    msg_power.values.clear();
    for (auto it = power_states_.begin(); it != power_states_.end(); it++) {
      static ff_hw_msgs::EpsPowerStateValue kv;
      kv.name = it->first;
      kv.value = it->second;
      msg_power.values.push_back(kv);
    }
    pub_power_.publish(msg_power);
  }

 private:
  ff_util::FSM fsm_;
  double rate_, distance_, delay_;
  bool lock_;
  double battery_capacity_, battery_charge_, battery_discharge_rate_;
  event::ConnectionPtr update_;
  std::map<std::string, ignition::math::Pose3d> berths_;
  std::map<std::string, ignition::math::Pose3d>::iterator nearest_;
  ignition::math::Vector3d force_;
  physics::JointPtr joint_;
  ros::Timer timer_update_, timer_delay_, telem_timer_;
  ros::Publisher pub_dock_state_, pub_housekeeping_, pub_power_;
  ros::Publisher battery_state_pub_tl_, battery_state_pub_tr_;
  ros::Publisher battery_state_pub_bl_, battery_state_pub_br_;
  ros::ServiceServer srv_undock_, srv_power_, srv_payload_, srv_led_;
  sensor_msgs::BatteryState state_tl_, state_tr_, state_bl_, state_br_;
  std::map<std::string, bool> power_states_;
  std::map<std::string, double> housekeeping_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginEps)

}   // namespace gazebo
