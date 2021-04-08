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

// Standard ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>

// FSW nodelet
#include <ff_util/ff_nodelet.h>

// PMC helper libraries
#include <pmc_actuator/pmc_actuator.h>

// Messages
#include <ff_hw_msgs/PmcCommand.h>
#include <ff_hw_msgs/PmcTelemetry.h>
#include <ff_hw_msgs/PmcState.h>

// Services
#include <ff_msgs/SetBool.h>
#include <ff_msgs/SetFloat.h>

#include <cerrno>
#include <cstring>

/**
 * \ingroup hardware
 */
namespace pmc_actuator {

class PmcActuatorNodelet : public ff_util::FreeFlyerNodelet {
 public:
  // As per FFFTEST003
  static constexpr  int32_t MAX_TRIM   = 64;
  static constexpr uint32_t NUM_NOZZLE = 6;
  enum PmcType  : uint32_t { PMC_STBD, PMC_PORT,  NUM_PMC  };
  enum TrimType : uint32_t { TRIM_LOW, TRIM_HIGH, NUM_TRIM };
  enum StbdType : uint32_t { RXP, RXN, FYP, AYP, RZP, RZN  };
  enum PortType : uint32_t { LXN, LXP, AYN, FYN, LZP, LZN  };

  // Constructor
  PmcActuatorNodelet() : ff_util::FreeFlyerNodelet(NODE_PMC_ACTUATOR),
    pmc_enabled_(true), cur_command_id_(0) {}

  // Destructor - clean up the dynamically allocated PMC and command arrays
  ~PmcActuatorNodelet() {
    std::vector < PmcActuator* > ::iterator it_pmc;
    for (it_pmc = pmcs_.begin(); it_pmc != pmcs_.end(); it_pmc++)
      delete *it_pmc;
    std::vector < Command* > ::iterator it_cmd;
    for (it_cmd = commands_.begin(); it_cmd != commands_.end(); it_cmd++)
      delete *it_cmd;
  }

 protected:
  void Initialize(ros::NodeHandle *nh) {
    config_params.AddFile("hw/pmc_actuator.config");

    // Try and read the configuration parameters
    if (!GetParams()) {
      ROS_ERROR("PMC Actuator: Failed to get parameters.");
      Exit(EXIT_FAILURE);
    }

    // Try and initialize the i2c bus
    if (!Init()) {
      ROS_ERROR("PMC Actuator: Failed to initialize.");
      Exit(EXIT_FAILURE);
    }

    // Set the watchdog period base don the control rate
    watchdog_period_ = ros::Duration(20.0/control_rate_hz_);

    // Command subscriber
    sub_command_ = nh->subscribe(TOPIC_HARDWARE_PMC_COMMAND, sub_queue_size_,
      &PmcActuatorNodelet::CommandCallback, this,
        ros::TransportHints().tcpNoDelay());

    // Telemetry publisher
    pub_telemetry_ = nh->advertise<ff_hw_msgs::PmcTelemetry>(
      TOPIC_HARDWARE_PMC_TELEMETRY, pub_queue_size_);

    // State publisher as a latched topic
    pub_state_ = nh->advertise<ff_hw_msgs::PmcState>(
      TOPIC_HARDWARE_PMC_STATE, pub_queue_size_, true);

    // PMC enable/disable service
    srv_ = nh->advertiseService(
      SERVICE_HARDWARE_PMC_ENABLE, &PmcActuatorNodelet::EnableService, this);

    // Update PMC watchdog timer timeout
    srv_ = nh->advertiseService(
      SERVICE_HARDWARE_PMC_MIN_RATE, &PmcActuatorNodelet::MinRateService, this);

    // Watchdog timer
    timer_ = nh->createTimer(
      watchdog_period_, &PmcActuatorNodelet::TimerCallback, this, false, true);

    // Set all initial PMC states to UNKNOWN and publish
    state_.header.frame_id = frame_id_;
    state_.header.stamp = ros::Time::now();
    state_.states.resize(num_pmcs_);
    for (uint32_t i = 0; i < num_pmcs_; i++)
      state_.states[i] = ff_hw_msgs::PmcState::UNKNOWN;
    pub_state_.publish(state_);
  }

  // Read the configuration from the LUA config file
  bool GetParams() {
    // Read all config files
    if (!config_params.ReadFiles()) {
      ROS_FATAL("PMC Actuator: Unable to load lua parameters!");
      return false;
    }

    // Not setting up reload timer for parameters changes during execution since
    // it seems like it would disturb the main loop which may be undesirable

    // get number of pmcs
    int num_pmcs = 0;
    if (!config_params.GetInt("num_pmcs", &num_pmcs)) {
      ROS_FATAL("PMC Actuator: number of pmcs not specified!");
      return false;
    }
    if (num_pmcs != NUM_PMC) {
      ROS_FATAL("PMC Actuator: wrong number of PMCs found!");
      return false;
    }
    num_pmcs_ = static_cast<uint32_t>(num_pmcs);

    // get frame id
    if (!config_params.GetStr("frame_id", &frame_id_)) {
      ROS_FATAL("PMC Actuator: frame id not specified!");
      return false;
    }

    // get sub topic stuff
    if (!config_params.GetInt("sub_queue_size", &sub_queue_size_)) {
      sub_queue_size_ = 10;
    }

    // get pub topic stuff
    if (!config_params.GetInt("pub_queue_size", &pub_queue_size_)) {
      pub_queue_size_ = 10;
    }

    // get bus file
    if (!config_params.GetStr("i2c_bus_file", &i2c_bus_file_)) {
      ROS_FATAL("PMC Actuator: i2c bus file not specified!");
      return false;
    }

    // get i2c addresses
    config_reader::ConfigReader::Table i2c_addresses(
      &config_params, "i2c_addrs");
    int temp_addr;
    // Lua table indices start at 1
    for (int i = 1; i < (i2c_addresses.GetSize() + 1); i++) {
      if (!i2c_addresses.GetInt(i, &temp_addr)) {
        ROS_FATAL("PMC Actuator: i2c address not an integer!");
        return false;
      }
      i2c_addrs_.push_back(temp_addr);
    }

    // get i2c retries
    if (!config_params.GetInt("i2c_retries", &i2c_retries_)) {
      ROS_FATAL("PMC Actuator: i2c retries not specified!");
      return false;
    }

    // get control rate
    if (!config_params.GetPosReal("control_rate_hz", &control_rate_hz_)) {
      ROS_FATAL("PMC Actuator: control rate not specified!");
      return false;
    }

    // get minimum allowed control rate
    if (!config_params.GetPosReal("control_min_rate_hz", &control_min_rate_hz_)) {
      ROS_FATAL("PMC Actuator: control rate not specified!");
      return false;
    }

    // get initial fan speed
    if (!config_params.GetInt("null_speed", &null_fan_speed_)) {
      ROS_FATAL("PMC Actuator: null fan speed not specified!");
      return false;
    }

    // get values used to calculate state
    if (!config_params.GetReal("state_command_scale",
      &state_command_scale_)) {
      ROS_FATAL("PMC Actuator: state_command_scale not specified!");
      return false;
    }
    if (!config_params.GetReal("state_telemetry_scale",
      &state_telemetry_scale_)) {
      ROS_FATAL("PMC Actuator: state_telemetry_scale not specified!");
      return false;
    }
    if (!config_params.GetReal("arrive_tol_rads_per_sec",
      &arrive_tol_rads_per_sec_)) {
      ROS_FATAL("PMC Actuator: arrive_tol_rads_per_sec not specified!");
      return false;
    }
    if (!config_params.GetReal("depart_tol_rads_per_sec",
      &depart_tol_rads_per_sec_)) {
      ROS_FATAL("PMC Actuator: depart_tol_rads_per_sec not specified!");
      return false;
    }

    // Get stbd nozzle trims
    config_reader::ConfigReader::Table stbd_trims(&config_params,
      "stbd_nozzle_trims");
     for (uint32_t i = 0; i < NUM_TRIM; i++) {
       config_reader::ConfigReader::Table nozzles;
       if (!stbd_trims.GetTable(i + 1, &nozzles)) {
         ROS_FATAL("PMC Actuator: cannot get stbd nozzle trims");
         return false;
       }
       int trim = 0;
       for (uint32_t j = 0; j < NUM_NOZZLE; j++) {
         if (!nozzles.GetInt(j + 1, &trim)) {
           ROS_FATAL("PMC Actuator: null nozzle position not an integer!");
           return false;
         }
         if (trim < 0 || trim > MAX_TRIM)
           ROS_FATAL("PMC Actuator: Trim values must be in range 0 - 32");
         trims_[PMC_STBD][i][j] = static_cast<uint8_t>(trim);
       }
    }

    // Get port nozzle trims
    config_reader::ConfigReader::Table port_trims(&config_params,
      "port_nozzle_trims");
     for (uint32_t i = 0; i < NUM_TRIM; i++) {
       config_reader::ConfigReader::Table nozzles;
       if (!port_trims.GetTable(i + 1, &nozzles)) {
         ROS_FATAL("PMC Actuator: cannot get port nozzle trims");
         return false;
       }
       int trim = 0;
       for (uint32_t j = 0; j < NUM_NOZZLE; j++) {
         if (!nozzles.GetInt(j + 1, &trim)) {
           ROS_FATAL("PMC Actuator: null nozzle position not an integer!");
           return false;
         }
         if (trim < 0 || trim > MAX_TRIM)
           ROS_FATAL("PMC Actuator: Trim values must be in range 0 - 32");
         trims_[PMC_PORT][i][j] = static_cast<uint8_t>(trim);
       }
    }

    // get intial nozzle positions
    config_reader::ConfigReader::Table positions(&config_params,
                                                 "null_nozzle_positions");
    int temp_pos;
    // Lua table indices start at 1
    for (int i = 1; i < (positions.GetSize() + 1); i++) {
      if (!positions.GetInt(i, &temp_pos)) {
        ROS_FATAL("PMC Actuator: null nozzle position not an integer!");
        return false;
      }
      null_nozzle_positions_.push_back(temp_pos);
    }

    if (i2c_addrs_.size() != num_pmcs_) {
      ROS_ERROR("Invalid parameters. Check num_pmcs and i2c_addrs to fix.");
      return false;
    }

    return true;
  }

  // Initialize the i2c bus
  bool Init(void) {
    i2c::Error err;

    // i2c::Open returns NULL in case of error.
    auto bus = i2c::Open(i2c_bus_file_, &err);

    if (!bus) {
      ROS_ERROR("Unable to open i2c_bus ('%s'): %s", i2c_bus_file_.c_str(),
                std::strerror(err));
      return false;
    }

    bus->SetRetries(i2c_retries_);

    for (uint32_t i = 0; i < num_pmcs_; i++) {
      pmcs_.push_back(new PmcActuator(bus->DeviceAt(i2c_addrs_.at(i))));
    }

    // Set initial commands
    for (uint32_t i = 0; i < num_pmcs_; i++) {
      commands_.push_back(new Command);
      commands_.at(i)->motor_speed = null_fan_speed_;
      for (uint32_t n = 0; n < null_nozzle_positions_.size(); n++)
        commands_.at(i)->nozzle_positions[n] = null_nozzle_positions_[n];
      commands_.at(i)->mode = CmdMode::SHUTDOWN;
      commands_.at(i)->command_id = cur_command_id_++;
      pmcs_.at(i)->SendCommand(*(commands_.at(i)));
    }

    return true;
  }

  // Safely exit
  void Exit(int status) {
    exit(status);
  }

  // Remap a command to lie in the correct range, taking into account trims
  uint8_t Trim(uint32_t p, uint32_t n, uint8_t x) {
    return static_cast<uint8_t>(x * (255 - trims_[p][TRIM_HIGH][n]
      - trims_[p][TRIM_LOW][n]) / 255 + trims_[p][TRIM_LOW][n]);
  }

  // Send the commands to the PMCs
  void SendAndPublish() {
    // Send the telemetry
    telemetry_vector_.header.seq++;
    telemetry_vector_.header.frame_id = frame_id_;
    telemetry_vector_.header.stamp = ros::Time::now();
    telemetry_vector_.statuses.clear();
    // Check if we need to change state based on the motor speed
    static ff_hw_msgs::PmcState msg;
    msg.states.resize(num_pmcs_);
    bool duplicate = true;
    for (uint32_t i = 0; i < num_pmcs_; i++) {
      ff_hw_msgs::PmcStatus t;
      if (!GetStatus(i, &t))
        ROS_WARN_THROTTLE(5, "Unable to get telemetry from PMCs");
      telemetry_vector_.statuses.push_back(t);
      // Determine the current state based on the different between the
      // commanded and telemetry motor speeds, scaled appropriately
      double crps = state_command_scale_ * commands_.at(i)->motor_speed;
      double trps = state_telemetry_scale_ * t.motor_speed;
      // Decide what to do based on the current state
      switch (msg.states[i]) {
      case ff_hw_msgs::PmcState::READY:
        if (crps - trps >  depart_tol_rads_per_sec_)        // +ve towards goal
          msg.states[i] = ff_hw_msgs::PmcState::RAMPING_UP;
        else if (crps - trps < -depart_tol_rads_per_sec_)   // -ve towards goal
          msg.states[i] = ff_hw_msgs::PmcState::RAMPING_DOWN;
        break;
      case ff_hw_msgs::PmcState::RAMPING_UP:
        if (crps - trps < arrive_tol_rads_per_sec_)         // +ve towards goal
          msg.states[i] = ff_hw_msgs::PmcState::READY;
        break;
      case ff_hw_msgs::PmcState::RAMPING_DOWN:
        if (crps - trps > -arrive_tol_rads_per_sec_)        // -ve towards goal
          msg.states[i] = ff_hw_msgs::PmcState::READY;
        break;
      default:  // Unknown
        msg.states[i] = ff_hw_msgs::PmcState::READY;
        break;
      }
      // One mismatch will trigger a state update
      duplicate &= (state_.states[i] == msg.states[i]);
    }
    // Send the PMC command
    for (uint32_t i = 0; i < num_pmcs_; i++) {
      if (!pmcs_.at(i)->SendCommand(*(commands_.at(i))))
        ROS_WARN_THROTTLE(5, "Unable to send command to PMCs");
    }
    // Publish a high-level state
    if (!duplicate) {
      state_.header = telemetry_vector_.header;
      state_.states = msg.states;
      pub_state_.publish(state_);
    }
    // Publish telemetry
    pub_telemetry_.publish(telemetry_vector_);
    ros::spinOnce();
  }

  // Get the status of the PMC
  bool GetStatus(int idx, ff_hw_msgs::PmcStatus *telemetry) {
    Telemetry t;

    if (!(pmcs_.at(idx)->GetTelemetry(&t)))
      return false;

    telemetry->motor_speed = t.motor_speed;
    telemetry->motor_current = t.motor_current;
    telemetry->v6_current = t.v6_current;
    telemetry->pressure = t.pressure;
    for (uint32_t i = 0; i < kTemperatureSensorsCount; i++) {
      telemetry->temperatures.push_back(t.temperatures[i]);
    }
    telemetry->status_1 = t.status_1.asUint8;
    telemetry->status_2 = t.status_2.asUint8;
    telemetry->command_id = t.command_id;

    return true;
  }

  // Called when a new command is received
  void CommandCallback(const ff_hw_msgs::PmcCommand::ConstPtr &msg) {
    // Immediately reset the watchdog timer
    timer_.stop();
    timer_.start();
    // Do a sanity check on the message
    uint32_t msg_goals_size = msg->goals.size();
    if (msg_goals_size < num_pmcs_) {
      ROS_ERROR_STREAM("Received a command with insufficient number of goals ("
        << msg_goals_size << ") vs. (" << num_pmcs_ << ")");
      return;
    }
    // Construct the command
    unsigned int num_nozzles = (unsigned int)null_nozzle_positions_.size();
    for (uint32_t i = 0; i < num_pmcs_; i++) {
      Command *cmd = commands_[i];
      // increment the command id since we received a new message
      cmd->command_id = cur_command_id_++;
      if (msg->goals[i].nozzle_positions.size() != num_nozzles) {
        ROS_ERROR_STREAM("Received a command with the wrong number of nozzles for PMC #" << i);
        return;
      }
      // General rule -- we are always in NOMINAL mode unless explicitly shut
      // down. If the motor speed is zeros then we automatically close nozzles.
      cmd->motor_speed = msg->goals[i].motor_speed;
      cmd->mode = (pmc_enabled_ ? CmdMode::NORMAL : CmdMode::SHUTDOWN);
      for (uint32_t n = 0; n < num_nozzles; n++)
        cmd->nozzle_positions[n] =
          Trim(i, n, (pmc_enabled_ ? msg->goals[i].nozzle_positions[n] : 0));
    }
    // Send the commands and publish telemetry
    SendAndPublish();
  }

  // If this is *ever* called, it means a FAM command was not received - note
  // that we are using a single-threaded spinner so at most one callback is
  // processed at any time which means that we wont have race conditions.
  void TimerCallback(const ros::TimerEvent&) {
    // set the blower speed to zero in case we do not receive messages from FAM
    for (uint32_t i = 0; i < num_pmcs_; i++) {
      Command *cmd = commands_.at(i);    // Get the command to send to the PMC
      cmd->motor_speed = null_fan_speed_;  // set initial fan speed
      cmd->mode = CmdMode::SHUTDOWN;          // set mode to shutdown
      // close all nozzles (should be done at the firmware level!
      for (uint32_t n = 0; n < null_nozzle_positions_.size(); n++)
        cmd->nozzle_positions[n] = null_nozzle_positions_[n];
    }
    // Send the commands to the PMCs
    SendAndPublish();
  }

  // Enable or disable the PMC
  bool EnableService(ff_msgs::SetBool::Request &req,
                     ff_msgs::SetBool::Response &res) {  // NOLINT
    pmc_enabled_ = req.enable;
    if (pmc_enabled_) {
      ROS_INFO("PMC Enabled.");
    } else {
      ROS_INFO("PMC Disabled.");
    }
    res.success = true;
    return true;
  }

  // Update Minimum Control Frequency (and cutoff time)
  bool MinRateService(ff_msgs::SetFloat::Request &req,
                      ff_msgs::SetFloat::Response &res) {  // NOLINT
    double new_min_rate = req.data;
    // Check if the new rate is within the safe and default limits
    if (new_min_rate > control_min_rate_hz_ && new_min_rate <= control_rate_hz_) {
      watchdog_period_ = ros::Duration(20.0 / new_min_rate);  // TODO(@bcoltin): why 20.0 ?
      timer_.setPeriod(watchdog_period_);
      ROS_INFO("New minimum rate UPDATED.");
      res.success = true;
    } else {
      ROS_INFO("Minimum rate not within the safe and default bounds.");
      res.success = false;
    }
    return true;
  }

 private:
  config_reader::ConfigReader config_params;       // LUA configuration reader
  ros::Subscriber sub_command_;                    // Command  flight mode sub
  ros::Publisher pub_telemetry_, pub_state_;       // Telemetry publisher
  ros::ServiceServer srv_;                         // Enable / disable service
  ros::ServiceServer update_rate_srv_;             // Update minimum control rate service
  ros::Timer timer_;                               // Watchdog timer
  ros::Duration watchdog_period_;                  // Watchdog period
  ff_hw_msgs::PmcTelemetry telemetry_vector_;      // Telemetry message
  uint32_t num_pmcs_;                              // Number of PMCs to control
  int sub_queue_size_;                             // Subscriber queue size
  int pub_queue_size_;                             // Publisher queue size
  std::string frame_id_;                           // Frame ID
  std::string i2c_bus_file_;                       // i2c bus for all PMCss
  std::vector<int> i2c_addrs_;                     // 7-bit I2C addresses
  int i2c_retries_;                                // Number of I2C bus retries
  double control_rate_hz_;                         // Control rate in Hz.
  double control_min_rate_hz_;                     // Control minimum allowed rate.
  int null_fan_speed_;                             // Initial fan speed.
  std::vector<int> null_nozzle_positions_;         // Initial nozzle positions
  uint8_t trims_[NUM_PMC][NUM_TRIM][NUM_NOZZLE];   // Trims for each nozzle
  std::vector<PmcActuator *> pmcs_;                // List of PMCs
  std::vector<Command *> commands_;                // List of commands
  double state_command_scale_;                     // Command scale
  double state_telemetry_scale_;                   // Telemetry scale
  double arrive_tol_rads_per_sec_;                 // Goal arrival threshold
  double depart_tol_rads_per_sec_;                 // Goal departure threshold
  ff_hw_msgs::PmcState state_;                     // State of the PMCs
  bool pmc_enabled_;                               // Is the PMC enabled?
  int cur_command_id_;                             // Current command ID
};

PLUGINLIB_EXPORT_CLASS(pmc_actuator::PmcActuatorNodelet, nodelet::Nodelet);

}  // namespace pmc_actuator
