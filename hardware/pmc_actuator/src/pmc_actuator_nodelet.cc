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
#include <ff_msgs/SetBool.h>

#include <cerrno>
#include <cstring>

/**
 * \ingroup hardware
 */
namespace pmc_actuator {

class PmcActuatorNodelet : public ff_util::FreeFlyerNodelet {
 public:
  // Constructor
  PmcActuatorNodelet() : ff_util::FreeFlyerNodelet(NODE_PMC_ACTUATOR) {}

  // Destructor - clean up the dynamically allocated PMC and command arrays
  virtual ~PmcActuatorNodelet() {
    std::vector < PmcActuator* > ::iterator it_pmc;
    for (it_pmc = pmcs_.begin(); it_pmc != pmcs_.end(); it_pmc++)
      delete *it_pmc;
    std::vector < Command* > ::iterator it_cmd;
    for (it_cmd = commands_.begin(); it_cmd != commands_.end(); it_cmd++)
      delete *it_cmd;
  }

 protected:
  virtual void Initialize(ros::NodeHandle *nh) {
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
    sub_ = nh->subscribe(TOPIC_HARDWARE_PMC_COMMAND, sub_queue_size_,
      &PmcActuatorNodelet::CommandCallback, this, ros::TransportHints().tcpNoDelay());

    // Telemetry publisher
    pub_ = nh->advertise < ff_hw_msgs::PmcTelemetry > (TOPIC_HARDWARE_PMC_TELEMETRY, pub_queue_size_);

    // PMC enable/disable service
    srv_ = nh->advertiseService(SERVICE_HARDWARE_PMC_ENABLE, &PmcActuatorNodelet::EnableService, this);

    // Watchdog timer
    timer_ = nh->createTimer(watchdog_period_, &PmcActuatorNodelet::TimerCallback, this, false, true);
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
    if (!config_params.GetInt("num_pmcs", &num_pmcs_)) {
      ROS_FATAL("PMC Actuator: number of pmcs not specified!");
      return false;
    }

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
    config_reader::ConfigReader::Table i2c_addresses(&config_params, "i2c_addrs");
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

    // get initial fan speed
    if (!config_params.GetInt("initial_speed", &initial_fan_speed_)) {
      ROS_FATAL("PMC Actuator: initial fan speed not specified!");
      return false;
    }

    // get intial nozzle positions
    config_reader::ConfigReader::Table positions(&config_params,
                                                 "initial_nozzle_positions");
    int temp_pos;
    // Lua table indices start at 1
    for (int i = 1; i < (positions.GetSize() + 1); i++) {
      if (!positions.GetInt(i, &temp_pos)) {
        ROS_FATAL("PMC Actuator: initial nozzle position not an integer!");
        return false;
      }
      initial_nozzle_positions_.push_back(temp_pos);
    }

    if (num_pmcs_ < 0 || i2c_addrs_.size() != (unsigned int)num_pmcs_) {
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

    for (int i = 0; i < num_pmcs_; i++) {
      pmcs_.push_back(new PmcActuator(bus->DeviceAt(i2c_addrs_.at(i))));
    }

    // Set initial commands.
    for (int i = 0; i < num_pmcs_; i++) {
      commands_.push_back(new Command);

      commands_.at(i)->motor_speed = initial_fan_speed_;

      commands_.at(i)->nozzle_positions[0] = initial_nozzle_positions_[0];
      commands_.at(i)->nozzle_positions[1] = initial_nozzle_positions_[1];
      commands_.at(i)->nozzle_positions[2] = initial_nozzle_positions_[2];
      commands_.at(i)->nozzle_positions[3] = initial_nozzle_positions_[3];
      commands_.at(i)->nozzle_positions[4] = initial_nozzle_positions_[4];
      commands_.at(i)->nozzle_positions[5] = initial_nozzle_positions_[5];

      commands_.at(i)->mode = CmdMode::NORMAL;

      // FIXME: How to set command_id?
      commands_.at(i)->command_id = cur_command_id_++;

      // Send a restart to the PMCs
      pmcs_.at(i)->SendCommand(*(commands_.at(i)));
    }

    return true;
  }

  // Safely exit
  void Exit(int status) {
    exit(status);
  }

  // Send the commands to the PMCs
  void SendAndPublish() {
    // Iterate over the number of PMCs
    for (int i = 0; i < num_pmcs_; i++)
      pmcs_.at(i)->SendCommand(*(commands_.at(i)));  // FIXME: lock required?
    // Send the telemetry
    telemetry_vector_.header.seq++;
    telemetry_vector_.header.frame_id = frame_id_;
    telemetry_vector_.header.stamp = ros::Time::now();
    telemetry_vector_.statuses.clear();
    for (int i = 0; i < num_pmcs_; i++) {
      ff_hw_msgs::PmcStatus t;
      if (!GetStatus(i, &t)) {
        ROS_WARN("Unable to get telemetry from PMC[%d]", i);
        // Exit(EXIT_FAILURE);
      }
      telemetry_vector_.statuses.push_back(t);
    }
    pub_.publish(telemetry_vector_);
    // Flush the event queue to ensure the messages are sent immediately
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
    for (size_t i = 0; i < kTemperatureSensorsCount; i++) {
      telemetry->temperatures.push_back(t.temperatures[i]);
    }
    telemetry->status_1 = t.status_1;
    telemetry->status_2 = t.status_2;
    telemetry->command_id = t.command_id;

    return true;
  }

  // Called when a new command is received
  void CommandCallback(const ff_hw_msgs::PmcCommand::ConstPtr &msg) {
    // Immediately reset the watchdog timer
    timer_.stop();
    timer_.start();
    // Do a sanity check on the message
    unsigned int msg_states_size = msg->states.size();
    if (msg_states_size < (unsigned int)num_pmcs_) {
      ROS_ERROR(
          "Received a PmcCommand with insufficient number of states (%u vs. %d)",
          msg_states_size, num_pmcs_);
      return;
    }
    // Construct the command
    unsigned int num_nozzles = (unsigned int)initial_nozzle_positions_.size();
    for (int i = 0; i < num_pmcs_; i++) {
      Command *cmd = commands_[i];
      // increment the command id since we received a new message
      cmd->command_id = cur_command_id_++;
      if (msg->states[i].nozzle_positions.size() != num_nozzles) {
        ROS_ERROR(
            "Received a PmcCommand with the wrong number of nozzles for PMC #%d",
            i);
        return;
      }
      for (unsigned int n = 0; n < num_nozzles; n++) {
        cmd->nozzle_positions[n] = msg->states[i].nozzle_positions[n];
      }
      if (pmc_enabled_) {
        cmd->motor_speed = msg->states[i].motor_speed;
        cmd->mode = CmdMode::NORMAL;
      } else {
        cmd->motor_speed = 0;
        cmd->mode = CmdMode::SHUTDOWN;
        cmd->nozzle_positions[0] = initial_nozzle_positions_[0];
        cmd->nozzle_positions[1] = initial_nozzle_positions_[1];
        cmd->nozzle_positions[2] = initial_nozzle_positions_[2];
        cmd->nozzle_positions[3] = initial_nozzle_positions_[3];
        cmd->nozzle_positions[4] = initial_nozzle_positions_[4];
        cmd->nozzle_positions[5] = initial_nozzle_positions_[5];
      }
    }
    // Send the commands and publish telemetry
    SendAndPublish();
  }

  // If this is *ever* called, it means a FAM command was not received - note that we
  // are using a single-threaded spinner so at most one callback is processed at any time
  // which means that we wont have race conditions on commands_
  void TimerCallback(const ros::TimerEvent&) {
    // Immediately reset the watchdog timer
    // timer_.stop();
    // timer_.start();
    // set the blower speed to zero in case we do not receive messages from FAM
    for (int i = 0; i < num_pmcs_; i++) {
      Command *cmd = commands_.at(i);    // Get the command to send to the ith PMC
      cmd->motor_speed = initial_fan_speed_;  // set initial fan speed
      cmd->mode = CmdMode::SHUTDOWN;          // set mode to shutdown
      // close all nozzles (should be done at the firmware level!
      commands_.at(i)->nozzle_positions[0] = initial_nozzle_positions_[0];
      commands_.at(i)->nozzle_positions[1] = initial_nozzle_positions_[1];
      commands_.at(i)->nozzle_positions[2] = initial_nozzle_positions_[2];
      commands_.at(i)->nozzle_positions[3] = initial_nozzle_positions_[3];
      commands_.at(i)->nozzle_positions[4] = initial_nozzle_positions_[4];
      commands_.at(i)->nozzle_positions[5] = initial_nozzle_positions_[5];
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

 private:
  config_reader::ConfigReader config_params;       // LUA configuration reader
  ros::Subscriber sub_;                            // Command subscriber
  ros::Publisher pub_;                             // Telemetry publisher
  ros::ServiceServer srv_;                         // Enable / disable service
  ros::Timer timer_;                               // Watchdog timer
  ros::Duration watchdog_period_;                  // Watchdog period
  ff_hw_msgs::PmcTelemetry telemetry_vector_;      // Telemetry message
  int num_pmcs_;                                   // Number of PMCs to control
  int sub_queue_size_;                             // Subscriber queue size
  int pub_queue_size_;                             // Publisher queue size
  std::string frame_id_;                           // Frame ID
  std::string i2c_bus_file_;                       // i2c bus -- assumes all PMC are on this bus
  std::vector<int> i2c_addrs_;                     // 7-bit I2C addresses for the PMCs.
  int i2c_retries_;                                // Number of I2C bus retries
  double control_rate_hz_;                         // Control rate in Hz.
  int initial_fan_speed_;                          // Initial fan speed.
  std::vector<int> initial_nozzle_positions_;      // Initial nozzle positions
  bool pmc_enabled_ = true;                        // Is the PMC enabled?
  int cur_command_id_ = 0;                         // Current command ID
  std::vector<PmcActuator *> pmcs_;  // List of PMCs
  std::vector<Command *> commands_;  // List of commands
};

PLUGINLIB_DECLARE_CLASS(pmc_actuator, PmcActuatorNodelet,
                        pmc_actuator::PmcActuatorNodelet, nodelet::Nodelet);

}  // namespace pmc_actuator
