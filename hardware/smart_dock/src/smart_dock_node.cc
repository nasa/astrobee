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

#include "smart_dock/smart_dock_node.h"

namespace {

typedef std::vector<kn::DdsNodeParameters> NodeVector;

void SubstituteROBOT_NAME(kn::DdsEntitiesFactorySvcParameters* params) {
  {
    NodeVector::iterator first, last = params->publishers.end();
    for (first = params->publishers.begin(); first != last; ++first) {
      if (first->name == "<ROBOTNAME>") {
        first->name = Miro::RobotParameters::instance()->name.c_str();
      }
      if (first->partition == "<ROBOTNAME>") {
        first->partition = Miro::RobotParameters::instance()->name.c_str();
      }
    }
  }

  {
    NodeVector::iterator first, last = params->subscribers.end();
    for (first = params->subscribers.begin(); first != last; ++first) {
      if (first->name == "<ROBOTNAME>") {
        first->name = Miro::RobotParameters::instance()->name.c_str();
      }
      if (first->partition == "<ROBOTNAME>") {
        first->partition = Miro::RobotParameters::instance()->name.c_str();
      }
    }
  }
}

}  // namespace

namespace smart_dock {

SmartDockNode::SmartDockNode(int argc, char** argv, const i2c::Device& i2c_dev,
                             std::string const& name)
    : sd_(i2c_dev),
      ack_pub_suffix_(""),
      dock_state_pub_suffix_(""),
      sub_suffix_(""),
      dds_event_loop_(name) {
  // Make path to QOS and NDDS files
  std::string config_path = common::GetConfigDir();
  config_path += "/communications/dds/";

  /* fake miro log into thinking we have no arguments */
  int fake = 1;
  Miro::Log::init(fake, argv);
  Miro::Log::level(9);
  Miro::Configuration::init(fake, argv);
  Miro::RobotParameters* robotParams = Miro::RobotParameters::instance();
  kn::DdsEntitiesFactorySvcParameters* ddsParams =
      kn::DdsEntitiesFactorySvcParameters::instance();

  // Load default configuration
  Miro::ConfigDocument* config = Miro::Configuration::document();
  config->setSection("Robot");
  config->getParameters("Miro::RobotParameters", *robotParams);
  config->getParameters("Miro::DdsEntitiesFactorySvcParameters", *ddsParams);

  // Set our robot name
  robotParams->name = "SmartDock";
  robotParams->namingContextName = robotParams->name;
  SubstituteROBOT_NAME(ddsParams);

  // Clear config files so that dds only looks for the files we add
  ddsParams->participants[0].discoveryPeersFiles.clear();
  ddsParams->configFiles.clear();

  // Configure DDS with domain and peers
  ddsParams->participants[0].participantName = argv[0];
  ddsParams->participants[0].domainId = 37;
  ddsParams->participants[0].discoveryPeersFiles.push_back(
      (config_path + "NDDS_DISCOVERY_PEERS"));
  ddsParams->configFiles.push_back((config_path + "RAPID_QOS_PROFILES.xml"));

  // Setup the entity factory
  dds_entities_factory_.reset(new kn::DdsEntitiesFactorySvc());
  dds_entities_factory_->init(ddsParams);

  // Set up the dock state publisher and initialize the header
  dock_state_supplier_ = new DockSupplier(
      rapid::ext::astrobee::DOCK_STATE_TOPIC + dock_state_pub_suffix_, "",
      "AstrobeeDockStateProfile", "");
  rapid::RapidHelper::initHeader(dock_state_supplier_->event().hdr);

  // Set up the ack publisher and initialize the header
  ack_state_supplier_ = new AckSupplier(rapid::ACK_TOPIC + ack_pub_suffix_, "",
                                        "RapidAckProfile", "");
  rapid::RapidHelper::initHeader(ack_state_supplier_->event().hdr);

  // Set up the command subscriber
  try {
    dds_event_loop_.connect<rapid::Command>(this,
                                            rapid::COMMAND_TOPIC + sub_suffix_,
                                            "", "RapidCommandProfile", "");
  } catch (std::exception& e) {
    std::cout << "Rapid command exception: " << e.what() << std::endl;
    throw;
  } catch (...) {
    std::cout << "Rapid command exeception unknown." << std::endl;
    throw;
  }
}

SmartDockNode::~SmartDockNode() {}

void SmartDockNode::operator()(rapid::Command const* rapid_cmd) {
  std::cout << "Received command from GDS." << std::endl;
  // Get the command from the
  SmartDock::BerthCommand command = SmartDock::COMMAND_UNKNOWN;
  // Check if this is a wake safe an
  if (strcmp(rapid_cmd->cmdName, "wakeSafe") == 0) {
    command = SmartDock::COMMAND_SET_POWER_MODE_AWAKE_SAFE;
  } else if (strcmp(rapid_cmd->cmdName, "wake") == 0) {
    command = SmartDock::COMMAND_SET_POWER_MODE_AWAKE_NOMINAL;
  } else {
    std::cerr << "Command unsupported: " << rapid_cmd->cmdName << std::endl;
    return PublishCmdAck(rapid_cmd->cmdId, rapid::ACK_COMPLETED,
                         rapid::ACK_COMPLETED_BAD_SYNTAX,
                         "Command not recognised");
  }
  // Check berth argument
  if (rapid_cmd->arguments.length() != 1 ||
      rapid_cmd->arguments[0]._d != rapid::RAPID_INT) {
    std::cerr << "Wake command has invalid syntax." << std::endl;
    return PublishCmdAck(rapid_cmd->cmdId, rapid::ACK_COMPLETED,
                         rapid::ACK_COMPLETED_BAD_SYNTAX,
                         "Wake command has invalid syntax!");
  }
  // Check berth validity
  uint32_t berth = 0;
  switch (rapid_cmd->arguments[0]._u.i) {
    case 1:
      berth = (1 << SmartDock::BERTH_1);
      break;
    case 2:
      berth = (1 << SmartDock::BERTH_2);
      break;
    default:
      std::cout << "Berth name not recognized!" << std::endl;
      return PublishCmdAck(rapid_cmd->cmdId, rapid::ACK_COMPLETED,
                           rapid::ACK_COMPLETED_BAD_SYNTAX,
                           "Berth number not recognized.");
  }
  // Try and wake up a robot on the requested berth
  if (!sd_.SendBerthCommand(berth, command))
    return PublishCmdAck(rapid_cmd->cmdId, rapid::ACK_COMPLETED,
                         rapid::ACK_COMPLETED_BAD_SYNTAX,
                         "Could not wake up requested robot");
  // Success
  std::cout << "Wake command successful!" << std::endl;
  // Everything worked correctly
  return PublishCmdAck(rapid_cmd->cmdId);
}

bool SmartDockNode::AddDevice(std::string const& serial,
                              std::string const& name) {
  if (devices_.find(serial) != devices_.end()) return false;
  devices_[serial] = name;
  return true;
}

void SmartDockNode::ProcessComms() {
  dds_event_loop_.processEvents(kn::milliseconds(0));
}

void SmartDockNode::ProcessTelem() {
  std::map<SmartDock::Berth, SmartDock::BerthState> res;
  // Get information about the berths
  uint32_t mask = 0x0;
  mask |= (1 << SmartDock::BERTH_1);
  mask |= (1 << SmartDock::BERTH_2);
  if (!sd_.GetBerthStates(mask, res)) {
    std::cerr << "Failed to query one of the berth states" << std::endl;
    return;
  }
  // Do a byte by byte comparison
  bool refresh = false;
  // Look at every berth
  std::map<SmartDock::Berth, SmartDock::BerthState>::iterator it;
  for (it = res.begin(); it != res.end(); it++) {
    // Perform the conversion
    rapid::ext::astrobee::BerthState tmp;
    tmp.occupied = (it->second.dock_state == EPS::DOCK_CONNECTED);
    tmp.awake = (it->second.power_state == EPS::POWER_STATE_AWAKE_NOMINAL);
    tmp.astrobeeName = static_cast<char*>("");
    tmp.numBatteries = 0;
    tmp.maxCapacity = 0;
    tmp.currentCapacity = 0;
    if (tmp.occupied && (it->second.power_state != EPS::POWER_STATE_UNKNOWN)) {
      // Convert the 6 byte serial to an uppercase hex string. I am 100% not
      // thrilled that rapid overloads the operator= to expect a char* on the
      // right hand side. Hence, the nasty two static_cast<> expressions. If
      // the name is not found then the serial is printed.
      std::string serial = EPS::SerialToString(it->second.serial);
      if (devices_.find(serial) != devices_.end())
        tmp.astrobeeName = static_cast<char*>(&devices_[serial][0]);
      else
        tmp.astrobeeName = static_cast<char*>(&serial[0]);
      // Get information about the energy
      for (size_t b = 0; b < EPS::NUM_BATTERIES; b++) {
        if (!it->second.batteries[b].present) continue;
        tmp.numBatteries += 1;
        tmp.maxCapacity += it->second.batteries[b].full;
        tmp.currentCapacity += it->second.batteries[b].remaining;
      }
    }
    // Get the current value
    rapid::ext::astrobee::BerthState* t = nullptr;
    switch (it->first) {
      case SmartDock::BERTH_1:
        t = &dock_state_supplier_->event().berthOne;
        break;
      case SmartDock::BERTH_2:
        t = &dock_state_supplier_->event().berthTwo;
        break;
      default:
        std::cerr << "Unknown berth" << std::endl;
        continue;
    }
    // If something changes, then copy the data and refresh!
    if (memcmp(t, &tmp, sizeof(rapid::ext::astrobee::BerthState))) {
      memcpy(t, &tmp, sizeof(rapid::ext::astrobee::BerthState));
      refresh = true;
    }
  }
  // If any of the berths change, then push a state update
  if (refresh) {
    std::cout << "Sending dock state!" << std::endl;
    dock_state_supplier_->event().hdr.timeStamp = MakeTimestamp();
    dock_state_supplier_->sendEvent();
  }
}

// PROTECTED FUNCTIONS

int64_t SmartDockNode::MakeTimestamp() {
  int64_t microseconds;
  struct timeval tv;
  gettimeofday(&tv, NULL);
  microseconds = (tv.tv_sec * 1000000) + tv.tv_usec;
  return microseconds;
}

void SmartDockNode::PublishCmdAck(std::string const& cmd_id,
                                  rapid::AckStatus status,
                                  rapid::AckCompletedStatus completed_status,
                                  std::string const& message) {
  ack_state_supplier_->event().hdr.timeStamp = MakeTimestamp();
  std::strncpy(ack_state_supplier_->event().cmdId, cmd_id.data(), 64);
  ack_state_supplier_->event().status = status;
  ack_state_supplier_->event().completedStatus = completed_status;
  std::strncpy(ack_state_supplier_->event().message, message.data(), 128);
  std::cout << "Sending command ack!" << std::endl;
  ack_state_supplier_->sendEvent();
}

}  // namespace smart_dock
