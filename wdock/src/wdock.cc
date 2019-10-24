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


#include "wdock/wdock.h"

namespace {

typedef std::vector<kn::DdsNodeParameters> NodeVector;

void SubstituteROBOT_NAME(kn::DdsEntitiesFactorySvcParameters * params) {
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

namespace w_dock {

WDock::WDock(int argc, char** argv, std::string const& entity_name) :
    dds_event_loop_(entity_name),
    ack_pub_suffix_(""),
    dock_state_pub_suffix_(""),
    sub_suffix_(""),
    echo_suffix_("-echo") {
  int fakeArgc = 1;

  // Make path to QOS and NDDS files
  std::string config_path = common::GetConfigDir();
  config_path += "/communications/dds/";

  /* fake miro log into thinking we have no arguments */
  Miro::Log::init(fakeArgc, argv);
  Miro::Log::level(9);

  /* fake miro configuration into thinking we have no arguments */
  Miro::Configuration::init(fakeArgc, argv);

  Miro::RobotParameters *robotParams = Miro::RobotParameters::instance();
  kn::DdsEntitiesFactorySvcParameters *ddsParams =
                              kn::DdsEntitiesFactorySvcParameters::instance();

  /* get the defaults for *all the things!* */
  Miro::ConfigDocument *config = Miro::Configuration::document();
  config->setSection("Robot");
  config->getParameters("Miro::RobotParameters", *robotParams);
  config->getParameters("Miro::DdsEntitiesFactorySvcParameters", *ddsParams);

  robotParams->name = "SmartDock";
  robotParams->namingContextName = robotParams->name;

  SubstituteROBOT_NAME(ddsParams);

  // Clear config files so that dds only looks for the files we add
  ddsParams->participants[0].discoveryPeersFiles.clear();
  ddsParams->configFiles.clear();

  ddsParams->participants[0].participantName = argv[0];
  ddsParams->participants[0].domainId = 37;
  ddsParams->participants[0].discoveryPeersFiles.push_back(
      (config_path + "NDDS_DISCOVERY_PEERS"));
  ddsParams->configFiles.push_back((config_path + "RAPID_QOS_PROFILES.xml"));

  dds_entities_factory_.reset(new kn::DdsEntitiesFactorySvc());
  dds_entities_factory_->init(ddsParams);

  // Set up the dock state publisher
  dock_state_supplier_.reset(new w_dock::WDock::DockStateSupplier(
    rapid::ext::astrobee::DOCK_STATE_TOPIC + dock_state_pub_suffix_,  // topic
    "",                                                               // name
    "AstrobeeDockStateProfile",                                       // profile
    ""));                                                             // library

  // Initialize rapid message
  rapid::RapidHelper::initHeader(dock_state_supplier_->event().hdr);

  dock_state_supplier_->event().berthOne.occupied = false;
  dock_state_supplier_->event().berthOne.astrobeeName = "";
  dock_state_supplier_->event().berthOne.awake = false;
  dock_state_supplier_->event().berthOne.numBatteries = 0;
  dock_state_supplier_->event().berthOne.maxCapacity = 0;
  dock_state_supplier_->event().berthOne.currentCapacity = 0;

  dock_state_supplier_->event().berthTwo.occupied = false;
  dock_state_supplier_->event().berthTwo.astrobeeName = "";
  dock_state_supplier_->event().berthTwo.awake = false;
  dock_state_supplier_->event().berthTwo.numBatteries = 0;
  dock_state_supplier_->event().berthTwo.maxCapacity = 0;
  dock_state_supplier_->event().berthTwo.currentCapacity = 0;

  PublishDockState();

  // Set up the ack publisher
  ack_state_supplier_.reset(new w_dock::WDock::AckStateSupplier(
          rapid::ACK_TOPIC + ack_pub_suffix_,   // topic
          "",                                   // name
          "RapidAckProfile",                    // profile
          ""));                                 // library

  rapid::RapidHelper::initHeader(ack_state_supplier_->event().hdr);

  // Set up the command-echo publisher
  command_echo_supplier_.reset(new w_dock::WDock::CommandEchoSupplier(
          rapid::COMMAND_TOPIC + echo_suffix_,  // topic
          "",                                   // name
          "RapidCommandProfile",                // profile
          ""));                                 // library

  rapid::RapidHelper::initHeader(command_echo_supplier_->event().hdr);

  // Set up the command subscriber
  try {
    dds_event_loop_.connect<rapid::Command>(this,
                                            rapid::COMMAND_TOPIC +
                                            sub_suffix_,            // topic
                                            "",                     // name
                                            "RapidCommandProfile",  // profile
                                            "");                    // library
  } catch (std::exception& e) {
    std::cout << "Rapid command exception: " << e.what() << std::endl;
    throw;
  } catch (...) {
    std::cout << "Rapid command exeception unknown." << std::endl;
    throw;
  }
}

WDock::~WDock() {
}

void WDock::operator() (rapid::Command const* rapid_cmd) {
  std::cout << "Received command from GDS." << std::endl;

  command_echo_supplier_->event() = *rapid_cmd;
  command_echo_supplier_->sendEvent();

  if (strcmp(rapid_cmd->cmdName, "wake") == 0) {
    std::cout << "Received command wake. Berth: " << rapid_cmd->arguments[0]._u.i << std::endl;
    // TODO(Someone) Add code to send wake command to astrobee over i2c
    if (rapid_cmd->arguments.length() == 1 &&
        rapid_cmd->arguments[0]._d == rapid::RAPID_INT) {
      if (rapid_cmd->arguments[0]._u.i == 1) {
        std::cout << "Waking berth one!" << std::endl;
        dock_state_supplier_->event().berthOne.occupied = true;
        dock_state_supplier_->event().berthOne.astrobeeName = "Honey";
        dock_state_supplier_->event().berthOne.awake = true;
        dock_state_supplier_->event().berthOne.numBatteries = 0;
        dock_state_supplier_->event().berthOne.maxCapacity = 0;
        dock_state_supplier_->event().berthOne.currentCapacity = 0;
      } else if (rapid_cmd->arguments[0]._u.i == 2) {
        std::cout << "Waking berth two!" << std::endl;
        dock_state_supplier_->event().berthTwo.occupied = true;
        dock_state_supplier_->event().berthTwo.astrobeeName = "Bumble";
        dock_state_supplier_->event().berthTwo.awake = true;
        dock_state_supplier_->event().berthTwo.numBatteries = 0;
        dock_state_supplier_->event().berthTwo.maxCapacity = 0;
        dock_state_supplier_->event().berthTwo.currentCapacity = 0;
      } else {
        std::cout << "Berth name not recognized!" << std::endl;
        PublishCmdAck(rapid_cmd->cmdId, rapid::ACK_COMPLETED,
                      rapid::ACK_COMPLETED_BAD_SYNTAX,
                      "Berth number not recognized! Needs to be 1 or 2!");
        return;
      }
      PublishDockState();
      PublishCmdAck(rapid_cmd->cmdId);
    } else {
      std::cout << "Wake command has invalid syntax." << std::endl;
      PublishCmdAck(rapid_cmd->cmdId, rapid::ACK_COMPLETED,
                    rapid::ACK_COMPLETED_BAD_SYNTAX,
                    "Wake command has invalid syntax!");
    }
  } else {
    // TODO(Katie) put cmd name in msg
    std::string msg = rapid_cmd->cmdName;
    msg += " command not recognized by the dock.";
    std::cout << msg << std::endl;
    PublishCmdAck(rapid_cmd->cmdId, rapid::ACK_COMPLETED,
                  rapid::ACK_COMPLETED_BAD_SYNTAX, msg);
  }
}

int64_t WDock::MakeTimestamp() {
  int64_t microseconds;
  struct timeval tv;

  gettimeofday(&tv, NULL);

  microseconds = (tv.tv_sec * 1000000) + tv.tv_usec;

  return microseconds;
}

void WDock::ProcessDdsEventLoop() {
  // process events as fast as possible since we are controlling the wait else
  // where
  dds_event_loop_.processEvents(kn::milliseconds(0));
}

void WDock::PublishCmdAck(std::string const& cmd_id, rapid::AckStatus status,
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

void WDock::PublishDockState() {
  dock_state_supplier_->event().hdr.timeStamp = MakeTimestamp();
  std::cout << "Sending dock state!" << std::endl;
  dock_state_supplier_->sendEvent();
}

// TODO(Katie) Remove following functions once real code is in
void WDock::SetBerthOne() {
  dock_state_supplier_->event().berthOne.occupied = true;
  dock_state_supplier_->event().berthOne.astrobeeName = "";
  dock_state_supplier_->event().berthOne.awake = false;
  dock_state_supplier_->event().berthOne.numBatteries = 4;
  dock_state_supplier_->event().berthOne.maxCapacity = 8000;
  dock_state_supplier_->event().berthOne.currentCapacity = 8000;

  PublishDockState();
}

void WDock::SetBerthTwo() {
  dock_state_supplier_->event().berthTwo.occupied = true;
  dock_state_supplier_->event().berthTwo.astrobeeName = "Bumble";
  dock_state_supplier_->event().berthTwo.awake = false;
  dock_state_supplier_->event().berthTwo.numBatteries = 2;
  dock_state_supplier_->event().berthTwo.maxCapacity = 4000;
  dock_state_supplier_->event().berthTwo.currentCapacity = 4000;

  PublishDockState();
}

}  // end namespace w_dock
