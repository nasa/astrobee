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


#ifndef WDOCK_WDOCK_H_
#define WDOCK_WDOCK_H_

#include <string.h>
#include <sys/time.h>

#include <iostream>
#include <memory>
#include <string>

#include "dds_msgs/AstrobeeConstants.h"

#include "ff_common/init.h"

#include "dds_msgs/DockStateSupport.h"

#include "knDds/DdsEntitiesFactory.h"
#include "knDds/DdsEntitiesFactorySvc.h"
#include "knDds/DdsEventLoop.h"
#include "knDds/DdsSupport.h"
#include "knDds/DdsTypedSupplier.h"

#include "miro/Configuration.h"
#include "miro/Robot.h"
#include "miro/Log.h"

#include "rapidDds/RapidConstants.h"
#include "rapidDds/AckSupport.h"
#include "rapidDds/CommandSupport.h"

#include "rapidUtil/RapidHelper.h"

namespace kn {
  class DdsEntitiesFactorySvc;
}  // end namespace kn

namespace w_dock {
class WDock {
 public:
  WDock(int argc, char** argv, std::string const& entity_name);
  ~WDock();

  // call back for ddsEventLoop
  void operator() (rapid::Command const* cmd);

  int64_t MakeTimestamp();

  void ProcessDdsEventLoop();

  void PublishCmdAck(std::string const& cmd_id,
                     rapid::AckStatus status = rapid::ACK_COMPLETED,
                     rapid::AckCompletedStatus completed_status =
                                                        rapid::ACK_COMPLETED_OK,
                     std::string const& message = "");

  void PublishDockState();

  // TODO(Katie) Remove the following functions once real code is in
  void SetBerthOne();
  void SetBerthTwo();

 private:
  kn::DdsEventLoop dds_event_loop_;

  using AckStateSupplier = kn::DdsTypedSupplier<rapid::Ack>;
  using AckStateSupplierPtr = std::unique_ptr<AckStateSupplier>;

  AckStateSupplierPtr ack_state_supplier_;

  using DockStateSupplier =
                        kn::DdsTypedSupplier<rapid::ext::astrobee::DockState>;
  using DockStateSupplierPtr = std::unique_ptr<DockStateSupplier>;

  DockStateSupplierPtr dock_state_supplier_;

  using CommandEchoSupplier =
                        kn::DdsTypedSupplier<rapid::Command>;
  using CommandEchoSupplierPtr = std::unique_ptr<CommandEchoSupplier>;

  CommandEchoSupplierPtr command_echo_supplier_;

  std::shared_ptr<kn::DdsEntitiesFactorySvc> dds_entities_factory_;

  std::string ack_pub_suffix_, dock_state_pub_suffix_,
      sub_suffix_, echo_suffix_;
};

}  // end namespace w_dock

#endif  // WDOCK_WDOCK_H_
