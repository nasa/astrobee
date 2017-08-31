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


#include <executive/utils/sequencer/command_conversion.h>

#include <jsonloader/command_repo.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/CommandConstants.h>

#include <string>
#include <unordered_map>

namespace {

// generic "do nothing" function used for commands that have no arguments
bool GenNoop(const jsonloader::Command *plan_cmd,
             ff_msgs::CommandStamped *dds_cmd) {
  return true;
}

bool GenDock(const jsonloader::Command *plan_cmd,
             ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::DockCommand *d_cmd =
      dynamic_cast<const jsonloader::DockCommand *>(plan_cmd);
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_INT;
  arg.i = d_cmd->berth_number();
  dds_cmd->args.push_back(arg);
  return true;
}

bool GenWait(const jsonloader::Command *plan_cmd,
             ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::StationKeepCommand *sk_cmd =
      dynamic_cast<const jsonloader::StationKeepCommand *>(plan_cmd);
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  arg.f = sk_cmd->duration();
  dds_cmd->args.push_back(arg);
  return true;
}

bool GenPower(const jsonloader::Command *plan_cmd,
                ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::PowerItemCommand *p_cmd =
      dynamic_cast<const jsonloader::PowerItemCommand *>(plan_cmd);
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = p_cmd->which();
  dds_cmd->args.push_back(arg);
  return true;
}

bool GenFlashlight(const jsonloader::Command *flash_cmd,
                   ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::FlashlightCommand *f_cmd =
      dynamic_cast<const jsonloader::FlashlightCommand *>(flash_cmd);
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = f_cmd->which();
  dds_cmd->args.push_back(arg);

  ff_msgs::CommandArg bright_arg;
  bright_arg.data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  bright_arg.f = f_cmd->brightness();
  dds_cmd->args.push_back(bright_arg);

  return true;
}

bool GenArm(const jsonloader::Command *plan_cmd,
            ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::ArmPanAndTiltCommand *arm_cmd =
      dynamic_cast<const jsonloader::ArmPanAndTiltCommand *>(plan_cmd);
  ff_msgs::CommandArg pan_arg;
  pan_arg.data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  pan_arg.f = arm_cmd->pan();
  dds_cmd->args.push_back(pan_arg);

  ff_msgs::CommandArg tilt_arg;
  tilt_arg.data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  tilt_arg.f = arm_cmd->tilt();
  dds_cmd->args.push_back(tilt_arg);

  ff_msgs::CommandArg which_arg;
  which_arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  which_arg.s = arm_cmd->which();
  dds_cmd->args.push_back(which_arg);
  return true;
}

bool GenGripper(const jsonloader::Command *plan_cmd,
                ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::GripperCommand *g_cmd =
      dynamic_cast<const jsonloader::GripperCommand *>(plan_cmd);
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_BOOL;
  arg.b = g_cmd->open();
  dds_cmd->args.push_back(arg);
  return true;
}

bool GenGuestScience(const jsonloader::Command *plan_cmd,
                     ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::GuestScienceCommand *g_cmd =
      dynamic_cast<const jsonloader::GuestScienceCommand *>(plan_cmd);
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = g_cmd->apk();
  dds_cmd->args.push_back(arg);
  return true;
}

bool GenData(const jsonloader::Command *plan_cmd,
             ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::DataCommand *d_cmd =
      dynamic_cast<const jsonloader::DataCommand *>(plan_cmd);
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = d_cmd->data_method();
  dds_cmd->args.push_back(arg);
  return true;
}

bool GenCustomScience(const jsonloader::Command *plan_cmd,
                      ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::CustomGuestScienceCommand *g_cmd =
      dynamic_cast<const jsonloader::CustomGuestScienceCommand *>(plan_cmd);
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = g_cmd->apk();
  dds_cmd->args.push_back(arg);

  ff_msgs::CommandArg arg2;
  arg2.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg2.s = g_cmd->command();
  dds_cmd->args.push_back(arg2);
  return true;
}

bool GenGeneric(const jsonloader::Command *plan_cmd,
                ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::GenericCommand *g_cmd =
      dynamic_cast<const jsonloader::GenericCommand *>(plan_cmd);
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = g_cmd->name();
  dds_cmd->args.push_back(arg);

  ff_msgs::CommandArg arg2;
  arg2.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg2.s = g_cmd->param();
  dds_cmd->args.push_back(arg2);
  return true;
}

bool GenSetCamera(const jsonloader::Command *plan_cmd,
                  ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::SetCameraCommand *s_cmd =
      dynamic_cast<const jsonloader::SetCameraCommand *>(plan_cmd);
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = s_cmd->camera();
  dds_cmd->args.push_back(arg);

  ff_msgs::CommandArg arg2;
  arg2.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg2.s = s_cmd->resolution();
  dds_cmd->args.push_back(arg2);

  ff_msgs::CommandArg arg3;
  arg3.data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  arg3.f = s_cmd->frame_rate();
  dds_cmd->args.push_back(arg3);

  ff_msgs::CommandArg arg4;
  arg4.data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
  arg4.f = s_cmd->bandwidth();
  dds_cmd->args.push_back(arg4);
  return true;
}

bool GenRecordCamera(const jsonloader::Command *plan_cmd,
                     ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::RecordCameraCommand *g_cmd =
      dynamic_cast<const jsonloader::RecordCameraCommand *>(plan_cmd);
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = g_cmd->camera();
  dds_cmd->args.push_back(arg);

  ff_msgs::CommandArg arg2;
  arg2.data_type = ff_msgs::CommandArg::DATA_TYPE_BOOL;
  arg2.b = g_cmd->record();
  dds_cmd->args.push_back(arg2);
  return true;
}

bool GenStreamCamera(const jsonloader::Command *plan_cmd,
                     ff_msgs::CommandStamped *dds_cmd) {
  const jsonloader::StreamCameraCommand *g_cmd =
      dynamic_cast<const jsonloader::StreamCameraCommand *>(plan_cmd);
  ff_msgs::CommandArg arg;
  arg.data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
  arg.s = g_cmd->camera();
  dds_cmd->args.push_back(arg);

  ff_msgs::CommandArg arg2;
  arg2.data_type = ff_msgs::CommandArg::DATA_TYPE_BOOL;
  arg2.b = g_cmd->stream();
  dds_cmd->args.push_back(arg2);
  return true;
}

}  // namespace

namespace sequencer {
namespace internal {

namespace jl = jsonloader;
using cc = ff_msgs::CommandConstants;

extern const std::unordered_map<std::string, CommandInfo> kCmdGenMap = {
  { jl::kCmdDock, { cc::CMD_NAME_DOCK, cc::CMD_SUBSYS_MOBILITY, GenDock } },
  { jl::kCmdUndock, { cc::CMD_NAME_UNDOCK, cc::CMD_SUBSYS_MOBILITY, GenNoop } },
  { jl::kCmdPerch, { cc::CMD_NAME_PERCH, cc::CMD_SUBSYS_MOBILITY, GenNoop } },
  { jl::kCmdUnperch, { cc::CMD_NAME_UNPERCH, cc::CMD_SUBSYS_MOBILITY, GenNoop } },
  { jl::kCmdPause, { cc::CMD_NAME_PAUSE_PLAN, cc::CMD_SUBSYS_PLAN, GenNoop } },
  { jl::kCmdWait, { cc::CMD_NAME_WAIT, cc::CMD_SUBSYS_PLAN, GenWait } },
  { jl::kCmdPowerOn, { cc::CMD_NAME_POWER_ON_ITEM, cc::CMD_SUBSYS_POWER, GenPower } },
  { jl::kCmdPowerOff, { cc::CMD_NAME_POWER_OFF_ITEM, cc::CMD_SUBSYS_POWER, GenPower } },
  { jl::kCmdArmPanTilt, { cc::CMD_NAME_ARM_PAN_AND_TILT, cc::CMD_SUBSYS_ARM, GenArm } },
  { jl::kCmdGripper, { cc::CMD_NAME_GRIPPER_CONTROL, cc::CMD_SUBSYS_ARM, GenGripper } },
  { jl::kCmdClearData, { cc::CMD_NAME_CLEAR_DATA, cc::CMD_SUBSYS_DATA, GenData } },
  { jl::kCmdDownload, { cc::CMD_NAME_DOWNLOAD_DATA, cc::CMD_SUBSYS_DATA, GenData } },
  { jl::kCmdStartGuest,
    { cc::CMD_NAME_START_GUEST_SCIENCE, cc::CMD_SUBSYS_GUEST_SCIENCE, GenGuestScience } },
  { jl::kCmdStopGuest,
    { cc::CMD_NAME_STOP_GUEST_SCIENCE, cc::CMD_SUBSYS_GUEST_SCIENCE, GenGuestScience } },
  { jl::kCmdCustomGuest,
    { cc::CMD_NAME_CUSTOM_GUEST_SCIENCE, cc::CMD_SUBSYS_GUEST_SCIENCE, GenCustomScience } },
  { jl::kCmdFlashlight,
    { cc::CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS, cc::CMD_SUBSYS_POWER, GenFlashlight } },
  { jl::kCmdGenericCmd,
    { cc::CMD_NAME_GENERIC_COMMAND, cc::CMD_SUBSYS_SETTINGS, GenGeneric } },
  { jl::kCmdIdleProp,
    { cc::CMD_NAME_IDLE_PROPULSION, cc::CMD_SUBSYS_MOBILITY, GenNoop } },
  { jl::kCmdSetCamera,
    { cc::CMD_NAME_SET_CAMERA, cc::CMD_SUBSYS_SETTINGS, GenSetCamera } },
  { jl::kCmdRecordCamera,
    { cc::CMD_NAME_SET_CAMERA_RECORDING, cc::CMD_SUBSYS_SETTINGS, GenRecordCamera } },
  { jl::kCmdStreamCamera,
    { cc::CMD_NAME_SET_CAMERA_STREAMING, cc::CMD_SUBSYS_SETTINGS, GenStreamCamera } },
};

}  // namespace internal
}  // namespace sequencer
