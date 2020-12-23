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


#include <jsonloader/command.h>
#include <jsonloader/command_repo.h>
#include <jsonloader/validation.h>
#include <jsonloader/insensitive_map.h>

#include <glog/logging.h>

#include <json/json.h>

#include <string>

namespace jsonloader {

namespace internal {

// Registry of all known commands and their types
extern const InsensitiveMap<CommandCreateFn> kCommandMap = {
  { kCmdDock,        &CreateCommand<DockCommand> },
  { kCmdUndock,      &CreateCommand<UndockCommand> },
  { kCmdPerch,       &CreateCommand<PerchCommand> },
  { kCmdUnperch,     &CreateCommand<UnperchCommand> },
  { kCmdPause,       &CreateCommand<PauseCommand> },
  { kCmdArmPanTilt,  &CreateCommand<ArmPanAndTiltCommand> },
  { kCmdStationKeep, &CreateCommand<StationKeepCommand> },
  { kCmdWait,        &CreateCommand<StationKeepCommand> },
  { kCmdPayloadOn,   &CreateCommand<PowerItemCommand> },
  { kCmdPowerOn,     &CreateCommand<PowerItemCommand> },
  { kCmdPayloadOff,  &CreateCommand<PowerItemCommand> },
  { kCmdPowerOff,    &CreateCommand<PowerItemCommand> },
  { kCmdGripper,     &CreateCommand<GripperCommand> },
  { kCmdClearData,   &CreateCommand<DataCommand> },
  { kCmdDownload,    &CreateCommand<DataCommand> },
  { kCmdStartGuest,  &CreateCommand<GuestScienceCommand> },
  { kCmdCustomGuest, &CreateCommand<CustomGuestScienceCommand> },
  { kCmdGuestCmd,    &CreateCommand<CustomGuestScienceCommand> },
  { kCmdStopGuest,   &CreateCommand<GuestScienceCommand> },
  { kCmdFlashlight,  &CreateCommand<FlashlightCommand> },
  { kCmdGenericCmd,  &CreateCommand<GenericCommand> },
  { kCmdSetCamera,   &CreateCommand<SetCameraCommand> },
  { kCmdStreamCamera, &CreateCommand<StreamCameraCommand> },
  { kCmdRecordCamera, &CreateCommand<RecordCameraCommand> },
  { kCmdIdleProp,    &CreateCommand<IdlePropulsionCommand> },

  { kCmdInitBias,    &CreateCommand<InitializeBiasCommand> },
  { kCmdChkObstacles, &CreateCommand<SetCheckObstaclesCommand> },
  { kCmdChkZones,    &CreateCommand<SetCheckZonesCommand> },
  { kCmdSetHolonomic, &CreateCommand<SetHolonomicModeCommand> },
  { kCmdSetPlanner,  &CreateCommand<SetPlannerCommand> },
  { kCmdTelemRate,   &CreateCommand<SetTelemetryRateCommand> },
  { kCmdSwitchLocal, &CreateCommand<SwitchLocalizationCommand> },
  { kCmdStartRecord, &CreateCommand<StartRecordingCommand> },
  { kCmdStopRecord,  &CreateCommand<StopRecordingCommand> },
};

// Command names that need the normalized
extern const InsensitiveMap<std::string> kNormalizedNames = {
  { kCmdStationKeep, kCmdWait },
  { kCmdPayloadOn,   kCmdPowerOn },
  { kCmdPayloadOff,  kCmdPowerOff },
  { kCmdGuestCmd,    kCmdCustomGuest },
};

}  // namespace internal

namespace {

const Fields dockFields {
  new Field("berth", Json::intValue),
};

const Fields stationKeepFields {
  new Field("duration", Json::realValue),
};

const Fields gripperFields {
  new Field("open", Json::booleanValue),
};

const Fields armPanAndTiltFields {
  new Field("pan", Json::realValue),
  new Field("tilt", Json::realValue),
  new Field("which", Json::stringValue)
};

const Fields powerFields {
  new Field("which", Json::stringValue)
};

const Fields dataFields {
  new Field("dataMethod", Json::stringValue)
};

const Fields guestScienceFields {
  new Field("apkName", Json::stringValue),
};

const Fields guestCommandFields {
  new Field("apkName", Json::stringValue),
  new Field("command", Json::stringValue)
};

const Fields flashlightFields {
  new Field("which", Json::stringValue),
  new Field("brightness", Json::realValue)
};

const Fields genericCommandFields {
  new Field("commandName", Json::stringValue),
  new Field("param", Json::stringValue)
};

const Fields cameraFields {
  new EnumField("cameraName", {
    "Science",
    "Navigation",
    "Hazard",
    "Dock",
    "Perch"
  })
};

const Fields setCameraFields {
  new EnumField("cameraMode", {
    "Both",
    "Streaming",
    "Recording"
  }),
  new Field("resolution", Json::stringValue),
  new Field("frameRate", Json::realValue),
  new Field("bandwidth", Json::realValue)
};

const Fields streamCameraFields {
  new Field("stream", Json::booleanValue)
};

const Fields recordCameraFields {
  new Field("record", Json::booleanValue)
};

const Fields setCheckObstactlesFields {
  new Field("checkObstacles", Json::booleanValue)
};

const Fields setCheckZonesFields {
  new Field("checkZones", Json::booleanValue)
};

const Fields enableHolonomicFields {
  new Field("enableHolonomic", Json::booleanValue)
};

const Fields plannerFields {
  new EnumField("planner", {
    "trapezoidal",
    "qp"
  })
};

const Fields telemetryRateFields {
  new EnumField("telemetryName", {
    "CommStatus",
    "CpuState",
    "DiskState",
    "EkfState",
    "GncState",
    "PmcCmdState",
    "Position"
  }),
  new Field("rate", Json::realValue)
};

const Fields switchLocalizationFields {
  new EnumField("mode", {
    "MappedLandmarks",
    "ARTags",
    "Handrail",
    "Perch",
    "Truth"
  }),
};

const Fields recordingFields {
  new Field("description", Json::stringValue)
};

}  // end namespace

// TODO(tfmorse): parse Berth Number out of command when it is
// eventually added. For now, hardcoding it.
DockCommand::DockCommand(Json::Value const& obj)
  : Command(obj), berth_number_(1) {
  if (!Validate(obj, dockFields)) {
    LOG(ERROR) << "invalid Dock command.";
    return;
  }

  berth_number_ = obj["berth"].asInt();

  set_valid(true);
}

int DockCommand::berth_number() const noexcept {
  return berth_number_;
}

UndockCommand::UndockCommand(Json::Value const& obj)
  : Command(obj) {
  set_valid(true);
}

PerchCommand::PerchCommand(Json::Value const& obj)
  : Command(obj) {
  set_valid(true);
}

UnperchCommand::UnperchCommand(Json::Value const& obj)
  : Command(obj) {
  set_valid(true);
}

PauseCommand::PauseCommand(Json::Value const& obj)
  : Command(obj) {
  set_valid(true);
}

ArmPanAndTiltCommand::ArmPanAndTiltCommand(Json::Value const& obj)
  : Command(obj), pan_(0.0f), tilt_(0.0f), which_("") {
  if (!Validate(obj, armPanAndTiltFields)) {
    LOG(ERROR) << "invalid ArmPanAndTilt command.";
    return;
  }

  pan_ = obj["pan"].asFloat();
  tilt_ = obj["tilt"].asFloat();
  which_ = obj["which"].asString();

  set_valid(true);
}

float ArmPanAndTiltCommand::pan() const noexcept {
  return pan_;
}

float ArmPanAndTiltCommand::tilt() const noexcept {
  return tilt_;
}

std::string const& ArmPanAndTiltCommand::which() const noexcept {
  return which_;
}

GripperCommand::GripperCommand(Json::Value const& obj)
  : Command(obj), open_(false) {
  if (!Validate(obj, gripperFields)) {
    LOG(ERROR) << "invalid Gripper command.";
    return;
  }

  open_ = obj["open"].asBool();

  set_valid(true);
}

bool GripperCommand::open() const noexcept {
  return open_;
}

StationKeepCommand::StationKeepCommand(Json::Value const& obj)
  : Command(obj), duration_(0.0f) {
  if (!Validate(obj, stationKeepFields)) {
    LOG(ERROR) << "invalid StationKeep command.";
    return;
  }

  duration_ = obj["duration"].asFloat();

  set_valid(true);
}

float StationKeepCommand::duration() const noexcept {
  return duration_;
}

PowerItemCommand::PowerItemCommand(Json::Value const& obj)
  : Command(obj), which_("") {
  if (!Validate(obj, powerFields)) {
    LOG(ERROR) << "invalid PowerItemCommand.";
    return;
  }

  which_ = obj["which"].asString();

  set_valid(true);
}

std::string const& PowerItemCommand::which() const noexcept {
  return which_;
}

DataCommand::DataCommand(Json::Value const& obj)
  : Command(obj), data_method_("") {
  set_valid(true);
  if (!Validate(obj, dataFields)) {
    LOG(ERROR) << "invalid DataCommand.";
    return;
  }

  data_method_ = obj["dataMethod"].asString();

  set_valid(true);
}

std::string const& DataCommand::data_method() const noexcept {
  return data_method_;
}

GuestScienceCommand::GuestScienceCommand(Json::Value const& obj)
  : Command(obj), apk_("") {
  if (!Validate(obj, guestScienceFields)) {
    LOG(ERROR) << "invalid GuestScienceCommand.";
    return;
  }

  apk_ = obj["apkName"].asString();

  set_valid(true);
}

std::string const& GuestScienceCommand::apk() const noexcept {
  return apk_;
}

CustomGuestScienceCommand::CustomGuestScienceCommand(Json::Value const& obj)
  : Command(obj), apk_(""), command_("") {
  if (!Validate(obj, guestCommandFields)) {
    LOG(ERROR) << "invalid CustomGuestScienceCommand.";
    return;
  }

  apk_ = obj["apkName"].asString();
  command_ = obj["command"].asString();

  set_valid(true);
}

std::string const& CustomGuestScienceCommand::apk() const noexcept {
  return apk_;
}

std::string const& CustomGuestScienceCommand::command() const noexcept {
  return command_;
}

FlashlightCommand::FlashlightCommand(Json::Value const& obj)
  : Command(obj), which_(""), brightness_(1.0f) {
  if (!Validate(obj, flashlightFields)) {
    LOG(ERROR) << "invalid FlashlightCommand.";
    return;
  }

  which_ = obj["which"].asString();
  brightness_ = obj["brightness"].asFloat();

  set_valid(true);
}

std::string const& FlashlightCommand::which() const noexcept {
  return which_;
}

float FlashlightCommand::brightness() const noexcept {
  return brightness_;
}

GenericCommand::GenericCommand(Json::Value const& obj)
  : Command(obj), name_(""), param_("") {
  if (!Validate(obj, genericCommandFields)) {
    LOG(ERROR) << "invalid GenericCommand.";
    return;
  }

  name_ = obj["commandName"].asString();
  param_ = obj["param"].asString();

  set_valid(true);
}

std::string const& GenericCommand::name() const noexcept {
  return name_;
}

std::string const& GenericCommand::param() const noexcept {
  return param_;
}

IdlePropulsionCommand::IdlePropulsionCommand(Json::Value const& obj)
  : Command(obj) {
  set_valid(true);
}

CameraCommand::CameraCommand(Json::Value const& obj)
  : Command(obj), camera_("") {
  if (!Validate(obj, cameraFields)) {
    LOG(ERROR) << "invalid CameraCommand.";
    return;
  }

  camera_ = obj["cameraName"].asString();
}

std::string const& CameraCommand::camera() const noexcept {
  return camera_;
}

SetCameraCommand::SetCameraCommand(Json::Value const& obj)
  : CameraCommand(obj), mode_(""), frame_rate_(1.0f),
    bandwidth_(1.0f), resolution_("") {
  if (!Validate(obj, setCameraFields)) {
    LOG(ERROR) << "invalid SetCameraCommand.";
    return;
  }

  mode_ = obj["cameraMode"].asString();
  frame_rate_ = obj["frameRate"].asFloat();
  bandwidth_ = obj["bandwidth"].asFloat();
  resolution_ = obj["resolution"].asString();

  set_valid(true);
}

std::string const& SetCameraCommand::mode() const noexcept {
  return mode_;
}

float SetCameraCommand::frame_rate() const noexcept {
  return frame_rate_;
}

float SetCameraCommand::bandwidth() const noexcept {
  return bandwidth_;
}

std::string const& SetCameraCommand::resolution() const noexcept {
  return resolution_;
}

RecordCameraCommand::RecordCameraCommand(Json::Value const& obj)
  : CameraCommand(obj), record_(false) {
  if (!Validate(obj, recordCameraFields)) {
    LOG(ERROR) << "invalid RecordCameraCommand.";
    return;
  }

  record_ = obj["record"].asBool();

  set_valid(true);
}

bool RecordCameraCommand::record() const noexcept {
  return record_;
}

StreamCameraCommand::StreamCameraCommand(Json::Value const& obj)
  : CameraCommand(obj), stream_(false) {
  if (!Validate(obj, streamCameraFields)) {
    LOG(ERROR) << "invalid StreamCameraCommand.";
    return;
  }

  stream_ = obj["stream"].asBool();

  set_valid(true);
}

bool StreamCameraCommand::stream() const noexcept {
  return stream_;
}

InitializeBiasCommand::InitializeBiasCommand(Json::Value const& obj)
  : Command(obj) {
  set_valid(true);
}

SetCheckObstaclesCommand::SetCheckObstaclesCommand(Json::Value const& obj)
  : Command(obj), checkObstacles_(false) {
  if (!Validate(obj, setCheckObstactlesFields)) {
    LOG(ERROR) << "invalid SetCheckObstaclesCommand.";
    return;
  }

  checkObstacles_ = obj["checkObstacles"].asBool();

  set_valid(true);
}

bool SetCheckObstaclesCommand::checkObstacles() const noexcept {
  return checkObstacles_;
}

SetCheckZonesCommand::SetCheckZonesCommand(Json::Value const& obj)
  : Command(obj), checkZones_(false) {
  if (!Validate(obj, setCheckZonesFields)) {
    LOG(ERROR) << "invalid SetCheckZonesCommand.";
    return;
  }

  checkZones_ = obj["checkZones"].asBool();

  set_valid(true);
}

bool SetCheckZonesCommand::checkZones() const noexcept {
  return checkZones_;
}

SetHolonomicModeCommand::SetHolonomicModeCommand(Json::Value const& obj)
  : Command(obj), enableHolonomic_(false) {
  if (!Validate(obj, enableHolonomicFields)) {
    LOG(ERROR) << "invalid SetEnableHolonomicCommand.";
    return;
  }

  enableHolonomic_ = obj["enableHolonomic"].asBool();

  set_valid(true);
}

bool SetHolonomicModeCommand::enableHolonomic() const noexcept {
  return enableHolonomic_;
}

SetPlannerCommand::SetPlannerCommand(Json::Value const& obj)
  : Command(obj), planner_("") {
  if (!Validate(obj, plannerFields)) {
    LOG(ERROR) << "invalid SetPlannerCommand.";
    return;
  }

  planner_ = obj["planner"].asString();

  set_valid(true);
}

std::string const& SetPlannerCommand::planner() const noexcept {
  return planner_;
}

SetTelemetryRateCommand::SetTelemetryRateCommand(Json::Value const& obj)
  : Command(obj), telemetryName_(""), rate_(0.0f) {
  if (!Validate(obj, telemetryRateFields)) {
    LOG(ERROR) << "invalid SetTelemetryRateCommand.";
    return;
  }

  telemetryName_ = obj["telemetryName"].asString();
  rate_ = obj["rate"].asFloat();

  set_valid(true);
}

std::string const& SetTelemetryRateCommand::telemetryName() const noexcept {
  return telemetryName_;
}

float SetTelemetryRateCommand::rate() const noexcept {
  return rate_;
}

SwitchLocalizationCommand::SwitchLocalizationCommand(Json::Value const& obj)
  : Command(obj), mode_("") {
  if (!Validate(obj, switchLocalizationFields)) {
    LOG(ERROR) << "invalid SwitchLocalizationCommand.";
    return;
  }

  mode_ = obj["mode"].asString();

  set_valid(true);
}

std::string const& SwitchLocalizationCommand::mode() const noexcept {
  return mode_;
}

StartRecordingCommand::StartRecordingCommand(Json::Value const& obj)
  : Command(obj), description_("") {
  if (!Validate(obj, recordingFields)) {
    LOG(ERROR) << "invalid StartRecordingCommand.";
    return;
  }

  description_ = obj["description"].asString();

  set_valid(true);
}

std::string const& StartRecordingCommand::description() const noexcept {
  return description_;
}

StopRecordingCommand::StopRecordingCommand(Json::Value const& obj)
  : Command(obj) {
  set_valid(true);
}

}  // namespace jsonloader
