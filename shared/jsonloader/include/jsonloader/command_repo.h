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


#ifndef JSONLOADER_COMMAND_REPO_H_
#define JSONLOADER_COMMAND_REPO_H_

#include <jsonloader/command.h>

#include <string>

namespace jsonloader {

// Command names
constexpr char kCmdDock[]        = "dock";
constexpr char kCmdUndock[]      = "undock";
constexpr char kCmdPerch[]       = "perch";
constexpr char kCmdUnperch[]     = "unperch";
constexpr char kCmdPause[]       = "pausePlan";
constexpr char kCmdArmPanTilt[]  = "armPanAndTilt";
constexpr char kCmdStationKeep[] = "stationKeep";
constexpr char kCmdWait[]        = "wait";
constexpr char kCmdPayloadOn[]   = "payloadOn";
constexpr char kCmdPowerOn[]     = "powerOnItem";
constexpr char kCmdPayloadOff[]  = "payloadOff";
constexpr char kCmdPowerOff[]    = "powerOffItem";
constexpr char kCmdStartGuest[]  = "startGuestScience";
constexpr char kCmdStopGuest[]   = "stopGuestScience";
constexpr char kCmdGuestCmd[]    = "guestScience";
constexpr char kCmdCustomGuest[] = "customGuestScience";
constexpr char kCmdGripper[]     = "gripperControl";
constexpr char kCmdFlashlight[]  = "setFlashlightBrightness";
constexpr char kCmdIdleProp[]    = "idlePropulsion";
constexpr char kCmdSetCamera[]   = "setCamera";
constexpr char kCmdStreamCamera[] = "setCameraStreaming";
constexpr char kCmdRecordCamera[] = "setCameraRecording";
constexpr char kCmdInitBias[]     = "initializeBias";
constexpr char kCmdChkObstacles[] = "setCheckObstacles";
constexpr char kCmdChkZones[]     = "setCheckZones";
constexpr char kCmdSetHolonomic[] = "setHolonomicMode";
constexpr char kCmdSetPlanner[]   = "setPlanner";
constexpr char kCmdTelemRate[]    = "setTelemetryRate";
constexpr char kCmdSwitchLocal[]  = "switchLocalization";
constexpr char kCmdStartRecord[]  = "startRecording";
constexpr char kCmdStopRecord[]   = "stopRecording";
constexpr char kCmdEnableAstrobeeIntercomms[] = "enableAstrobeeIntercomms";

class DockCommand : public Command {
 public:
  explicit DockCommand(Json::Value const& obj);

  int berth_number() const noexcept;

 private:
  int berth_number_;
};

class UndockCommand : public Command {
 public:
  explicit UndockCommand(Json::Value const& obj);
};

class PerchCommand : public Command {
 public:
  explicit PerchCommand(Json::Value const& obj);
};

class UnperchCommand : public Command {
 public:
  explicit UnperchCommand(Json::Value const& obj);
};

class PauseCommand : public Command {
 public:
  explicit PauseCommand(Json::Value const& obj);
};

// This is also the "Wait" command, but backwards compatibility dictates
// we keep it as it is.
class StationKeepCommand : public Command {
 public:
  explicit StationKeepCommand(Json::Value const& obj);

  float duration() const noexcept;

 private:
  float duration_;
};

class ArmPanAndTiltCommand : public Command {
 public:
  explicit ArmPanAndTiltCommand(Json::Value const& obj);

  float pan() const noexcept;
  float tilt() const noexcept;
  std::string const& which() const noexcept;

 private:
  float pan_;
  float tilt_;
  std::string which_;
};

class GripperCommand : public Command {
 public:
  explicit GripperCommand(Json::Value const& obj);

  bool open() const noexcept;

 private:
  bool open_;
};

// In plans this is called "PayloadOn"/"PayloadOff", but will be changed to
// match the RAPID command soon, which is powerOnItem/powerOffItem (for now)
class PowerItemCommand : public Command {
 public:
  explicit PowerItemCommand(Json::Value const& obj);

  std::string const& which() const noexcept;
  int id() const noexcept;

 private:
  std::string which_;
};

class GuestScienceCommand : public Command {
 public:
  explicit GuestScienceCommand(Json::Value const& obj);

  std::string const& apk() const noexcept;

 private:
  std::string apk_;
};

class CustomGuestScienceCommand : public Command {
 public:
  explicit CustomGuestScienceCommand(Json::Value const& obj);

  std::string const& apk() const noexcept;
  std::string const& command() const noexcept;

 private:
  std::string apk_;
  std::string command_;
};

class FlashlightCommand : public Command {
 public:
  explicit FlashlightCommand(Json::Value const& obj);

  std::string const& which() const noexcept;
  float brightness() const noexcept;

 private:
  std::string which_;
  float brightness_;
};

class IdlePropulsionCommand : public Command {
 public:
  explicit IdlePropulsionCommand(Json::Value const& obj);
};

class CameraCommand : public Command {
 public:
  explicit CameraCommand(Json::Value const& obj);

  std::string const& camera() const noexcept;

 private:
  std::string camera_;
};

class SetCameraCommand : public CameraCommand {
 public:
  explicit SetCameraCommand(Json::Value const& obj);

  std::string const& mode() const noexcept;
  float frame_rate() const noexcept;
  float bandwidth() const noexcept;
  std::string const& resolution() const noexcept;

 private:
  std::string mode_;
  float frame_rate_;
  float bandwidth_;
  std::string resolution_;
};

class RecordCameraCommand : public CameraCommand {
 public:
  explicit RecordCameraCommand(Json::Value const& obj);

  bool record() const noexcept;

 private:
  bool record_;
};

class StreamCameraCommand : public CameraCommand {
 public:
  explicit StreamCameraCommand(Json::Value const& obj);

  bool stream() const noexcept;

 private:
  bool stream_;
};

class InitializeBiasCommand : public Command {
 public:
  explicit InitializeBiasCommand(Json::Value const& obj);
};

class SetCheckObstaclesCommand : public Command {
 public:
  explicit SetCheckObstaclesCommand(Json::Value const& obj);

  bool checkObstacles() const noexcept;

 private:
  bool checkObstacles_;
};

class SetCheckZonesCommand : public Command {
 public:
  explicit SetCheckZonesCommand(Json::Value const& obj);

  bool checkZones() const noexcept;

 private:
  bool checkZones_;
};

class SetHolonomicModeCommand : public Command {
 public:
  explicit SetHolonomicModeCommand(Json::Value const& obj);

  bool enableHolonomic() const noexcept;

 private:
  bool enableHolonomic_;
};

class SetPlannerCommand : public Command {
 public:
  explicit SetPlannerCommand(Json::Value const& obj);

  std::string const& planner() const noexcept;

 private:
  std::string planner_;
};

class SetTelemetryRateCommand : public Command {
 public:
  explicit SetTelemetryRateCommand(Json::Value const& obj);

  std::string const& telemetryName() const noexcept;
  float rate() const noexcept;

 private:
  std::string telemetryName_;
  float rate_;
};

class SwitchLocalizationCommand : public Command {
 public:
  explicit SwitchLocalizationCommand(Json::Value const& obj);

  std::string const& mode() const noexcept;

 private:
  std::string mode_;
};

class StartRecordingCommand : public Command {
 public:
  explicit StartRecordingCommand(Json::Value const& obj);

  std::string const& description() const noexcept;

 private:
  std::string description_;
};

class StopRecordingCommand : public Command {
 public:
  explicit StopRecordingCommand(Json::Value const& obj);
};

class EnableAstrobeeIntercommsCommand : public Command {
 public:
  explicit EnableAstrobeeIntercommsCommand(Json::Value const& obj);
};

}  // end namespace jsonloader

#endif  // JSONLOADER_COMMAND_REPO_H_
