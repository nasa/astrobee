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
constexpr char kCmdClearData[]   = "clearData";
constexpr char kCmdDownload[]    = "downloadData";
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
constexpr char kCmdGenericCmd[]  = "genericCommand";
constexpr char kCmdIdleProp[]    = "idlePropulsion";
constexpr char kCmdSetCamera[]   = "setCamera";
constexpr char kCmdStreamCamera[] = "setCameraStreaming";
constexpr char kCmdRecordCamera[] = "setCameraRecording";

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

class DataCommand : public Command {
 public:
  explicit DataCommand(Json::Value const& obj);

  std::string const& data_method() const noexcept;

 private:
  std::string data_method_;
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

class GenericCommand : public Command {
 public:
  explicit GenericCommand(Json::Value const& obj);

  std::string const& name() const noexcept;
  std::string const& param() const noexcept;

 private:
  std::string name_;
  std::string param_;
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

  float frame_rate() const noexcept;
  float bandwidth() const noexcept;
  std::string const& resolution() const noexcept;

 private:
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

}  // end namespace jsonloader

#endif  // JSONLOADER_COMMAND_REPO_H_
