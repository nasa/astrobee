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

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <json/json.h>

#include <string>

using jsonloader::Command;

TEST(Command, InvalidObject) {
  // Missing "blocking"
  const std::string data = u8R"({
    "type": "not_a_command"
  })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  ASSERT_EQ(Command::Make(v), nullptr);
}

TEST(Command, InvalidCommand) {
  const std::string data = u8R"({
    "type": "not_a_command",
    "blocking": false
  })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  ASSERT_EQ(Command::Make(v), nullptr);
}

TEST(Command, VaildDockingCommand) {
  const std::string data = u8R"({
    "type" : "dock",
    "berth" : 2,
    "blocking" : true,
    "color" : "#555555",
    "scopeTerminate" : true,
    "name" : "0.0 Dock",
    "id" : "ce48dbca-696f-4750-8447-997b67524152"
  })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  Command *cmd = Command::Make(v);

  ASSERT_NE(cmd, nullptr);
  ASSERT_STREQ(cmd->type().data(), jsonloader::kCmdDock);

  const jsonloader::DockCommand *d_cmd =
      dynamic_cast<const jsonloader::DockCommand *>(cmd);

  ASSERT_EQ(d_cmd->berth_number(), 2);
}

TEST(Command, VaildWaitCommand) {
  const std::string data = u8R"({
    "type": "StationKeep",
    "duration": 1.0,
    "blocking": true
  })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  Command *cmd = Command::Make(v);

  ASSERT_NE(cmd, nullptr);
  ASSERT_STREQ(cmd->type().data(), jsonloader::kCmdWait);
}

TEST(Command, VaildGripperCommand) {
  const std::string data = u8R"({
    "type" : "gripperControl",
    "open" : true,
    "blocking" : true,
    "color" : "#555555",
    "scopeTerminate" : true,
    "name" : "2.1 GripperControl",
    "id" : "9dccc38d-867b-4318-b09b-eb0527dd388e"
  })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  Command *cmd = Command::Make(v);

  ASSERT_NE(cmd, nullptr);
  ASSERT_TRUE(cmd->valid());
  ASSERT_STREQ(cmd->type().data(), jsonloader::kCmdGripper);

  const jsonloader::GripperCommand *g_cmd =
      dynamic_cast<const jsonloader::GripperCommand *>(cmd);

  ASSERT_EQ(g_cmd->open(), true);
}

TEST(Command, VaildDataCommand) {
  const std::string data = u8R"({
    "type" : "downloadData",
    "dataMethod" : "Immediate",
    "blocking" : true,
    "color" : "#555555",
    "scopeTerminate" : true,
    "name" : "3.1 DownloadData",
    "id" : "0f15c5e7-dc59-457e-9cda-62394754b9de"
  })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  Command *cmd = Command::Make(v);

  ASSERT_NE(cmd, nullptr);
  ASSERT_TRUE(cmd->valid());
  ASSERT_STREQ(cmd->type().data(), jsonloader::kCmdDownload);

  const jsonloader::DataCommand *d_cmd =
      dynamic_cast<const jsonloader::DataCommand *>(cmd);

  ASSERT_STREQ(d_cmd->data_method().data(), "Immediate");
}

TEST(Command, ValidGuestScienceCommand) {
  const std::string data = u8R"({
    "type" : "startGuestScience",
    "apkName" : "gov.nasa.arc.irg.astrobee.air_sampler",
    "blocking" : true,
    "color" : "#555555",
    "scopeTerminate" : true,
    "name" : "1.0 StartGuestScience",
    "id" : "142dd61c-44c0-4b45-811e-2859efde6fec"
  })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  Command *cmd = Command::Make(v);

  ASSERT_NE(cmd, nullptr);
  ASSERT_TRUE(cmd->valid());
  ASSERT_STREQ(cmd->type().data(), jsonloader::kCmdStartGuest);

  const jsonloader::GuestScienceCommand *gs_cmd =
      dynamic_cast<const jsonloader::GuestScienceCommand *>(cmd);

  ASSERT_STREQ(gs_cmd->apk().data(), "gov.nasa.arc.irg.astrobee.air_sampler");
}

TEST(Command, ValidGuestCustomCommand) {
  const std::string data = u8R"({
    "type" : "customGuestScience",
    "command" : "{name=Take, num=5, time_between=10}",
    "apkName" : "gov.nasa.arc.irg.astrobee.air_sampler",
    "blocking" : true,
    "color" : "#555555",
    "scopeTerminate" : true,
    "name" : "2.0 CustomGuestScience",
    "id" : "d73842b0-4150-4efd-a02e-19f202048fab"
  })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  Command *cmd = Command::Make(v);

  ASSERT_NE(cmd, nullptr);
  ASSERT_TRUE(cmd->valid());
  ASSERT_STREQ(cmd->type().data(), jsonloader::kCmdCustomGuest);

  const jsonloader::CustomGuestScienceCommand *gs_cmd =
      dynamic_cast<const jsonloader::CustomGuestScienceCommand *>(cmd);

  ASSERT_STREQ(gs_cmd->apk().data(), "gov.nasa.arc.irg.astrobee.air_sampler");
  ASSERT_STREQ(gs_cmd->command().data(), "{name=Take, num=5, time_between=10}");
}

TEST(Command, VaildGenericCommand) {
  const std::string data = u8R"({
    "type" : "genericCommand",
    "commandName" : "CommandName",
    "param" : "Parameters",
    "blocking" : true,
    "color" : "#555555",
    "scopeTerminate" : true,
    "name" : "0.0 GenericCommand",
    "id" : "1ea5674c-802e-469f-b8a7-be5b7875b71b"
  })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  Command *cmd = Command::Make(v);

  ASSERT_NE(cmd, nullptr);
  ASSERT_STREQ(cmd->type().data(), jsonloader::kCmdGenericCmd);

  const jsonloader::GenericCommand *g_cmd =
      dynamic_cast<const jsonloader::GenericCommand *>(cmd);

  ASSERT_STREQ(g_cmd->name().data(), "CommandName");
  ASSERT_STREQ(g_cmd->param().data(), "Parameters");
}

TEST(Command, VaildSetCameraCommand) {
  const std::string data = u8R"({
    "type" : "setCamera",
    "presetName" : "High Def",
    "options" : {
      "preset" : [ {
        "presetName" : "High Def",
        "resolution" : "1920_1080",
        "frameRate" : 5.0,
        "bandwidth" : 100.0
      }, {
        "presetName" : "Low Def",
        "resolution" : "640_480",
        "frameRate" : 25.0,
        "bandwidth" : 300.0
      } ],
      "cameraName" : "Navigation"
    },
    "resolution" : "1920_1080",
    "frameRate" : 5.0,
    "bandwidth" : 100.0,
    "cameraName" : "Navigation",
    "presetIndex" : "0",
    "blocking" : true,
    "color" : "#555555",
    "scopeTerminate" : true,
    "name" : "0.1 SetCamera",
    "id" : "d2ccb6d9-0944-4dfc-ae62-c5c371698d58"
  })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  Command *cmd = Command::Make(v);

  ASSERT_NE(cmd, nullptr);
  ASSERT_STREQ(cmd->type().data(), jsonloader::kCmdSetCamera);

  const jsonloader::SetCameraCommand *s_cmd =
      dynamic_cast<const jsonloader::SetCameraCommand *>(cmd);

  ASSERT_STREQ(s_cmd->camera().data(), "Navigation");
  ASSERT_STREQ(s_cmd->resolution().data(), "1920_1080");
  ASSERT_FLOAT_EQ(s_cmd->frame_rate(), 5.0f);
  ASSERT_FLOAT_EQ(s_cmd->bandwidth(), 100.0f);
}

TEST(Command, VaildStreamingCamera) {
  const std::string data = u8R"({
    "type" : "setCameraStreaming",
    "stream" : true,
    "cameraName" : "Navigation",
    "blocking" : true,
    "color" : "#555555",
    "scopeTerminate" : true,
    "name" : "0.2 RecordCamera",
    "id" : "28b8af3c-a6ca-4745-8178-9d6077508924"
  })";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  Command *cmd = Command::Make(v);

  ASSERT_NE(cmd, nullptr);
  ASSERT_STREQ(cmd->type().data(), jsonloader::kCmdStreamCamera);

  const jsonloader::StreamCameraCommand *s_cmd =
      dynamic_cast<const jsonloader::StreamCameraCommand *>(cmd);

  ASSERT_STREQ(s_cmd->camera().data(), "Navigation");
  ASSERT_EQ(s_cmd->stream(), true);
}
