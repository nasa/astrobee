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
 * Unless requie by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include "dds_ros_bridge/ros_telemetry_rapid_telemetry.h"

namespace ff {

RosTelemetryRapidTelemetry::RosTelemetryRapidTelemetry(
                                    const std::string& subscribe_topic,
                                    const std::string& pub_topic,
                                    const ros::NodeHandle &nh,
                                    config_reader::ConfigReader& config_params,
                                    unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  config_supplier_.reset(
      new ff::RosTelemetryRapidTelemetry::ConfigSupplier(
          rapid::ext::astrobee::TELEMETRY_CONFIG_TOPIC + publish_topic_, "",
          "AstrobeeTelemetryConfigProfile", "", ""));

  state_supplier_.reset(
      new ff::RosTelemetryRapidTelemetry::StateSupplier(
          rapid::ext::astrobee::TELEMETRY_STATE_TOPIC + publish_topic_, "",
          "AstrobeeTelemetryStateProfile", "", ""));

  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosTelemetryRapidTelemetry::CameraStateCallback,
                       this);

  rapid::RapidHelper::initHeader(config_supplier_->event().hdr);
  rapid::RapidHelper::initHeader(state_supplier_->event().hdr);

  // Initialize the serial number to be 1, it will stay 1 during execution since
  // the config won't change during run time.
  // Technically this could have been set in the initHeader function but it is
  // the last argument and I didn't care to specify any of the other arguments
  config_supplier_->event().hdr.serial = 1;
  state_supplier_->event().hdr.serial = 1;

  // Initialize rates to be 0. Setters will be called at the end of the bridge's
  // initialization process.
  state_supplier_->event().commStatusRate = 0;
  state_supplier_->event().diskStateRate = 0;
  state_supplier_->event().ekfStateRate = 0;
  state_supplier_->event().gncStateRate = 0;
  state_supplier_->event().positionRate = 0;

  if (AssembleConfig(config_params)) {
    config_supplier_->sendEvent();
  }
}

void RosTelemetryRapidTelemetry::CameraStateCallback(
                            ff_msgs::CameraStatesStampedConstPtr const& state) {
  // Get config message so we know where to but the camera in the array
  rapid::ext::astrobee::TelemetryConfig &config_msg = config_supplier_->event();
  rapid::ext::astrobee::TelemetryState &state_msg = state_supplier_->event();

  int camera_index;
  for (unsigned int i = 0; i < state->states.size(); i++) {
    camera_index = -1;
    for (int j = 0; j < config_msg.cameras.length(); j++) {
      if (state->states[i].camera_name == config_msg.cameras[j].name) {
        camera_index = j;
        break;
      }
    }

    // Don't add camera to state message if it wasn't in the config message
    if (camera_index != -1) {
      // Combine resolutions to be a string so we can use our helper function to
      // get the enum value
      std::string temp_res = std::to_string(state->states[i].stream_width) + "_"
                              + std::to_string(state->states[i].stream_height);
      // State vector size was initialized in assemble config function
      state_msg.cameras[camera_index].streaming = state->states[i].streaming;
      state_msg.cameras[camera_index].recording = state->states[i].recording;
      state_msg.cameras[camera_index].resolution = ConvertResolution(temp_res);
      state_msg.cameras[camera_index].frameRate = state->states[i].stream_rate;
      state_msg.cameras[camera_index].bandwidth = state->states[i].bandwidth;
    } else {
      ROS_ERROR("DDS Bridge: Camera %s not added to telem state, not in config",
                state->states[i].camera_name.c_str());
    }
  }

  state_supplier_->sendEvent();
}

void RosTelemetryRapidTelemetry::SetCommStatusRate(float rate) {
  if (rate < 0) {
    state_supplier_->event().commStatusRate = 0;
  } else {
    state_supplier_->event().commStatusRate = rate;
  }
  state_supplier_->sendEvent();
}

void RosTelemetryRapidTelemetry::SetCpuStateRate(float rate) {
  if (rate < 0) {
    state_supplier_->event().cpuStateRate = 0;
  } else {
    state_supplier_->event().cpuStateRate = rate;
  }
  state_supplier_->sendEvent();
}

void RosTelemetryRapidTelemetry::SetDiskStateRate(float rate) {
  if (rate < 0) {
    state_supplier_->event().diskStateRate = 0;
  } else {
    state_supplier_->event().diskStateRate = rate;
  }
  state_supplier_->sendEvent();
}

void RosTelemetryRapidTelemetry::SetEkfStateRate(float rate) {
  if (rate < 0) {
    state_supplier_->event().ekfStateRate = 0;
  } else {
    state_supplier_->event().ekfStateRate = rate;
  }
  state_supplier_->sendEvent();
}

void RosTelemetryRapidTelemetry::SetGncStateRate(float rate) {
  if (rate < 0) {
    state_supplier_->event().gncStateRate = 0;
  } else {
    state_supplier_->event().gncStateRate = rate;
  }
  state_supplier_->sendEvent();
}

void RosTelemetryRapidTelemetry::SetPositionRate(float rate) {
  if (rate < 0) {
    state_supplier_->event().positionRate = 0;
  } else {
    state_supplier_->event().positionRate = rate;
  }
  state_supplier_->sendEvent();
}

bool RosTelemetryRapidTelemetry::AssembleConfig(
                                  config_reader::ConfigReader& config_params) {
  std::string temp_name, temp_resolution, temp_mode;

  rapid::ext::astrobee::TelemetryConfig &config_msg = config_supplier_->event();

  config_reader::ConfigReader::Table cameras(&config_params, "cameras");

  int num_cameras = cameras.GetSize();
  if (num_cameras > 8) {
    ROS_ERROR("DDS Bridge: Number of cameras is greater than 8.");
    num_cameras = 8;
  }

  config_msg.cameras.length(num_cameras);

  // Make state message have the same number of cameras
  state_supplier_->event().cameras.length(num_cameras);

  // Extract camera information
  for (int i = 0; i < num_cameras; i++) {
    config_reader::ConfigReader::Table camera(&cameras, (i + 1));

    // Get camera name
    if (!camera.GetStr("name", &temp_name)) {
      ROS_ERROR("DDS Bridge: name not listed for camera %i!", i);
      return false;
    }

    strncpy(config_msg.cameras[i].name, temp_name.data(), 32);
    config_msg.cameras[i].name[31] = '\0';

    config_reader::ConfigReader::Table resolutions(&camera,
                                                          "valid_resolutions");

    int num_resolutions = resolutions.GetSize();
    if (num_resolutions > 8) {
      ROS_ERROR("DDS Bridge: Number of resolutions is greater than 8 for %s!",
                                                            temp_name.c_str());
      num_resolutions = 8;
    }

    config_msg.cameras[i].availResolutions.length(num_resolutions);

    // Extract valid resolutions
    for (int j = 0; j < num_resolutions; j++) {
      if (!resolutions.GetStr((j + 1), &temp_resolution)) {
        ROS_ERROR("DDS Bridge: Resolution not listed as string for camera %s!",
                                                            temp_name.c_str());
        return false;
      }

      config_msg.cameras[i].availResolutions[j] =
                                            ConvertResolution(temp_resolution);
    }

    // Get mode
    if (!camera.GetStr("mode", &temp_mode)) {
      ROS_ERROR("DDS Bridge: Mode not listed for camera %s", temp_name.c_str());
      return false;
    }

    if (temp_mode == "FRAMES") {
      config_msg.cameras[i].mode = rapid::ext::astrobee::MODE_FRAMES;
    } else if (temp_mode == "VIDEO") {
      config_msg.cameras[i].mode = rapid::ext::astrobee::MODE_VIDEO;
    } else {
      ROS_ERROR("DDS Bridge: Camera mode %s not recognized", temp_mode.c_str());
      return false;
    }

    // Get frame rate
    unsigned int max_frame_rate;
    if (!camera.GetUInt("max_frame_rate", &max_frame_rate)) {
      ROS_ERROR("DDS Bridge: Max frame rate not specified for camera %s!",
                                                            temp_name.c_str());
      return false;
    }
    config_msg.cameras[i].maxFrameRate = max_frame_rate;
  }

  return true;
}

rapid::ext::astrobee::CameraResolution
    RosTelemetryRapidTelemetry::ConvertResolution(std::string const& resolution) {
  if (resolution == "1_1") {
    return rapid::ext::astrobee::RESOLUTION_1_1;
  } else if (resolution == "224_172") {
    return rapid::ext::astrobee::RESOLUTION_224_172;
  } else if (resolution == "320_240") {
    return rapid::ext::astrobee::RESOLUTION_320_240;
  } else if (resolution == "480_270") {
    return rapid::ext::astrobee::RESOLUTION_480_270;
  } else if (resolution == "640_480") {
    return rapid::ext::astrobee::RESOLUTION_640_480;
  } else if (resolution == "960_540") {
    return rapid::ext::astrobee::RESOLUTION_960_540;
  } else if (resolution == "1024_768") {
    return rapid::ext::astrobee::RESOLUTION_1024_768;
  } else if (resolution == "1280_720") {
    return rapid::ext::astrobee::RESOLUTION_1280_720;
  } else if (resolution == "1280_960") {
    return rapid::ext::astrobee::RESOLUTION_1280_960;
  } else if (resolution == "1920_1080") {
    return rapid::ext::astrobee::RESOLUTION_1920_1080;
  } else {
    ROS_ERROR("DDS Bridge: Resolution %s not valid. Setting to default.",
                                                          resolution.c_str());
    return rapid::ext::astrobee::RESOLUTION_1_1;
  }
}

}  // end namespace ff
