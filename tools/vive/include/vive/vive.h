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

#ifndef VIVE_VIVE_H_
#define VIVE_VIVE_H_

// ROS includes
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

// ROS messages
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

// FSW tools
#include <config_reader/config_reader.h>

// FSW mesages
#include <ff_msgs/ViveCalibration.h>
#include <ff_msgs/ViveCalibrationLighthouseArray.h>
#include <ff_msgs/ViveCalibrationTrackerArray.h>
#include <ff_msgs/ViveCalibrationGeneral.h>

// Meassage convertions
#include <msg_conversions/msg_conversions.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Boost includes
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>

// STL C++ includes
#include <string>
#include <utility>
#include <vector>
#include <map>
#include <fstream>

/**
 * \ingroup tools
 */

#define TRACKER_SENSORS_NUMBER 40

enum AXIS {HORIZONTAL = 0, VERTICAL = 1};
enum LIGHTHOUSE {MASTER_LH = 0, SLAVE_LH = 1};
enum PARAMETER {POSE = 0, EXTRINSICS = 1, LIGHTHOUSE = 2, RADIUS = 3};

namespace vive {

struct Transform {
  std::string parent_frame;
  std::string child_frame;
  geometry_msgs::Vector3 translation;
  geometry_msgs::Quaternion rotation;
};

struct Sensor {
  geometry_msgs::Point position;
  geometry_msgs::Vector3 normal;
};

struct Tracker {
  std::string serial;
  std::map<uint8_t, Sensor> sensors;
  geometry_msgs::Vector3 acc_bias;
  geometry_msgs::Vector3 acc_scale;
  geometry_msgs::Vector3 gyr_bias;
  geometry_msgs::Vector3 gyr_scale;
};

struct Motor {
  double phase;
  double tilt;
  double gib_phase;
  double gib_magnitude;
  double curve;
};

struct Lighthouse {
  std::string serial;
  uint8_t id;
  Motor vertical_motor;
  Motor horizontal_motor;
};

struct LightSpecs {
  int timebase_hz;
  int timecenter_ticks;
  int pulsedist_max_ticks;
  int pulselength_min_sync;
  int pulse_in_clear_time;
  int pulse_max_for_sweep;
  int pulse_synctime_offset;
  int pulse_synctime_slack;
};

struct Environment {
  Transform vive;
  Transform offset;  // Body calibration offset
  std::map<std::string, Transform> lighthouses;  // indexed by the serial
  std::map<std::string, Transform> bodies;  // indexed by the serial
};

struct CalibrationData {
  std::map<std::string, Tracker> trackers;
  std::map<std::string, Lighthouse> lighthouses;
  LightSpecs light_specs;
  Environment environment;
};

struct Light {
  int sensor_id;
  double timecode;
  double angle;
  double length;
};

struct Imu {
  geometry_msgs::Vector3 accel;
  geometry_msgs::Vector3 gyro;
};

typedef std::vector<Imu> ImuVec;

typedef std::vector<Light> LightVec;

typedef std::pair<Eigen::Vector3d, Eigen::Matrix3d> PoseVM;
typedef std::pair<Eigen::Vector3d, Eigen::Quaterniond> PoseVQ;
typedef std::pair<Eigen::Vector3d, Eigen::AngleAxisd> PoseVA;
typedef std::pair<Eigen::Vector3d, Eigen::Vector4d> PoseVV;

class Calibration {
 public:
  // Sets the enviroment structure from a calibration message.
  bool SetEnvironment(ff_msgs::ViveCalibration const& msg);

  // Pulls the enviroment structure into a calibration message.
  bool GetEnvironment(ff_msgs::ViveCalibration * msg);

  // Sets the Light Specification stuff of the laser and flash from a
  // ViveCalibrationGeneral message.
  bool SetLightSpecs(ff_msgs::ViveCalibrationGeneral const& msg);

  // Gets the Light Specification stuff of the laser and flash and saves it
  // to the class.
  bool GetLightSpecs(ff_msgs::ViveCalibrationGeneral * msg);

  // Sets the lighthouse structure from a ViveCalibrationLighthouseArray message.
  bool SetLighthouses(ff_msgs::ViveCalibrationLighthouseArray const& msg);

  // Puts the lighthouse data into a ViveCalibrationLighthouseArray message.
  bool GetLighthouses(ff_msgs::ViveCalibrationLighthouseArray * msg);

  // Sets the tracker structure from a ViveCalibrationTrackerArray message.
  bool SetTrackers(ff_msgs::ViveCalibrationTrackerArray const& msg);

  // Puts the tracker data into a ViveCalibrationTrackerArray message.
  bool GetTrackers(ff_msgs::ViveCalibrationTrackerArray * msg);

  std::map<std::string, Tracker> trackers;
  std::map<std::string, Lighthouse> lighthouses;
  LightSpecs light_specs;
  Environment environment;
};

class ViveUtils {
 public:
  // Write to config file the calibration class
  static bool WriteConfig(std::string file_name,
    Calibration const& calibration);

  // Read from config file the calibration and write to calibration class
  static bool ReadConfig(std::string file_name,
    Calibration * calibration);

  // Read lighthouses from lighthouse table
  static bool GetLighthouses(config_reader::ConfigReader::Table lighthouses,
    Calibration * calibration);

  // Read trackers from tracker table
  static bool GetTrackers(config_reader::ConfigReader::Table trackers,
    Calibration * calibration);

  // Read bodies from bodies table
  static bool GetBodies(config_reader::ConfigReader::Table bodies,
    Calibration * calibration);

  // Read calibration details - world-vive - from table
  static bool GetCalibration(config_reader::ConfigReader::Table calibration_table,
    Calibration * calibration);

  // Broadcast all static trasnforms
  static bool SendTransforms(Calibration const& calibration_data);

  // Get double array from tracker struct - double array previously initialized
  static size_t ConvertExtrinsics(Tracker const& tracker,
    double * extrinsics);
};

}  // namespace vive

#endif  // VIVE_VIVE_H_
