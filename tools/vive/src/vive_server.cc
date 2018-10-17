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

// This package code
#include <vive/vive.h>
#include <vive/vive_solve.h>
#include <vive/vive_calibrate.h>
#include <vive/vive_visualization.h>

// Standard includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/transform_broadcaster.h>

// FSW utils
#include <ff_util/ff_fsm.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>
#include <ff_util/config_server.h>
#include <ff_util/ff_serialization.h>

// Services
#include <ff_msgs/ViveConfig.h>

// Messages
#include <ff_msgs/ViveLight.h>
#include <ff_msgs/ViveCalibration.h>
#include <ff_msgs/ViveCalibrationGeneral.h>
#include <ff_msgs/ViveCalibrationTrackerArray.h>
#include <ff_msgs/ViveCalibrationLighthouseArray.h>

// Meassage Convertions
#include <msg_conversions/msg_conversions.h>

/**
 * \ingroup tools
 */
namespace vive {

using FSM = ff_util::FSM;

// Convenience declaration
typedef std::map<std::string, ViveSolve> TrackerMap;
typedef std::map<std::string, Visualization> VisualMap;

class ServerNodelet : public ff_util::FreeFlyerNodelet {
 public:
  // All possible FSM states
  enum : FSM::State {
    TRACKING       = 1,
    CALIBRATING    = 2
  };

  // All possible FSM events
  enum : FSM::Event {
    GOAL_START     = (1<<0),
    GOAL_STOP      = (1<<1)
  };

  // Constructor : build the FSM and start by default in tracking mode
  ServerNodelet() : ff_util::FreeFlyerNodelet(NODE_VIVE_SERVER, false),
    calibrator_(std::bind(&ServerNodelet::CalibrationCallback, this, std::placeholders::_1)),
    fsm_(TRACKING, std::bind(&ServerNodelet::UpdateCallback,
      this, std::placeholders::_1, std::placeholders::_2)) {
      // Edge [0]
      fsm_.Add(TRACKING,
        GOAL_START,
        [this](FSM::Event const& event) -> FSM::State {
          calibrator_.Reset();
          calibrator_.Initialize(calibration_data_);
          return CALIBRATING;
        });
      // Edge [1]
      fsm_.Add(CALIBRATING,
        GOAL_STOP,
        [this](FSM::Event const& event) -> FSM::State {
          calibrator_.Solve();
          calibrator_.Reset();
          return TRACKING;
        });
    }

  // Destructor
  ~ServerNodelet() {}

 protected:
  void Initialize(ros::NodeHandle *nh) {
    cfg_.Initialize(GetPrivateHandle(), "tools/vive.config");
    cfg_.Listen(boost::bind(&ServerNodelet::ReconfigureCallback, this, _1));

    // Subscribers for light measurements
    sub_light_ = nh->subscribe(TOPIC_LOCALIZATION_VIVE_LIGHT, 1000,
      &ServerNodelet::LightCallback, this);
    sub_imu_ = nh->subscribe(TOPIC_LOCALIZATION_VIVE_IMU, 1000,
      &ServerNodelet::ImuCallback, this);

    // Subscribers to calibration stuff
    sub_lighthouses_ = nh->subscribe(TOPIC_LOCALIZATION_VIVE_LIGHTHOUSES, 1000,
      &ServerNodelet::LighthouseCallback, this);
    sub_trackers_ = nh->subscribe(TOPIC_LOCALIZATION_VIVE_TRACKERS, 1000,
      &ServerNodelet::TrackerCallback, this);
    sub_general_ = nh->subscribe(TOPIC_LOCALIZATION_VIVE_GENERAL, 1000,
      &ServerNodelet::LightSpecsCallback, this);

    // Visualiatization tools
    pub_imu_markers_ = nh->advertise<visualization_msgs::Marker>(
      TOPIC_LOCALIZATION_VIVE_IMU_MARKERS, 1000);
    pub_light_markers_ = nh->advertise<visualization_msgs::MarkerArray>(
      TOPIC_LOCALIZATION_VIVE_LIGHT_MARKERS, 1000);
    pub_tracker_markers_ = nh->advertise<visualization_msgs::MarkerArray>(
      TOPIC_LOCALIZATION_VIVE_TRACKER_MARKERS, 1000);

    // Allow planners to register themselves
    service_ = nh->advertiseService(
      SERVICE_LOCALIZATION_VIVE_CONFIG,
        &ServerNodelet::ConfigureCallback, this);

    // Periodic timer to query and send pose
    timer_ = nh->createTimer(ros::Rate(cfg_.Get<double>("tracking_rate")),
        &ServerNodelet::TimerCallback, this, false, true);

    // Reading the config
    config_reader::ConfigReader *handle = cfg_.GetConfigReader();
    config_reader::ConfigReader::Table tmp;  // Auxiliary pointer

    if (handle == nullptr)
      NODELET_FATAL_STREAM("Cannot read LUA config");
    config_reader::ConfigReader::Table vive_parameters;
    if (!handle->GetTable("vive_parameters", &vive_parameters))
      NODELET_FATAL_STREAM("Cannot read vive_parameters from LUA config");

    // Finding the calibration data
    if (!handle->GetStr("vive_calibration", &calib_file_))
      ROS_FATAL("Could not read the calibration file name");
    if (!ViveUtils::ReadConfig(calib_file_, &calibration_data_)) {
      // Getting the calibration data world-vive
      config_reader::ConfigReader::Table calibration;
      if (!vive_parameters.GetTable("calibration", &calibration))
        NODELET_FATAL_STREAM("Cannot read calibration in the config file");
      if (!ViveUtils::GetCalibration(calibration, &calibration_data_))
        NODELET_FATAL_STREAM("Cannot read calibration in the config file");
      // Getting the calibration data for the lighthouses
      config_reader::ConfigReader::Table lighthouses;
      if (!vive_parameters.GetTable("lighthouses", &lighthouses))
        NODELET_FATAL_STREAM("Cannot read lighthouses in the config file");
      if (!ViveUtils::GetLighthouses(lighthouses, &calibration_data_))
        NODELET_FATAL_STREAM("Cannot read lighthouses in the config file");
    }

    // Read body data from the condig file
    config_reader::ConfigReader::Table bodies;
    if (!vive_parameters.GetTable("extrinsics", &bodies))
      NODELET_WARN_STREAM("No body available in config file - Calibration won't work");
    if (bodies.GetSize() == 0)
      NODELET_WARN_STREAM("0 bodies available in config file - Calibration won't work");
    if (bodies.GetSize() >= 2)
      NODELET_WARN_STREAM("Multiple bodies available in config file - First one will be used for calibration");
    // if (!ViveUtils::GetBodies(bodies, calibration_))
    if (!ViveUtils::GetBodies(bodies, &calibration_data_))
      NODELET_WARN_STREAM("No body available in config file - Calibration won't work");

    // Publishing all the static transforms obtained from calibration
    ViveUtils::SendTransforms(calibration_data_);
  }

  // When the FSM state changes we get a callback here, so that we can debug
  void UpdateCallback(FSM::State const& state, FSM::Event const& event) {
    static std::string str;
    switch (event) {
    case GOAL_START:          str = "GOAL_START";           break;
    case GOAL_STOP:           str = "GOAL_STOP";            break;
    default:                  str = "UNKNOWN";              break;
    }
    NODELET_INFO_STREAM("Received event " << str);
    switch (state) {
    case TRACKING:            str = "TRACKING";             break;
    case CALIBRATING:           str = "CALIBRATING";            break;
    default:                  str = "UNKNOWN";              break;
    }
    NODELET_INFO_STREAM("State changed to " << str);
  }

  // Called when a new pose must be sent for each tracker
  void TimerCallback(const ros::TimerEvent&) {
    // Ignore if not in tracking mode
    if (fsm_.GetState() != TRACKING) return;
    // Iterate over all trackers that we are solving for, and send their pose
    for (TrackerMap::iterator tr_it = trackers_.begin();
      tr_it != trackers_.end(); tr_it++) {
      // Get the transform
      geometry_msgs::TransformStamped tf;
      if (tr_it->second.GetTransform(tf)) {
        static tf2_ros::TransformBroadcaster br;
        br.sendTransform(tf);
        // IMU
        visualization_msgs::Marker arrow;
        if (vive_visualization_[tr_it->first].GetImu(&arrow)) {
          pub_imu_markers_.publish(arrow);
        }
        // LIGHTS
        visualization_msgs::MarkerArray directions;
        if (vive_visualization_[tr_it->first].GetLight(&directions)) {
          pub_light_markers_.publish(directions);
        }
        // SENSORS
        visualization_msgs::MarkerArray sensors;
        if (vive_visualization_[tr_it->first].GetSensors(&sensors)) {
          pub_tracker_markers_.publish(sensors);
        }
      }
    }
    return;
  }

  // Called back when the calibration procedure completes
  void CalibrationCallback(Calibration const& calibration) {
    ViveUtils::WriteConfig(calib_file_, calibration);
    ViveUtils::SendTransforms(calibration);
    // Set solvers with the right parameters
    for (TrackerMap::iterator tr_it = trackers_.begin();
      tr_it != trackers_.end(); tr_it++) {
      tr_it->second.Update(calibration.environment);
    }
    calibration_data_ = calibration;
    return;
  }

  // Called when IMU data becomes available
  void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    switch (fsm_.GetState()) {
    case CALIBRATING:
      calibrator_.AddImu(msg);
      break;

    case TRACKING:
      trackers_[msg->header.frame_id].ProcessImu(msg);
      vive_visualization_[msg->header.frame_id].AddImu(msg);
      break;

    default:
      break;
    }
  }

  // Called when light data becomes available
  void LightCallback(const ff_msgs::ViveLight::ConstPtr& msg) {
    switch (fsm_.GetState()) {
    case CALIBRATING:
      calibrator_.AddLight(msg);
      break;

    case TRACKING:
      // Check if tracker is registred
      if (trackers_.find(msg->header.frame_id) == trackers_.end()) {
        ROS_FATAL("Can't find tracker");
        break;
      }
      trackers_[msg->header.frame_id].ProcessLight(msg);
      vive_visualization_[msg->header.frame_id].AddLight(msg);
      break;

    default:
      break;
    }
  }

  // Called when lighthouse data becomes available
  void LighthouseCallback(const ff_msgs::ViveCalibrationLighthouseArray::ConstPtr& msg) {
    calibration_data_.SetLighthouses(*msg);
  }

  // Called when tracker data becomes available
  void TrackerCallback(const ff_msgs::ViveCalibrationTrackerArray::ConstPtr& msg) {
    calibration_data_.SetTrackers(*msg);
    // Initialize each tracker's solver
    for (std::map<std::string, Tracker>::const_iterator tr_it = calibration_data_.trackers.begin();
      tr_it != calibration_data_.trackers.end(); tr_it++) {
      // Solver
      trackers_[tr_it->first].Initialize(calibration_data_.environment,
        tr_it->second);
      // Visualization tools
      vive_visualization_[tr_it->first].Initialize(tr_it->second, trackers_.size()-1);
    }
    return;
  }

  // Called when general data becomes available
  void LightSpecsCallback(const ff_msgs::ViveCalibrationGeneral::ConstPtr& msg) {
    calibration_data_.SetLightSpecs(*msg);
  }

  // When parameters to this node are changed
  bool ReconfigureCallback(dynamic_reconfigure::Config &config) {
    return cfg_.Reconfigure(config);
  }

  // Configuration call from the vive_tool
  bool ConfigureCallback(ff_msgs::ViveConfig::Request & req,
                         ff_msgs::ViveConfig::Response & res ) {
    switch (req.action) {
    case ff_msgs::ViveConfig::Request::START:
      fsm_.Update(GOAL_START);
      res.success = true;
      res.status = std::to_string(fsm_.GetState());
      break;
    case ff_msgs::ViveConfig::Request::STOP:
      fsm_.Update(GOAL_STOP);
      res.success = true;
      res.status = std::to_string(fsm_.GetState());
      break;
    default:
      return false;
    }
    return true;
  }

 public:
  // Solvers
  std::string solver_;                  // Active solver
  TrackerMap trackers_;                 // Tracker solvers

  // Calibration tools
  std::string calib_file_;              // Name of the calibration file
  ViveCalibrate calibrator_;            // Calibrater
  Calibration calibration_data_;        // Structure with all the data

  // State machine
  ff_util::FSM fsm_;                    // State machine

  // ROS stuff
  ros::Subscriber sub_imu_,
    sub_light_,
    sub_lighthouses_,
    sub_trackers_,
    sub_general_;                       // ROS subscriber
  ff_util::ConfigServer cfg_;           // Config server
  ros::ServiceServer service_;          // Service
  ros::Timer timer_;                    // Tracking timer

  // Visualization tools
  VisualMap vive_visualization_;        // visualization object
  ros::Publisher pub_imu_markers_;      // Imu visualization marker
  ros::Publisher pub_light_markers_;    // light visualization markers
  ros::Publisher pub_tracker_markers_;  // tracker visualization markers
};

}  // namespace vive

PLUGINLIB_EXPORT_CLASS(vive::ServerNodelet, nodelet::Nodelet);
