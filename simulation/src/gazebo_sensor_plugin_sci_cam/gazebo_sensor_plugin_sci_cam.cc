/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <ros/ros.h>
#include <config_reader/config_reader.h>
#include <astrobee_gazebo/astrobee_gazebo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/CommandConstants.h>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <string>

namespace gazebo {

class GazeboSensorPluginSciCam : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginSciCam() :
    FreeFlyerSensorPlugin("sci_cam", "sci_cam", true),
    continuousPictureTaking_(false), takeSinglePicture_(false), rate_(0.0) {}

  ~GazeboSensorPluginSciCam() {
    if (update_) {
      #if GAZEBO_MAJOR_VERSION > 7
      update_.reset();
      #else
      sensor_->DisconnectUpdated(update_);
      #endif
    }
  }

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle *nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginSciCam requires a parent camera sensor.\n";
      return;
    }

    // Check that we have a mono camera
    if (sensor_->Camera()->ImageFormat() != "L8")
      ROS_FATAL_STREAM("Camera format must be L8");

    // Set image constants
    sci_cam_image_msg_.is_bigendian = false;
    sci_cam_image_msg_.header.frame_id = GetFrame();
    sci_cam_image_msg_.encoding = sensor_msgs::image_encodings::MONO8;

    // Create subscriber to DDS commands though which the sci cam will be controlled
    dds_cmd_sub_ = nh->subscribe(TOPIC_COMMUNICATIONS_DDS_COMMAND, 10,
                                 &GazeboSensorPluginSciCam::DdsCmdCallback, this);

    // Create publishers for sci cam image, pose, and camera info
    pub_sci_cam_image_ = nh->advertise<sensor_msgs::Image>(TOPIC_HARDWARE_SCI_CAM, 2,
      boost::bind(&GazeboSensorPluginSciCam::ToggleCallback, this),
      boost::bind(&GazeboSensorPluginSciCam::ToggleCallback, this));
    pub_sci_cam_pose_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_SCI_CAM_SIM_POSE, 10);
    pub_sci_cam_info_ = nh->advertise<sensor_msgs::CameraInfo>(TOPIC_SCI_CAM_SIM_INFO, 10);

    // Read configuration
    config_reader::ConfigReader config;
    config.AddFile("simulation/simulation.config");
    if (!config.ReadFiles()) {
      ROS_FATAL("Failed to read simulation config file.");
      return;
    }
    bool dos = true;
    if (!config.GetBool("disable_cameras_on_speedup", &dos))
      ROS_FATAL("Could not read the disable_cameras_on_speedup parameter.");
    if (!config.GetReal("sci_cam_rate", &rate_))
      ROS_FATAL("Could not read the sci_cam_rate parameter.");

    bool cp;
    if (config.GetBool("sci_cam_continuous_picture_taking", &cp)) {
      continuousPictureTaking_ = cp;
    }
    config.Close();

    // If we have a sped up simulation and we need to disable the camera
    double simulation_speed = 1.0;
    if (nh->getParam("/simulation_speed", simulation_speed))
      if (simulation_speed > 1.0 && dos) rate_ = 0.0;

    // Toggle if the camera is active or not
    ToggleCallback();
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(ros::NodeHandle *nh) {
    // Connect to the camera update event.
    update_ = sensor_->ConnectUpdated(
      std::bind(&GazeboSensorPluginSciCam::ImageCallback, this));
  }

  // Turn camera on or off based on topic subscription
  void ToggleCallback() {
    if (pub_sci_cam_image_.getNumSubscribers() > 0 && rate_ > 0) {
      sensor_->SetUpdateRate(rate_);
      sensor_->SetActive(true);
    } else {
      sensor_->SetUpdateRate(0.0001);
      sensor_->SetActive(false);
    }
  }

  // Out of string: "{"name": "turnOnContinuousPictureTaking"}"
  // collect the part in the second set of quotes.
  // Some honest json parsing could be used here.
  std::string parseJsonStr(std::string const& json_str) {
    size_t start = 0;
    size_t colon_pos = json_str.find(":", start);
    if (colon_pos == std::string::npos) {
      return "";
    }
    size_t quote1_pos = json_str.find("\"", colon_pos + 1);
    if (quote1_pos == std::string::npos) {
      return "";
    }
    size_t quote2_pos = json_str.find("\"", quote1_pos + 1);

    if (quote2_pos == std::string::npos) {
      return "";
    }

    std::string parsed = json_str.substr(quote1_pos + 1, quote2_pos - quote1_pos - 1);

    return parsed;
  }

  // Called when a dds command is received. Process only guest science
  // sci cam control commands.
  void DdsCmdCallback(ff_msgs::CommandStamped const& cmd) {
    // Process only guest science commands
    if (cmd.cmd_name != ff_msgs::CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE) {
      // Only process custom sci cam commands
      return;
    }

    if (cmd.args.size() != 2) {
      // Custom sci cam commands have two arguments
      return;
    }

    if (cmd.args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
        cmd.args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ) {
      return;
    }

    std::string app_name   = cmd.args[0].s.data();

    // Process only sci cam commands
    if (app_name != "gov.nasa.arc.irg.astrobee.sci_cam_image") {
      return;
    }

    std::string json_str = cmd.args[1].s.data();
    ROS_INFO_STREAM("Received command: " << json_str);

    std::string action = parseJsonStr(json_str);
    if (action == "")
      return;

    // Record the desired intention. Use a lock.
    {
      const std::lock_guard<std::mutex> lock(sci_cam_image_lock);
      if (action == "takeSinglePicture") {
        takeSinglePicture_ = true;
        continuousPictureTaking_ = false;
      } else if (action == "turnOnContinuousPictureTaking") {
        takeSinglePicture_ = false;
        continuousPictureTaking_ = true;
      } else if (action == "turnOffContinuousPictureTaking") {
        takeSinglePicture_ = false;
        continuousPictureTaking_ = false;
      } else {
        ROS_FATAL_STREAM("Unknown sci_cam command: " << action);
      }
    }

    return;
  }

  // Called when a new image must be rendered
  void ImageCallback() {
    // Quickly record the current time and current pose before doing other computations
    ros::Time curr_time = ros::Time::now();

    // Publish the sci cam pose
    #if GAZEBO_MAJOR_VERSION > 7
    Eigen::Affine3d sensor_to_world = SensorToWorld(GetModel()->WorldPose(), sensor_->Pose());
    #else
    Eigen::Affine3d sensor_to_world = SensorToWorld(GetModel()->GetWorldPose(), sensor_->Pose());
    #endif
    sci_cam_pose_msg_.header.frame_id = GetFrame();

    sci_cam_pose_msg_.header.stamp = curr_time;  // it is very important to get the time right
    sci_cam_pose_msg_.pose.position.x = sensor_to_world.translation().x();
    sci_cam_pose_msg_.pose.position.y = sensor_to_world.translation().y();
    sci_cam_pose_msg_.pose.position.z = sensor_to_world.translation().z();
    Eigen::Quaterniond q(sensor_to_world.rotation());
    sci_cam_pose_msg_.pose.orientation.w = q.w();
    sci_cam_pose_msg_.pose.orientation.x = q.x();
    sci_cam_pose_msg_.pose.orientation.y = q.y();
    sci_cam_pose_msg_.pose.orientation.z = q.z();
    pub_sci_cam_pose_.publish(sci_cam_pose_msg_);

    // Publish the sci cam intrinsics
    sci_cam_info_msg_.header.frame_id = GetFrame();
    sci_cam_info_msg_.header.stamp = curr_time;  // it is very important to get the time right
    FillCameraInfo(sensor_->Camera(), sci_cam_info_msg_);  // fill in from the camera pointer
    pub_sci_cam_info_.publish(sci_cam_info_msg_);

    // Do not publish unless specifically told to
    if (!continuousPictureTaking_ && !takeSinglePicture_) {
      return;
    }

    // Publish the sci cam image
    // Record not the current time, but the time when the image was acquired
    sci_cam_image_msg_.header.stamp.sec = sensor_->LastMeasurementTime().sec;
    sci_cam_image_msg_.header.stamp.nsec = sensor_->LastMeasurementTime().nsec;
    sci_cam_image_msg_.height = sensor_->ImageHeight();
    sci_cam_image_msg_.width = sensor_->ImageWidth();
    sci_cam_image_msg_.step = sci_cam_image_msg_.width;
    sci_cam_image_msg_.data.resize(sci_cam_image_msg_.step * sci_cam_image_msg_.height);
    const uint8_t* data_start = reinterpret_cast<const uint8_t*>(sensor_->ImageData());
    std::copy(data_start, data_start + sci_cam_image_msg_.step * sci_cam_image_msg_.height,
              sci_cam_image_msg_.data.begin());
    pub_sci_cam_image_.publish(sci_cam_image_msg_);

    if (takeSinglePicture_) {
      // Done taking a single picture. Use a lock to change this flag.
      const std::lock_guard<std::mutex> lock(sci_cam_image_lock);
      takeSinglePicture_ = false;
    }

    return;
  }

 private:
  sensor_msgs::Image sci_cam_image_msg_;
  geometry_msgs::PoseStamped sci_cam_pose_msg_;
  sensor_msgs::CameraInfo sci_cam_info_msg_;

  bool continuousPictureTaking_;
  bool takeSinglePicture_;
  std::mutex sci_cam_image_lock;

  ros::Publisher pub_sci_cam_image_;
  ros::Publisher pub_sci_cam_pose_;
  ros::Publisher pub_sci_cam_info_;
  ros::Subscriber dds_cmd_sub_;
  std::shared_ptr<sensors::CameraSensor> sensor_;
  event::ConnectionPtr update_;
  double rate_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginSciCam)

}   // namespace gazebo
