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

// ROS includes
#include <ros/ros.h>

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// Basic messahes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// FSW messages
#include <ff_msgs/VisualLandmarks.h>
#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/SetBool.h>

// Config reader access
#include <config_reader/config_reader.h>

// Camera model
#include <camera/camera_model.h>
#include <camera/camera_params.h>

#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

// STL includes
#include <string>

namespace gazebo {

// This class is a plugin that produces features and registration for EKF
// to complete the control loop
class GazeboSensorPluginSparseMap : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginSparseMap() : FreeFlyerSensorPlugin(NODE_MAPPED_LANDMARKS) {}

  virtual ~GazeboSensorPluginSparseMap() {}

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh, sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast < sensors::DepthCameraSensor > (sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginDepth requires a camera sensor as a parent.\n";
      return;
    }

     // Connect to the camera update event.
    update_ = sensor_->ConnectUpdated(
      std::bind(&GazeboSensorPluginSparseMap::UpdateCallback, this));

    // Create a publisher for the registration messages
    reg_pub_ = nh->advertise < ff_msgs::CameraRegistration > (TOPIC_LOCALIZATION_ML_REGISTRATION, 1,
      boost::bind(&GazeboSensorPluginSparseMap::ToggleCallback, this),
      boost::bind(&GazeboSensorPluginSparseMap::ToggleCallback, this));

    // Create a publisher for the feature messages
    feat_pub_ = nh->advertise < ff_msgs::VisualLandmarks > (TOPIC_LOCALIZATION_ML_FEATURES, 1,
      boost::bind(&GazeboSensorPluginSparseMap::ToggleCallback, this),
      boost::bind(&GazeboSensorPluginSparseMap::ToggleCallback, this));

    // Enable mapped landmarks
    enable_srv_ = nh->advertiseService(SERVICE_LOCALIZATION_ML_ENABLE,
      &GazeboSensorPluginSparseMap::EnableService, this);

    // Build the nav cam Camera Model
    config_cam_.AddFile("cameras.config");
    if (!config_cam_.ReadFiles()) {
        ROS_ERROR("Failed to read config files.");
        return;
    }

    // Initialize lua config reader for sparse map
    config_reader::ConfigReader config_;
    config_.AddFile("simulation/sparse_map.config");
    ReadParams(&config_);

    // Initialize a timer to be used for the delay in sending feature messages
    processing_timer_ = nh->createTimer(ros::Duration(delay_),
      &GazeboSensorPluginSparseMap::SendFeatures, this, true, false);

    // variables needed for sending both feat and reg messages
    cam_id_ = 1;
    processing_ = false;
  }

  void ReadParams(config_reader::ConfigReader *config_) {
    // Read config files into lua
    if (!config_->ReadFiles()) {
      ROS_ERROR("Error loading sparse map configuration file.");
      return;
    }

    if (!config_->GetReal("delay", &delay_))
      ROS_FATAL("Could not read the delay parameter.");

    if (!config_->GetUInt("num_features", &num_features_))
      ROS_FATAL("Could not read the num_features parameter.");
  }

  // Called on simulation reset
  virtual void Reset() {
  }

  // Turn sensor on or off if either reg or features are subscribed to
  void ToggleCallback() {
    if ((reg_pub_.getNumSubscribers() > 0) || (feat_pub_.getNumSubscribers() > 0))
      sensor_->SetActive(true);
    else
      sensor_->SetActive(false);
  }

  bool EnableService(ff_msgs::SetBool::Request & req, ff_msgs::SetBool::Response & res) {
    sensor_->SetActive(false);
    return true;
  }

  // Helper function for filling depth clouds
  Eigen::Vector3d ProjectPoint(double depth, double hfov,
    size_t u, size_t rows, size_t v, size_t cols) {
    double fl = (static_cast<double>(cols) / (2.0 *tan(hfov/2.0)));
    double pAngle = 0.0, yAngle = 0.0;
    if (rows > 1)
      pAngle = atan2(static_cast<double>(u) - 0.5*static_cast<double>(rows-1), fl);
    if (cols > 1)
      yAngle = atan2(static_cast<double>(v) - 0.5*static_cast<double>(cols-1), fl);
    return Eigen::Vector3d(
     depth * tan(yAngle),
     depth * tan(pAngle),
     depth);
  }

  // called whenever there is a new depth image
  void UpdateCallback() {
    if (!sensor_->IsActive()) return;
    if (!processing_) {
      processing_ = true;
      // Immediately send a registration pulse
      SendRegistration();

      // Get a transform from the world to sensor frame
      math::Pose tf_wb = GetModel()->GetWorldPose();
      math::Pose tf_bs = sensor_->Pose();
      math::Pose tf_ws = tf_bs + tf_wb;
      Eigen::Affine3d t = (Eigen::Translation3d(tf_ws.pos.x, tf_ws.pos.y, tf_ws.pos.z)
       * Eigen::Quaterniond(tf_ws.rot.w, tf_ws.rot.x, tf_ws.rot.y, tf_ws.rot.z));

      // Get the camera parameters for a perfect undistorted camera
      camera::CameraParameters cam_params(&config_cam_, "nav_cam");
      camera::CameraModel camera(Eigen::Vector3d(0, 0, 0), Eigen::Matrix3d::Identity(), cam_params);

      // set the pose of the camera in the world frame in the feature message
      feat_msg_.pose.position.x    = tf_ws.pos.x;
      feat_msg_.pose.position.y    = tf_ws.pos.y;
      feat_msg_.pose.position.z    = tf_ws.pos.z;
      feat_msg_.pose.orientation.x = tf_ws.rot.x;
      feat_msg_.pose.orientation.y = tf_ws.rot.y;
      feat_msg_.pose.orientation.z = tf_ws.rot.z;
      feat_msg_.pose.orientation.w = tf_ws.rot.w;

      // Preallocate landmarks and message
      feat_msg_.landmarks.reserve(num_features_);
      feat_msg_.landmarks.clear();

      // Keep going until we have enough features
      static ff_msgs::VisualLandmark landmark;
      static Eigen::Vector3d pt_c, pt_w;

      size_t N = sensor_->DepthCamera()->ImageHeight() * sensor_->DepthCamera()->ImageWidth();
      for (size_t i = 0; feat_msg_.landmarks.size() < num_features_; i++) {
        // Get a random point and it's (u,v) coordinate
        size_t rnd = std::rand() % N;
        size_t u = rnd / sensor_->DepthCamera()->ImageWidth();
        size_t v = rnd % sensor_->DepthCamera()->ImageWidth();

        // Project the random point
        pt_c = ProjectPoint(
          reinterpret_cast< const float*>(sensor_->DepthCamera()->DepthData())[rnd],
          sensor_->DepthCamera()->HFOV().Radian(),
          u, sensor_->DepthCamera()->ImageHeight(),
          v, sensor_->DepthCamera()->ImageWidth());

        // Check the point is not nan
        if (std::isnan(pt_c.x() * pt_c.y() * pt_c.z())) {
          continue;
        }

        // Check the point is in the camera FOV
        if (!camera.IsInFov(pt_c.x(), pt_c.y(), pt_c.z())) {
          continue;
        }

        // Project the point into the world frame
        pt_w = t * pt_c;

        // Create the landmark message
        landmark.x = pt_w.x();
        landmark.y = pt_w.y();
        landmark.z = pt_w.z();
        landmark.u = camera.ImageCoordinates(pt_c.x(), pt_c.y(), pt_c.z())[0];
        landmark.v = camera.ImageCoordinates(pt_c.x(), pt_c.y(), pt_c.z())[1];

        // Push the landmark message on to the feature
        feat_msg_.landmarks.push_back(landmark);
      }
      // start the timer to send the feature messages
      processing_timer_.start();
    }
  }

  // sends a registration message and starts the feature timer
  void SendRegistration(void) {
    static ff_msgs::CameraRegistration reg_msg;
    reg_msg.header.stamp = ros::Time::now();
    reg_msg.header.frame_id = std::string(FRAME_NAME_WORLD);
    reg_msg.camera_id = cam_id_;
    reg_pub_.publish(reg_msg);
  }

  // sends the features when the timer runs out
  void SendFeatures(const ros::TimerEvent& event) {
    processing_ = false;
    processing_timer_.stop();
    // ROS_INFO("delay %f", ros::Time::now().toSec() - time_test_);
    feat_msg_.header.stamp = ros::Time::now();
    feat_msg_.header.frame_id = std::string(FRAME_NAME_WORLD);
    feat_msg_.camera_id = cam_id_++;
    feat_pub_.publish(feat_msg_);
  }

 private:
  ros::Publisher feat_pub_;
  ros::Publisher reg_pub_;
  ff_msgs::VisualLandmarks feat_msg_;
  event::ConnectionPtr update_;
  ros::Timer processing_timer_;
  double delay_;
  unsigned int num_features_;
  uint32_t cam_id_;
  bool processing_;
  sensors::DepthCameraSensorPtr sensor_;
  config_reader::ConfigReader config_cam_;
  ros::ServiceServer enable_srv_;
};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginSparseMap)

}   // namespace gazebo
