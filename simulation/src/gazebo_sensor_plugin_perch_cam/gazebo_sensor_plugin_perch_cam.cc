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

// FSW includes
#include <config_reader/config_reader.h>

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// IMU Sensor message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// STL includes
#include <string>

// Normal_distribution
#include <iostream>
#include <random>

namespace gazebo {
class GazeboSensorPluginPerchCam : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginPerchCam() :
    FreeFlyerSensorPlugin("perch_cam", "perch_cam", false), rate_(0.0) {}

  ~GazeboSensorPluginPerchCam() {
    if (update_)
      camera_->DisconnectNewRGBPointCloud(update_);
  }

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle *nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginPerchCam requires a parent camera sensor.\n";
      return;
    }
    // Get a link to the depth camera
    camera_ = sensor_->DepthCamera();
    if (!camera_) {
      gzerr << "GazeboSensorPluginHazCam cant get rendering object.\n";
      return;
    }
    // Create a publisher for the point cloud
    std::string point_topic = TOPIC_HARDWARE_PICOFLEXX_PREFIX
                            + (std::string) TOPIC_HARDWARE_NAME_PERCH_CAM
                            + (std::string) TOPIC_HARDWARE_PICOFLEXX_SUFFIX;
    point_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>(point_topic, 1,
      boost::bind(&GazeboSensorPluginPerchCam::ToggleCallback, this),
      boost::bind(&GazeboSensorPluginPerchCam::ToggleCallback, this));
    // Basic header information
    point_cloud_msg_.header.frame_id = GetFrame();
    point_cloud_msg_.is_dense = true;
    point_cloud_msg_.is_bigendian = false;
    point_cloud_msg_.point_step = sizeof(float) * 4;
    // Declare the striped memory layout
    sensor_msgs::PointField field;
    field.name = "x";
    field.offset = 0 * sizeof(float);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes
    point_cloud_msg_.fields.push_back(field);
    field.name = "y";
    field.offset = 1 * sizeof(float);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes
    point_cloud_msg_.fields.push_back(field);
    field.name = "z";
    field.offset = 2 * sizeof(float);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes
    point_cloud_msg_.fields.push_back(field);

    // Read configuration
    config_reader::ConfigReader config;
    config.AddFile("simulation/simulation.config");
    if (!config.ReadFiles()) {
      ROS_FATAL("Failed to read simulation config file.");
      return;
    }
    bool dos = true;
    if (!config.GetBool("disable_cameras_on_speedup", &dos))
      ROS_FATAL("Could not read the drawing_width parameter.");
    if (!config.GetReal("perch_cam_rate", &rate_))
      ROS_FATAL("Could not read the drawing_width parameter.");
    config.Close();

    // If we have a sped up simulation and we need to disable the camera
    double simulation_speed = 1.0;
    if (nh->getParam("/simulation_speed", simulation_speed))
      if (simulation_speed > 1.0 && dos) rate_ = 0.0;
  }

  // Only send measurements when estrinsics are available
  void OnExtrinsicsReceived(ros::NodeHandle *nh) {
    // Setup the camera
    ToggleCallback();

    // Listen to the point cloud
    update_ = camera_->ConnectNewRGBPointCloud(boost::bind(
      &GazeboSensorPluginPerchCam::Callback, this, _1, _2, _3, _4, _5));
  }

  // Turn camera on or off based on topic subscription
  void ToggleCallback() {
    if (point_cloud_pub_.getNumSubscribers() > 0 && rate_ > 0) {
      sensor_->SetUpdateRate(rate_);
      sensor_->SetActive(true);
      camera_->SetClipDist(0.001, 5);
    } else {
      sensor_->SetUpdateRate(0.0001);
      sensor_->SetActive(false);
    }
  }

  // this->dataPtr->depthBuffer, width, height, 1, "FLOAT32"
  void Callback(const float *data, unsigned int width, unsigned height,
    unsigned int len, const std::string & type) {
    point_cloud_msg_.header.stamp = ros::Time::now();
    point_cloud_msg_.width = width;
    point_cloud_msg_.height = height;
    point_cloud_msg_.row_step = point_cloud_msg_.width
                              * point_cloud_msg_.point_step;
    point_cloud_msg_.data.resize(point_cloud_msg_.row_step
      * point_cloud_msg_.height);

    /* 
     * J.L Proposed change: The simulated camera is "perfect", and
     * carries no noise on depth whatsoever. Usually, one would add 
     * the noise when creating the camera model. Gazebo currently does 
     * not permit to add noise to modeled depth sensors. Hence, we 
     * propose here to add a gaussian noise to the data acquired.
     */
    float noisy_data[static_cast<int>(point_cloud_msg_.row_step)
      * static_cast<int>(height)];
    std::default_random_engine generator;
    // distribution(mean, stdev)
    if (noisy_data != NULL) {
      std::normal_distribution<double> distribution(0.0, 0.009);
      for (int i = 0; i<static_cast<int>(height); i++) {
        for (int j = 0; j<static_cast<int>(point_cloud_msg_.row_step); j++) {
          noisy_data[i * static_cast<int>(point_cloud_msg_.row_step) + j] =
          data[i * static_cast<int>(point_cloud_msg_.row_step) + j]
          + distribution(generator);
        }
      }
      std::copy(
        reinterpret_cast<const uint8_t*>(noisy_data),
        reinterpret_cast<const uint8_t*>(noisy_data) + point_cloud_msg_.row_step
        * point_cloud_msg_.height, point_cloud_msg_.data.begin());
        point_cloud_pub_.publish(point_cloud_msg_);
    }

    /*std::copy(
      reinterpret_cast<const uint8_t*>(data),
      reinterpret_cast<const uint8_t*>(data) + point_cloud_msg_.row_step
        * point_cloud_msg_.height, point_cloud_msg_.data.begin());
    point_cloud_pub_.publish(point_cloud_msg_);*/
  }

 private:
  // ROS variables
  ros::Publisher point_cloud_pub_;
  // Sensor pointer
  sensors::DepthCameraSensorPtr sensor_;
  rendering::DepthCameraPtr camera_;
  // Camera and Point Cloud messages
  sensor_msgs::Image image_msg_;
  sensor_msgs::PointCloud2 point_cloud_msg_;
  // Gazebo variables
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  event::ConnectionPtr update_;
  std::string frame_id_;
  double rate_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginPerchCam)

}   // namespace gazebo
