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

// IMU Sensor message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// STL includes
#include <string>

namespace gazebo {
class GazeboSensorPluginHazCam : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginHazCam() : FreeFlyerSensorPlugin(NODE_HAZ_CAM) {}

  ~GazeboSensorPluginHazCam() {}

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle *nh, sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast < sensors::DepthCameraSensor > (sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginHazCam requires a camera sensor as a parent.\n";
      return;
    }
    // Create a publisher for the point cloud
    std::string point_topic = TOPIC_HARDWARE_PICOFLEXX_PREFIX
                            + (std::string) TOPIC_HARDWARE_NAME_HAZ_CAM
                            + (std::string) TOPIC_HARDWARE_PICOFLEXX_SUFFIX;
    point_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>(point_topic, 1,
      boost::bind(&GazeboSensorPluginHazCam::ToggleCallback, this),
      boost::bind(&GazeboSensorPluginHazCam::ToggleCallback, this));
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
    field.count = sizeof(float);
    point_cloud_msg_.fields.push_back(field);
    field.name = "y";
    field.offset = 1 * sizeof(float);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = sizeof(float);
    point_cloud_msg_.fields.push_back(field);
    field.name = "z";
    field.offset = 2 * sizeof(float);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = sizeof(float);
    point_cloud_msg_.fields.push_back(field);
    // Listen to the point cloud
    update_ = sensor_->DepthCamera()->ConnectNewRGBPointCloud(
      boost::bind(&GazeboSensorPluginHazCam::Callback, this, _1, _2, _3, _4, _5));
  }

  // Turn camera on or off based on topic subscription
  void ToggleCallback() {
    if (point_cloud_pub_.getNumSubscribers() > 0 || image_pub_.getNumSubscribers() > 0)
      sensor_->SetActive(true);
    else
      sensor_->SetActive(false);
  }

  // this->dataPtr->depthBuffer, width, height, 1, "FLOAT32"
  void Callback(const float *data, unsigned int width, unsigned height, unsigned int len, const std::string & type) {
    if (!sensor_->IsActive()) return;
    point_cloud_msg_.header.stamp = ros::Time::now();
    point_cloud_msg_.width = width;
    point_cloud_msg_.height = height;
    point_cloud_msg_.row_step = point_cloud_msg_.width * point_cloud_msg_.point_step;
    point_cloud_msg_.data.resize(point_cloud_msg_.row_step * point_cloud_msg_.height);
    std::copy(
      reinterpret_cast<const uint8_t*>(data),
      reinterpret_cast<const uint8_t*>(data) + point_cloud_msg_.row_step * point_cloud_msg_.height,
      point_cloud_msg_.data.begin());
    point_cloud_pub_.publish(point_cloud_msg_);
  }

 private:
  // ROS variables
  ros::Publisher image_pub_;
  ros::Publisher point_cloud_pub_;
  // Sensor pointer
  sensors::DepthCameraSensorPtr sensor_;
  // Camera and Point Cloud messages
  sensor_msgs::Image image_msg_;
  sensor_msgs::PointCloud2 point_cloud_msg_;
  // Gazebo variables
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  event::ConnectionPtr update_;
  std::string frame_id_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginHazCam)

}   // namespace gazebo
