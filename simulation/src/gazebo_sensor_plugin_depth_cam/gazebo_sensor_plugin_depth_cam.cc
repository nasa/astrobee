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

#include <astrobee_gazebo/astrobee_gazebo.h>
#include <config_reader/config_reader.h>
#include <ff_msgs/msg/command_constants.hpp>
#include <ff_msgs/msg/command_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Geometry>
#include <Eigen/Core>

// STL includes
#include <string>

namespace gazebo {

FF_DEFINE_LOGGER("gazebo_sensor_plugin_depth_cam");

class GazeboSensorPluginDepthCam : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginDepthCam() :
    FreeFlyerSensorPlugin("", "", true), rate_(0.0), num_channels_(0) {
  }

  ~GazeboSensorPluginDepthCam() {
    if (depth_update_) {
      depth_update_.reset();
    }
    if (image_update_) {
      image_update_.reset();
    }
  }

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(NodeHandle& nh,
                    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    if (sdf->HasElement("haz_cam"))
      is_haz_cam_ = sdf->Get<bool>("haz_cam");
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);
    if (!sensor_) {
      FF_FATAL("GazeboSensorPluginDepthCam requires a depth camera sensor.");
      return;
    }

    // Get a link to the depth camera
    camera_ = sensor_->DepthCamera();
    if (!camera_) {
      FF_FATAL("GazeboSensorPluginDepthCam cannot get rendering object.");
      return;
    }

    // Look at the intensity component of the depth camera,
    // which we will call the "amplitude"
    // Check that we have a mono camera
    if (camera_->ImageFormat() != "L8")
      FF_FATAL_STREAM("Camera format must be L8");

    std::string topic_prefix = TOPIC_HARDWARE_PICOFLEXX_PREFIX
      + (std::string)(is_haz_cam_ ?
         TOPIC_HARDWARE_NAME_HAZ_CAM : TOPIC_HARDWARE_NAME_PERCH_CAM);

    // Create a publisher for the depth camera intensity
    std::string amplitude_topic = topic_prefix + "/extended/amplitude_int";
    pub_image_ = FF_CREATE_PUBLISHER(nh, sensor_msgs::msg::Image, amplitude_topic, 2);

    // Set image constants
    image_msg_.is_bigendian = false;
    image_msg_.header.frame_id = GetFrame();
    image_msg_.encoding = sensor_msgs::image_encodings::MONO8;

    // Create a publisher for the depth camera point cloud
    std::string point_topic = topic_prefix
                            + (std::string) TOPIC_HARDWARE_PICOFLEXX_SUFFIX;
    pub_point_cloud_ = FF_CREATE_PUBLISHER(nh, sensor_msgs::msg::PointCloud2, point_topic, 1);
    pub_pose_ = FF_CREATE_PUBLISHER(nh, geometry_msgs::msg::PoseStamped,
                        is_haz_cam_ ? TOPIC_HAZ_CAM_SIM_POSE : TOPIC_PERCH_CAM_SIM_POSE, 10);

    // Create a publisher for the intrinsics
    pub_info_ = FF_CREATE_PUBLISHER(nh, sensor_msgs::msg::CameraInfo,
                        is_haz_cam_ ? TOPIC_HAZ_CAM_SIM_INFO : TOPIC_PERCH_CAM_SIM_INFO, 10);

    // Basic header information
    // Must set in <format>L8</format> in sensor_haz_cam.urdf.xacro
    // in order to get an output with 4 channels (x, y, z, intensity)
    num_channels_ = 4;
    point_cloud_msg_.header.frame_id = GetFrame();
    point_cloud_msg_.is_dense = true;
    point_cloud_msg_.is_bigendian = false;
    point_cloud_msg_.point_step = sizeof(float) * num_channels_;

    // Declare the striped memory layout.
    sensor_msgs::msg::PointField field;
    field.name = "x";
    field.offset = 0 * sizeof(float);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes
    point_cloud_msg_.fields.push_back(field);
    field.name = "y";
    field.offset = 1 * sizeof(float);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes
    point_cloud_msg_.fields.push_back(field);
    field.name = "z";
    field.offset = 2 * sizeof(float);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes
    point_cloud_msg_.fields.push_back(field);
    field.name = "intensity";  // for voxblox
    field.offset = 3 * sizeof(float);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes
    point_cloud_msg_.fields.push_back(field);

    // Read configuration
    config_reader::ConfigReader config;
    config.AddFile("simulation/simulation.config");
    if (!config.ReadFiles()) {
      FF_FATAL("Failed to read simulation config file.");
      return;
    }
    bool dos = true;
    if (!config.GetBool("disable_cameras_on_speedup", &dos))
      FF_FATAL("Could not read the drawing_width parameter.");
    if (!config.GetReal(is_haz_cam_ ? "haz_cam_rate" : "perch_cam_rate", &rate_))
      FF_FATAL("Could not read the camera rate parameter.");
    config.Close();

    // If we have a sped up simulation and we need to disable the camera
    double simulation_speed = 1.0;
    if (nh->get_parameter("/simulation_speed", simulation_speed))
      if (simulation_speed > 1.0 && dos) rate_ = 0.0;
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(NodeHandle& nh) {
    // Toggle if the camera is active or not
    ToggleCallback();
    // no callback for subscribers, check for subscribers on timer
    timer_.createTimer(0.5,
      std::bind(&GazeboSensorPluginDepthCam::ToggleCallback, this), nh, false, true);

    // Listen to the point cloud
    depth_update_ = camera_->ConnectNewRGBPointCloud
      (boost::bind(&GazeboSensorPluginDepthCam::PointCloudCallback, this, _1, _2, _3, _4, _5));
    image_update_ = camera_->ConnectNewImageFrame
      (boost::bind(&GazeboSensorPluginDepthCam::ImageCallback, this, _1, _2, _3, _4, _5));
  }

  // Turn camera on or off based on topic subscription
  void ToggleCallback() {
    if ((pub_point_cloud_->get_subscription_count() > 0 ||
         pub_image_->get_subscription_count() > 0) && rate_ > 0) {
      sensor_->SetUpdateRate(rate_);
      sensor_->SetActive(true);
    } else {
      sensor_->SetUpdateRate(0.0001);
      sensor_->SetActive(false);
    }
  }

  // Publish the haz cam image
  void ImageCallback(const unsigned char * image_data, unsigned int width, unsigned height,
                     unsigned int len, const std::string & type) {
    // Publish the haz cam intrinsics
    info_msg_.header.frame_id = GetFrame();
    info_msg_.header.stamp = GetTimeNow();  // it is very important to get the time right
    FillCameraInfo(sensor_->DepthCamera(), info_msg_);  // fill in from the camera pointer
    pub_info_->publish(info_msg_);

    // Record not the current time, but the time when the image was acquired
    image_msg_.header.stamp.sec = sensor_->LastMeasurementTime().sec;
    image_msg_.header.stamp.nanosec = sensor_->LastMeasurementTime().nsec;
    image_msg_.height = camera_->ImageHeight();
    image_msg_.width = camera_->ImageWidth();
    image_msg_.step = image_msg_.width;
    image_msg_.data.resize(image_msg_.step * image_msg_.height);
    const uint8_t* data_start = reinterpret_cast<const uint8_t*>(image_data);
    std::copy(data_start, data_start + image_msg_.step * image_msg_.height,
              image_msg_.data.begin());
    pub_image_->publish(image_msg_);
  }

  // Publish the haz cam cloud and other data
  void PointCloudCallback(const float *depth_data, unsigned int width, unsigned int height,
    unsigned int len, const std::string & type) {

    // Publish the haz cam pose
    Eigen::Affine3d sensor_to_world = SensorToWorld(GetModel()->WorldPose(), sensor_->Pose());
    pose_msg_.header.frame_id = GetFrame();
    pose_msg_.header.stamp = GetTimeNow();  // it is very important to get the time right
    pose_msg_.pose.position.x = sensor_to_world.translation().x();
    pose_msg_.pose.position.y = sensor_to_world.translation().y();
    pose_msg_.pose.position.z = sensor_to_world.translation().z();

    Eigen::Quaterniond q(sensor_to_world.rotation());
    pose_msg_.pose.orientation.w = q.w();
    pose_msg_.pose.orientation.x = q.x();
    pose_msg_.pose.orientation.y = q.y();
    pose_msg_.pose.orientation.z = q.z();
    pub_pose_->publish(pose_msg_);

    // Ensure that the cloud we publish has the timestamp for when
    // the data was actually measured.
    point_cloud_msg_.header.stamp.sec  = sensor_->LastMeasurementTime().sec;
    point_cloud_msg_.header.stamp.nanosec = sensor_->LastMeasurementTime().nsec;

    // Populate the point cloud fields
    point_cloud_msg_.width = width;
    point_cloud_msg_.height = height;
    point_cloud_msg_.row_step = point_cloud_msg_.width * point_cloud_msg_.point_step;

    size_t num_bytes = point_cloud_msg_.row_step * point_cloud_msg_.height;
    size_t num_floats = num_bytes / sizeof(float);

    // Sanity checks
    if (num_channels_ != 4)
      FF_FATAL("Expecting 4 channels.");
    if (num_floats != num_channels_ * width * height)
      FF_FATAL("Expecting 4 floats per pixel.");

    // Copy the x, y, z values and the intensity. The latter is, for some reason, always 1.
    std::vector<float> cloud_data(num_floats);
    for (size_t i = 0; i < num_floats; i++)
      cloud_data[i] = depth_data[i];

    // Copy the correct intensity. NOTE(oalexan1): This intensity data
    // seems to be not fresh. The ImageCallback() above publishes the
    // correct intensity on a separate topic.
    const unsigned char * image_data = camera_->ImageData();

    if (!image_data) {
      // first frame of image data may not be received
      return;
    }
    for (size_t i = 0; i < num_floats / num_channels_; i++)
      cloud_data[num_channels_ * i + num_channels_ - 1] = static_cast<float>(image_data[i]);

    // Copy these to the point cloud
    point_cloud_msg_.data.resize(num_bytes);
    std::copy(reinterpret_cast<const uint8_t*>(&cloud_data[0]),              // input beg
              reinterpret_cast<const uint8_t*>(&cloud_data[0]) + num_bytes,  // input end
              point_cloud_msg_.data.begin());                                // destination

    pub_point_cloud_->publish(point_cloud_msg_);
  }

 private:
  ff_util::FreeFlyerTimer timer_;
  // Published topics
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;

  // Sensor and camera pointers
  sensors::DepthCameraSensorPtr sensor_;
  rendering::DepthCameraPtr camera_;

  // Published messages
  sensor_msgs::msg::PointCloud2 point_cloud_msg_;
  sensor_msgs::msg::Image image_msg_;
  geometry_msgs::msg::PoseStamped pose_msg_;
  sensor_msgs::msg::CameraInfo info_msg_;

  event::ConnectionPtr depth_update_, image_update_;
  double rate_;
  int num_channels_;
  bool is_haz_cam_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginDepthCam)

}   // namespace gazebo
