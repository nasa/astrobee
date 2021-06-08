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

// ROS includes
#include <ros/ros.h>

#include <astrobee_gazebo/astrobee_gazebo.h>
#include <config_reader/config_reader.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

// STL includes
#include <string>

namespace gazebo {
class GazeboSensorPluginHazCam : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginHazCam() :
    FreeFlyerSensorPlugin("pico_driver", "haz_cam", true), rate_(0.0), num_channels_(0) {
  }

  ~GazeboSensorPluginHazCam() {
    if (depth_update_) {
      #if GAZEBO_MAJOR_VERSION > 7
      depth_update_.reset();
      #else
      camera_->DisconnectNewRGBPointCloud(depth_update_);
      #endif
    }
    if (image_update_) {
      #if GAZEBO_MAJOR_VERSION > 7
      image_update_.reset();
      #else
      camera_->DisconnectNewImageFrame(image_update_);
      #endif
    }
  }

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle *nh,
                    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginHazCam requires a depth camera sensor.\n";
      return;
    }

    // Get a link to the depth camera
    camera_ = sensor_->DepthCamera();
    if (!camera_) {
      gzerr << "GazeboSensorPluginHazCam cannot get rendering object.\n";
      return;
    }

    // Look at the intensity component of the depth camera,
    // which we will call the "amplitude"
    // Check that we have a mono camera
    if (camera_->ImageFormat() != "L8")
      ROS_FATAL_STREAM("Camera format must be L8");

    // Create a publisher for the depth camera intensity
    std::string amplitude_topic =
      TOPIC_HARDWARE_PICOFLEXX_PREFIX
      + (std::string) TOPIC_HARDWARE_NAME_HAZ_CAM
      + "/extended/amplitude_int";
    pub_image_ = nh->advertise<sensor_msgs::Image>(amplitude_topic, 2);

    // Set image constants
    image_msg_.is_bigendian = false;
    image_msg_.header.frame_id = GetFrame();
    image_msg_.encoding = sensor_msgs::image_encodings::MONO8;

    // Create a publisher for the depth camera point cloud
    std::string point_topic = TOPIC_HARDWARE_PICOFLEXX_PREFIX
                            + (std::string) TOPIC_HARDWARE_NAME_HAZ_CAM
                            + (std::string) TOPIC_HARDWARE_PICOFLEXX_SUFFIX;
    pub_point_cloud_ = nh->advertise<sensor_msgs::PointCloud2>(point_topic, 1,
      boost::bind(&GazeboSensorPluginHazCam::ToggleCallback, this),
      boost::bind(&GazeboSensorPluginHazCam::ToggleCallback, this));
    pub_pose_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_HAZ_CAM_SIM_POSE, 10);

    // Create a publisher for the intrinsics
    pub_info_ = nh->advertise<sensor_msgs::CameraInfo>(TOPIC_HAZ_CAM_SIM_INFO, 10);

    // Basic header information
    // Must set in <format>L8</format> in sensor_haz_cam.urdf.xacro
    // in order to get an output with 4 channels (x, y, z, intensity)
    num_channels_ = 4;
    point_cloud_msg_.header.frame_id = GetFrame();
    point_cloud_msg_.is_dense = true;
    point_cloud_msg_.is_bigendian = false;
    point_cloud_msg_.point_step = sizeof(float) * num_channels_;

    // Declare the striped memory layout.
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
    field.name = "intensity";  // for voxblox
    field.offset = 3 * sizeof(float);
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
    if (!config.GetReal("haz_cam_rate", &rate_))
      ROS_FATAL("Could not read the drawing_width parameter.");
    config.Close();

    // If we have a sped up simulation and we need to disable the camera
    double simulation_speed = 1.0;
    if (nh->getParam("/simulation_speed", simulation_speed))
      if (simulation_speed > 1.0 && dos) rate_ = 0.0;
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(ros::NodeHandle *nh) {
    // Toggle if the camera is active or not
    ToggleCallback();

    // Listen to the point cloud
    depth_update_ = camera_->ConnectNewRGBPointCloud
      (boost::bind(&GazeboSensorPluginHazCam::PointCloudCallback, this, _1, _2, _3, _4, _5));
    image_update_ = camera_->ConnectNewImageFrame
      (boost::bind(&GazeboSensorPluginHazCam::ImageCallback, this, _1, _2, _3, _4, _5));
  }

  // Turn camera on or off based on topic subscription
  void ToggleCallback() {
    if (pub_point_cloud_.getNumSubscribers() > 0 && rate_ > 0) {
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
    ros::Time curr_time = ros::Time::now();
    info_msg_.header.frame_id = GetFrame();
    info_msg_.header.stamp = curr_time;  // it is very important to get the time right
    FillCameraInfo(sensor_->DepthCamera(), info_msg_);  // fill in from the camera pointer
    pub_info_.publish(info_msg_);

    // Record not the current time, but the time when the image was acquired
    image_msg_.header.stamp.sec = sensor_->LastMeasurementTime().sec;
    image_msg_.header.stamp.nsec = sensor_->LastMeasurementTime().nsec;
    image_msg_.height = camera_->ImageHeight();
    image_msg_.width = camera_->ImageWidth();
    image_msg_.step = image_msg_.width;
    image_msg_.data.resize(image_msg_.step * image_msg_.height);
    const uint8_t* data_start = reinterpret_cast<const uint8_t*>(image_data);
    std::copy(data_start, data_start + image_msg_.step * image_msg_.height,
              image_msg_.data.begin());
    pub_image_.publish(image_msg_);
  }

  // Publish the haz cam cloud and other data
  void PointCloudCallback(const float *depth_data, unsigned int width, unsigned int height,
    unsigned int len, const std::string & type) {
    // Quickly record the current time and current pose before doing other computations
    ros::Time curr_time = ros::Time::now();

    // Publish the haz cam pose
    #if GAZEBO_MAJOR_VERSION > 7
    Eigen::Affine3d sensor_to_world = SensorToWorld(GetModel()->WorldPose(), sensor_->Pose());
    #else
    Eigen::Affine3d sensor_to_world = SensorToWorld(GetModel()->GetWorldPose(), sensor_->Pose());
    #endif
    pose_msg_.header.frame_id = GetFrame();
    pose_msg_.header.stamp = curr_time;  // it is very important to get the time right
    pose_msg_.pose.position.x = sensor_to_world.translation().x();
    pose_msg_.pose.position.y = sensor_to_world.translation().y();
    pose_msg_.pose.position.z = sensor_to_world.translation().z();

    Eigen::Quaterniond q(sensor_to_world.rotation());
    pose_msg_.pose.orientation.w = q.w();
    pose_msg_.pose.orientation.x = q.x();
    pose_msg_.pose.orientation.y = q.y();
    pose_msg_.pose.orientation.z = q.z();
    pub_pose_.publish(pose_msg_);

    // Ensure that the cloud we publish has the timestamp for when
    // the data was actually measured.
    point_cloud_msg_.header.stamp.sec  = sensor_->LastMeasurementTime().sec;
    point_cloud_msg_.header.stamp.nsec = sensor_->LastMeasurementTime().nsec;

    // Populate the point cloud fields
    point_cloud_msg_.width = width;
    point_cloud_msg_.height = height;
    point_cloud_msg_.row_step = point_cloud_msg_.width * point_cloud_msg_.point_step;

    size_t num_bytes = point_cloud_msg_.row_step * point_cloud_msg_.height;
    size_t num_floats = num_bytes / sizeof(float);

    // Sanity checks
    if (num_channels_ != 4)
      ROS_FATAL("Expecting 4 channels.");
    if (num_floats != num_channels_ * width * height)
      ROS_FATAL("Expecting 4 floats per pixel.");

    // Copy the x, y, z values and the intensity. The latter is, for some reason, always 1.
    std::vector<float> cloud_data(num_floats);
    for (size_t i = 0; i < num_floats; i++)
      cloud_data[i] = depth_data[i];

    // Copy the correct intensity. NOTE(oalexan1): This intensity data
    // seems to be not fresh. The ImageCallback() above publishes the
    // correct intensity on a separate topic.
    const unsigned char * image_data = camera_->ImageData();

    if (!image_data) {
        //first frame of image data may not be received
        return;
    }
    for (size_t i = 0; i < num_floats / num_channels_; i++)
      cloud_data[num_channels_ * i + num_channels_ - 1] = static_cast<float>(image_data[i]);

    // Copy these to the point cloud
    point_cloud_msg_.data.resize(num_bytes);
    std::copy(reinterpret_cast<const uint8_t*>(&cloud_data[0]),              // input beg
              reinterpret_cast<const uint8_t*>(&cloud_data[0]) + num_bytes,  // input end
              point_cloud_msg_.data.begin());                                // destination

    pub_point_cloud_.publish(point_cloud_msg_);
  }

 private:
  // Published topics
  ros::Publisher pub_image_;
  ros::Publisher pub_point_cloud_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_info_;

  // Sensor and camera pointers
  sensors::DepthCameraSensorPtr sensor_;
  rendering::DepthCameraPtr camera_;

  // Published messages
  sensor_msgs::PointCloud2 point_cloud_msg_;
  sensor_msgs::Image image_msg_;
  geometry_msgs::PoseStamped pose_msg_;
  sensor_msgs::CameraInfo info_msg_;

  event::ConnectionPtr depth_update_, image_update_;
  double rate_;
  int num_channels_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginHazCam)

}   // namespace gazebo
