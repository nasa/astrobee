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

// FSW includes
#include <config_reader/config_reader.h>

// FSW messages
#include <ff_msgs/VisualLandmarks.h>
#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/SetBool.h>

// Camera model
#include <camera/camera_model.h>
#include <camera/camera_params.h>

// STL includes
#include <string>

namespace gazebo {

class GazeboSensorPluginSparseMap : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginSparseMap() :
    FreeFlyerSensorPlugin("localization_node", "nav_cam", true),
      active_(true), processing_(false) {}

  virtual ~GazeboSensorPluginSparseMap() {}

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle *nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginSparseMap requires a parent camera sensor.\n";
      return;
    }

    // Check that we have a mono camera
    if (sensor_->Camera()->ImageFormat() != "L8")
      ROS_FATAL_STREAM("Camera format must be L8");

    // Build the nav cam Camera Model
    config_.AddFile("cameras.config");
    config_.AddFile("simulation/sparse_map.config");
    if (!config_.ReadFiles()) {
        ROS_ERROR("Failed to read config files.");
        return;
    }

    if (!config_.GetReal("rate", &rate_))
      ROS_FATAL("Could not read the rate parameter.");

    if (!config_.GetReal("delay_camera", &delay_camera_))
      ROS_FATAL("Could not read the delay_camera parameter.");

    if (!config_.GetReal("delay_features", &delay_features_))
      ROS_FATAL("Could not read the delay_features parameter.");

    if (!config_.GetReal("near_clip", &near_clip_))
      ROS_FATAL("Could not read the near_clip parameter.");

    if (!config_.GetReal("far_clip", &far_clip_))
      ROS_FATAL("Could not read the far_clip parameter.");

    if (!config_.GetUInt("num_samp", &num_samp_))
      NODELET_ERROR("Could not read the num_samp parameter.");

    if (!config_.GetUInt("num_features", &num_features_))
      ROS_FATAL("Could not read the num_features parameter.");

    // Create a publisher for the registration messages
    pub_reg_ = nh->advertise<ff_msgs::CameraRegistration>(
      TOPIC_LOCALIZATION_ML_REGISTRATION, 100);

    // Create a publisher for the feature messages
    pub_feat_ = nh->advertise<ff_msgs::VisualLandmarks>(
      TOPIC_LOCALIZATION_ML_FEATURES, 100);

    // Enable mapped landmarks
    srv_enable_ = nh->advertiseService(SERVICE_LOCALIZATION_ML_ENABLE,
      &GazeboSensorPluginSparseMap::EnableService, this);

    // Timer triggers features
    timer_registration_ = nh->createTimer(ros::Duration(ros::Rate(rate_)),
      &GazeboSensorPluginSparseMap::SendRegistration, this, false, true);

    // Timer triggers features
    timer_delay_ = nh->createTimer(ros::Duration(delay_features_),
      &GazeboSensorPluginSparseMap::SendFeatures, this, true, false);

    // Create a shape for collision testing
    GetWorld()->GetPhysicsEngine()->InitForThread();
    shape_ = boost::dynamic_pointer_cast<physics::RayShape>(GetWorld()
        ->GetPhysicsEngine()->CreateShape("ray", physics::CollisionPtr()));
  }

  // Enable or disable the feature timer
  bool EnableService(ff_msgs::SetBool::Request & req,
                     ff_msgs::SetBool::Response & res) {
    active_ = req.enable;
    processing_ = false;
    res.success = true;
    return true;
  }

  // Send a registration pulse
  void SendRegistration(ros::TimerEvent const& event) {
    if (processing_ || !active_ || !ExtrinsicsFound()) return;
    // Initialize and lock the physics engine
    GetWorld()->GetPhysicsEngine()->InitForThread();
    boost::unique_lock<boost::recursive_mutex> lock(*(
      GetWorld()->GetPhysicsEngine()->GetPhysicsUpdateMutex()));

    // We are now processing an image
    processing_ = true;

    // Send a registration pulse
    static ff_msgs::CameraRegistration msg_reg;
    msg_reg.header.stamp = ros::Time::now() + ros::Duration(delay_camera_);
    msg_reg.header.frame_id = std::string(FRAME_NAME_WORLD);
    msg_reg.camera_id++;
    pub_reg_.publish(msg_reg);
    ros::spinOnce();

    // Handle the transform for all sensor types
    Eigen::Affine3d wTb = (
        Eigen::Translation3d(
          GetModel()->GetWorldPose().pos.x,
          GetModel()->GetWorldPose().pos.y,
          GetModel()->GetWorldPose().pos.z) *
        Eigen::Quaterniond(
          GetModel()->GetWorldPose().rot.w,
          GetModel()->GetWorldPose().rot.x,
          GetModel()->GetWorldPose().rot.y,
          GetModel()->GetWorldPose().rot.z));
    Eigen::Affine3d bTs = (
        Eigen::Translation3d(
          sensor_->Pose().Pos().X(),
          sensor_->Pose().Pos().Y(),
          sensor_->Pose().Pos().Z()) *
        Eigen::Quaterniond(
          sensor_->Pose().Rot().W(),
          sensor_->Pose().Rot().X(),
          sensor_->Pose().Rot().Y(),
          sensor_->Pose().Rot().Z()));
    Eigen::Affine3d wTs = wTb * bTs;

    // Initialize the camera paremeters
    camera::CameraParameters cam_params(&config_, "nav_cam");
    camera::CameraModel camera(Eigen::Vector3d(0, 0, 0),
      Eigen::Matrix3d::Identity(), cam_params);

    // Assemble the feature message
    msg_feat_.camera_id = msg_reg.camera_id;
    msg_feat_.pose.position.x = wTs.translation().x();
    msg_feat_.pose.position.y = wTs.translation().y();
    msg_feat_.pose.position.z = wTs.translation().z();
    Eigen::Quaterniond q(wTs.rotation());
    msg_feat_.pose.orientation.w = q.w();
    msg_feat_.pose.orientation.x = q.x();
    msg_feat_.pose.orientation.y = q.y();
    msg_feat_.pose.orientation.z = q.z();
    msg_feat_.landmarks.clear();

    // Create a new ray in the world
    size_t i = 0;
    for (; i < num_samp_ && msg_feat_.landmarks.size() < num_features_; i++) {
      // Get a ray through a random image coordinate
      Eigen::Vector3d ray = camera.Ray(
        (static_cast<double>(rand() % 1000) / 1000 - 0.5)  // NOLINT
          * camera.GetParameters().GetDistortedSize()[0],
        (static_cast<double>(rand() % 1000) / 1000 - 0.5)  // NOLINT
          * camera.GetParameters().GetDistortedSize()[1]);

      // Get the camera coordinate of the ray near and far clips
      Eigen::Vector3d n_c = near_clip_ * ray;
      Eigen::Vector3d f_c = far_clip_ * ray;

      // Get the world coordinate of the ray near and far clips
      Eigen::Vector3d n_w = wTs * n_c;
      Eigen::Vector3d f_w = wTs * f_c;

      // Collision detection
      double dist;
      std::string entity;

      // Set the start anf end points of the ray
      shape_->SetPoints(
        ignition::math::Vector3d(n_w.x(), n_w.y(), n_w.z()),
        ignition::math::Vector3d(f_w.x(), f_w.y(), f_w.z()));
      shape_->GetIntersection(dist, entity);

      // If we don't have an entity then we didnt collide
      if (entity.empty())
        continue;

      // Get the landmark coordinate
      Eigen::Vector3d p_w = n_w + dist * (f_w - n_w).normalized();

      // Create the landmark message
      static ff_msgs::VisualLandmark landmark;
      landmark.x = p_w.x();
      landmark.y = p_w.y();
      landmark.z = p_w.z();
      landmark.u = camera.ImageCoordinates(f_c)[0];
      landmark.v = camera.ImageCoordinates(f_c)[1];
      msg_feat_.landmarks.push_back(landmark);
    }

    // Start a delay for the message
    timer_delay_.stop();
    timer_delay_.start();
  }

  // Called when featured must be sent
  void SendFeatures(ros::TimerEvent const& event) {
    if (!processing_ || !active_ || !ExtrinsicsFound()) return;

    // Send off the features
    msg_feat_.header.stamp = ros::Time::now();
    msg_feat_.header.frame_id = std::string(FRAME_NAME_WORLD);
    pub_feat_.publish(msg_feat_);
    ros::spinOnce();

    // Go back to an idel state
    processing_ = false;
  }

 private:
  config_reader::ConfigReader config_;
  ros::Publisher pub_reg_, pub_feat_;
  ros::ServiceServer srv_enable_;
  ros::Timer timer_delay_, timer_registration_;
  sensors::WideAngleCameraSensorPtr sensor_;
  gazebo::physics::RayShapePtr shape_;
  bool active_, processing_;
  ff_msgs::VisualLandmarks msg_feat_;
  double rate_;
  double delay_camera_;
  double delay_features_;
  double near_clip_;
  double far_clip_;
  unsigned int num_features_;
  unsigned int num_samp_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginSparseMap)

}   // namespace gazebo
