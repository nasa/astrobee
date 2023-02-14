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

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// FSW includes
#include <config_reader/config_reader.h>

// Camera model
#include <camera/camera_model.h>
#include <camera/camera_params.h>

// FSW messages
#include <ff_msgs/msg/visual_landmarks.hpp>
#include <ff_msgs/msg/camera_registration.hpp>
#include <ff_msgs/srv/set_bool.hpp>
namespace ff_msgs {
typedef msg::VisualLandmark VisualLandmark;
typedef msg::VisualLandmarks VisualLandmarks;
typedef msg::CameraRegistration CameraRegistration;
typedef srv::SetBool SetBool;
}  // namespace ff_msgs

// STL includes
#include <string>

namespace gazebo {

FF_DEFINE_LOGGER("gazebo_sensor_plugin_sparse_map");

class GazeboSensorPluginSparseMap : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginSparseMap() :
    FreeFlyerSensorPlugin("localization_node", "nav_cam", true),
      active_(true) {}

  ~GazeboSensorPluginSparseMap() {}

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(NodeHandle &nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginSparseMap requires a parent camera sensor.\n";
      return;
    }

    // Check that we have a mono camera
    if (sensor_->Camera()->ImageFormat() != "L8")
      FF_FATAL_STREAM("Camera format must be L8");

    // Build the nav cam Camera Model
    config_.AddFile("cameras.config");
    config_.AddFile("simulation/sparse_map.config");
    if (!config_.ReadFiles()) {
        FF_ERROR("Failed to read config files.");
        return;
    }

    if (!config_.GetReal("rate", &rate_))
      FF_FATAL("Could not read the rate parameter.");

    if (!config_.GetReal("delay_camera", &delay_camera_))
      FF_FATAL("Could not read the delay_camera parameter.");

    if (!config_.GetReal("delay_features", &delay_features_))
      FF_FATAL("Could not read the delay_features parameter.");

    if (!config_.GetReal("near_clip", &near_clip_))
      FF_FATAL("Could not read the near_clip parameter.");

    if (!config_.GetReal("far_clip", &far_clip_))
      FF_FATAL("Could not read the far_clip parameter.");

    if (!config_.GetUInt("num_samp", &num_samp_))
      FF_ERROR("Could not read the num_samp parameter.");

    if (!config_.GetUInt("num_features", &num_features_))
      FF_FATAL("Could not read the num_features parameter.");

    // Create a publisher for the registration messages
    pub_reg_ = FF_CREATE_PUBLISHER(nh, ff_msgs::CameraRegistration, TOPIC_LOCALIZATION_ML_REGISTRATION, 1);

    // Create a publisher for the feature messages
    pub_feat_ = FF_CREATE_PUBLISHER(nh, ff_msgs::VisualLandmarks, TOPIC_LOCALIZATION_ML_FEATURES, 1);

    // Create a shape for collision testing
    #if GAZEBO_MAJOR_VERSION > 7
     GetWorld()->Physics()->InitForThread();
     shape_ = boost::dynamic_pointer_cast<physics::RayShape>(GetWorld()
        ->Physics()->CreateShape("ray", physics::CollisionPtr()));
    #else
    GetWorld()->GetPhysicsEngine()->InitForThread();
    shape_ = boost::dynamic_pointer_cast<physics::RayShape>(GetWorld()
        ->GetPhysicsEngine()->CreateShape("ray", physics::CollisionPtr()));
    #endif

    // Only do this once
    msg_feat_.header.frame_id = std::string(FRAME_NAME_WORLD);
    msg_reg_.header.frame_id = std::string(FRAME_NAME_WORLD);
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(NodeHandle &nh) {
    // Sercide for enabling mapped landmarks
    srv_enable_ = nh->create_service<ff_msgs::SetBool>(SERVICE_LOCALIZATION_ML_ENABLE,
      std::bind(&GazeboSensorPluginSparseMap::EnableService, this, std::placeholders::_1, std::placeholders::_2));

    // Timer triggers registration
    timer_registration_.createTimer(1 / rate_,
      std::bind(&GazeboSensorPluginSparseMap::SendRegistration, this), nh, false, true);

    // Timer triggers features
    timer_features_.createTimer(0.8 / rate_,
      std::bind(&GazeboSensorPluginSparseMap::SendFeatures, this), nh, true, false);
  }

  // Enable or disable the feature timer
  bool EnableService(const std::shared_ptr<ff_msgs::SetBool::Request> req,
                     std::shared_ptr<ff_msgs::SetBool::Response> res) {
    active_ = req->enable;
    res->success = true;
    return true;
  }


  // Send a registration pulse
  void SendRegistration() {
    if (!active_) return;

    // Add a short delay between the features and new registration pulse
    timer_features_.stop();
    timer_features_.start();

    // Send the registration pulse
    msg_reg_.header.stamp = GetTimeNow() + rclcpp::Duration::from_seconds(delay_camera_);
    msg_reg_.camera_id++;
    pub_reg_->publish(msg_reg_);
    // FF_SPIN_ONCE();

    // Copy over the camera id to the feature message
    msg_feat_.camera_id = msg_reg_.camera_id;

    // Handle the transform for all sensor types
    #if GAZEBO_MAJOR_VERSION > 7
      Eigen::Affine3d sensor_to_world = SensorToWorld(GetModel()->WorldPose(), sensor_->Pose());
    #else
      Eigen::Affine3d sensor_to_world = SensorToWorld(GetModel()->GetWorldPose(), sensor_->Pose());
    #endif

    msg_feat_.header.stamp = GetTimeNow();

    // Initialize the camera parameters
    static camera::CameraParameters cam_params(&config_, "nav_cam");
    static camera::CameraModel camera(Eigen::Vector3d(0, 0, 0),
      Eigen::Matrix3d::Identity(), cam_params);

    // Assemble the feature message
    msg_feat_.pose.position.x = sensor_to_world.translation().x();
    msg_feat_.pose.position.y = sensor_to_world.translation().y();
    msg_feat_.pose.position.z = sensor_to_world.translation().z();
    Eigen::Quaterniond q(sensor_to_world.rotation());
    msg_feat_.pose.orientation.w = q.w();
    msg_feat_.pose.orientation.x = q.x();
    msg_feat_.pose.orientation.y = q.y();
    msg_feat_.pose.orientation.z = q.z();
    msg_feat_.landmarks.clear();

    {
      // Initialize and lock the physics engine
    #if GAZEBO_MAJOR_VERSION > 7
      GetWorld()->Physics()->InitForThread();
      boost::unique_lock<boost::recursive_mutex> lock(*(
          GetWorld()->Physics()->GetPhysicsUpdateMutex()));
      std::string physics = boost::any_cast<std::string>(GetWorld()->Physics()->GetParam("type"));
    #else
      GetWorld()->GetPhysicsEngine()->InitForThread();
      boost::unique_lock<boost::recursive_mutex> lock(*(
        GetWorld()->GetPhysicsEngine()->GetPhysicsUpdateMutex()));
      std::string physics = boost::any_cast<std::string>(GetWorld()->GetPhysicsEngine()->GetParam("type"));
    #endif
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
        Eigen::Vector3d n_w = sensor_to_world * n_c;
        Eigen::Vector3d f_w = sensor_to_world * f_c;

        // Collision detection
        double dist;
        if (physics != "dart") {
          std::string entity;

          // Set the start anf end points of the ray
          shape_->SetPoints(
            ignition::math::Vector3d(n_w.x(), n_w.y(), n_w.z()),
            ignition::math::Vector3d(f_w.x(), f_w.y(), f_w.z()));
          shape_->GetIntersection(dist, entity);

          // If we don't have an entity then we didnt collide
          if (entity.empty())
            continue;
        } else {
          // The ray functionality does not work for dart physics engine
          // Here we just fake some random distance values
          // https://answers.gazebosim.org/question/16241/ray-sensor-doesnt-work-properly-with-dart/
          dist = std::max(near_clip_, std::min(static_cast<double>(rand() % 1000) / 500, far_clip_));  // NOLINT
        }

        // Get the landmark coordinate
        Eigen::Vector3d p_w = n_w + dist * (f_w - n_w).normalized();

        // Create the landmark message
        ff_msgs::VisualLandmark landmark;
        landmark.x = p_w.x();
        landmark.y = p_w.y();
        landmark.z = p_w.z();
        landmark.u = camera.ImageCoordinates(f_c)[0];
        landmark.v = camera.ImageCoordinates(f_c)[1];
        msg_feat_.landmarks.push_back(landmark);
      }
    }
  }

  // Send off the features
  void SendFeatures() {
    if (!active_) return;
    pub_feat_->publish(msg_feat_);
  }

 private:
  config_reader::ConfigReader config_;
  rclcpp::Publisher<ff_msgs::CameraRegistration>::SharedPtr pub_reg_;
  rclcpp::Publisher<ff_msgs::VisualLandmarks>::SharedPtr pub_feat_;
  rclcpp::Service<ff_msgs::SetBool>::SharedPtr srv_enable_;
  ff_util::FreeFlyerTimer timer_registration_, timer_features_;
  std::shared_ptr<sensors::WideAngleCameraSensor> sensor_;
  gazebo::physics::RayShapePtr shape_;
  bool active_;
  ff_msgs::VisualLandmarks msg_feat_;
  ff_msgs::CameraRegistration msg_reg_;
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
