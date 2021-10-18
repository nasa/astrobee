// Copyright 2016 Intelligent RObotics Group, NASA ARC

#ifndef IMAGE_SAMPLER_IMAGE_SAMPLER_H_
#define IMAGE_SAMPLER_IMAGE_SAMPLER_H_

#include <pluginlib/class_list_macros.h>

#include <image_transport/image_transport.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <ff_util/ff_nodelet.h>
#include <ff_msgs/ConfigureCamera.h>
#include <ff_msgs/EnableCamera.h>
#include <ff_msgs/CameraState.h>

#include <string>

#define NUM_CAMERAS       6
#define NAV_CAM_ID        0
#define DOCK_CAM_ID       1
#define HAZ_CAM_ID        2
#define PERCH_CAM_ID      3
#define NAV_CAM_BAYER_ID  4
#define DOCK_CAM_BAYER_ID 5

namespace image_sampler {

class ImageSampler : public ff_util::FreeFlyerNodelet {
 public:
  ImageSampler();
  ~ImageSampler();

 protected:
  virtual void Initialize(ros::NodeHandle *nh);
  bool ConfigureService(ff_msgs::ConfigureCamera::Request& req, ff_msgs::ConfigureCamera::Response& res, int camera);  // NOLINT
  bool ConfigureServiceNavCam(ff_msgs::ConfigureCamera::Request& req, ff_msgs::ConfigureCamera::Response& res);  // NOLINT
  bool ConfigureServiceDockCam(ff_msgs::ConfigureCamera::Request& req, ff_msgs::ConfigureCamera::Response& res);  // NOLINT
  bool ConfigureServiceHazCam(ff_msgs::ConfigureCamera::Request& req, ff_msgs::ConfigureCamera::Response& res);  // NOLINT
  bool ConfigureServicePerchCam(ff_msgs::ConfigureCamera::Request& req, ff_msgs::ConfigureCamera::Response& res);  // NOLINT
  bool ConfigureServiceNavCamBayer(ff_msgs::ConfigureCamera::Request& req, ff_msgs::ConfigureCamera::Response& res);  // NOLINT
  bool ConfigureServiceDockCamBayer(ff_msgs::ConfigureCamera::Request& req, ff_msgs::ConfigureCamera::Response& res);  // NOLINT
  bool EnableService(ff_msgs::EnableCamera::Request& req, ff_msgs::EnableCamera::Response& res,  // NOLINT
        int camera, std::string topic, void (ImageSampler::*callback)(const sensor_msgs::ImageConstPtr &));
  bool EnableServiceNavCam(ff_msgs::EnableCamera::Request& req, ff_msgs::EnableCamera::Response& res);  // NOLINT
  bool EnableServiceDockCam(ff_msgs::EnableCamera::Request& req, ff_msgs::EnableCamera::Response& res);  // NOLINT
  bool EnableServiceHazCam(ff_msgs::EnableCamera::Request& req, ff_msgs::EnableCamera::Response& res);  // NOLINT
  bool EnableServicePerchCam(ff_msgs::EnableCamera::Request& req, ff_msgs::EnableCamera::Response& res);  // NOLINT
  bool EnableServiceNavCamBayer(ff_msgs::EnableCamera::Request& req, ff_msgs::EnableCamera::Response& res);  // NOLINT
  bool EnableServiceDockCamBayer(ff_msgs::EnableCamera::Request& req, ff_msgs::EnableCamera::Response& res);  // NOLINT
  void NavCamCallback(const sensor_msgs::ImageConstPtr & msg);
  void DockCamCallback(const sensor_msgs::ImageConstPtr & msg);
  void HazCamCallback(const sensor_msgs::ImageConstPtr & msg);
  void PerchCamCallback(const sensor_msgs::ImageConstPtr & msg);
  void NavCamBayerCallback(const sensor_msgs::ImageConstPtr & msg);
  void DockCamBayerCallback(const sensor_msgs::ImageConstPtr & msg);
  void ImageCallback(const sensor_msgs::ImageConstPtr & msg, int camera);
  void UpdateState(int camera, bool streaming, int width, int height, float rate);

 private:
  image_transport::Subscriber image_sub_[NUM_CAMERAS];
  image_transport::CameraPublisher record_image_pub_[NUM_CAMERAS];
  image_transport::CameraPublisher stream_image_pub_[NUM_CAMERAS];
  ros::ServiceServer configure_srv_[NUM_CAMERAS];
  ros::ServiceServer enable_srv_[NUM_CAMERAS];
  ros::Publisher camera_state_pub_;

  std::string camera_topics_[NUM_CAMERAS];
  int record_output_width_[NUM_CAMERAS], record_output_height_[NUM_CAMERAS];
  int stream_output_width_[NUM_CAMERAS], stream_output_height_[NUM_CAMERAS];
  ros::Duration record_publication_interval_[NUM_CAMERAS];
  ros::Duration stream_publication_interval_[NUM_CAMERAS];
  ros::Time record_last_publish_time_[NUM_CAMERAS];
  ros::Time stream_last_publish_time_[NUM_CAMERAS];

  ff_msgs::CameraState camera_states_[2 * NUM_CAMERAS];
};

}  // namespace image_sampler

#endif  // IMAGE_SAMPLER_IMAGE_SAMPLER_H_
