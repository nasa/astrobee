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


#include <image_sampler/image_sampler.h>

#include <common/init.h>
#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <ff_msgs/CameraStatesStamped.h>

namespace image_sampler {

ImageSampler::ImageSampler() :
    ff_util::FreeFlyerNodelet(NODE_IMG_SAMPLER) {
}

ImageSampler::~ImageSampler() {
}

void ImageSampler::Initialize(ros::NodeHandle *nh) {
  camera_states_[NAV_CAM_ID].camera_name =  "nav_cam";
  camera_states_[DOCK_CAM_ID].camera_name = "dock_cam";

  for (int i = 0; i < NUM_CAMERAS; i++) {
    record_last_publish_time_[i] = ros::Time::now();
    stream_last_publish_time_[i] = ros::Time::now();
    record_publication_interval_[i] = ros::Duration(0.0);
    stream_publication_interval_[i] = ros::Duration(0.0);
    stream_output_width_[i] = 1;
    stream_output_height_[i] = 1;
    record_output_width_[i] = 1;
    record_output_height_[i] = 1;

    camera_states_[i].streaming = false;
    camera_states_[i].recording = false;
    camera_states_[i].stream_width = 1;
    camera_states_[i].stream_height = 1;
    camera_states_[i].stream_rate = 1.0;
    camera_states_[i].record_width = 1;
    camera_states_[i].record_height = 1;
    camera_states_[i].record_rate = 1.0;
    camera_states_[i].bandwidth = 0.0;
  }

  camera_state_pub_ = nh->advertise<ff_msgs::CameraStatesStamped>(TOPIC_MANAGEMENT_CAMERA_STATE, 5, true);
  // update some state so we publish an update
  UpdateState(NAV_CAM_ID, false, 1, 1, 1.0);

  image_transport::ImageTransport img_transp(*nh);
  record_image_pub_[NAV_CAM_ID]  = img_transp.advertiseCamera(TOPIC_MANAGEMENT_IMG_SAMPLER_NAV_CAM_RECORD,  1);
  stream_image_pub_[NAV_CAM_ID]  = img_transp.advertiseCamera(TOPIC_MANAGEMENT_IMG_SAMPLER_NAV_CAM_STREAM,  1);
  record_image_pub_[DOCK_CAM_ID] = img_transp.advertiseCamera(TOPIC_MANAGEMENT_IMG_SAMPLER_DOCK_CAM_RECORD, 1);
  stream_image_pub_[DOCK_CAM_ID] = img_transp.advertiseCamera(TOPIC_MANAGEMENT_IMG_SAMPLER_DOCK_CAM_STREAM, 1);

  configure_srv_[NAV_CAM_ID]  = nh->advertiseService(SERVICE_MANAGEMENT_IMG_SAMPLER_CONFIG_NAV,
                                                     &ImageSampler::ConfigureServiceNavCam,  this);
  configure_srv_[DOCK_CAM_ID] = nh->advertiseService(SERVICE_MANAGEMENT_IMG_SAMPLER_CONFIG_DOCK,
                                                     &ImageSampler::ConfigureServiceDockCam, this);
  enable_srv_[NAV_CAM_ID] = nh->advertiseService(SERVICE_MANAGEMENT_IMG_SAMPLER_ENABLE_NAV,
                                                     &ImageSampler::EnableServiceNavCam,  this);
  enable_srv_[DOCK_CAM_ID] = nh->advertiseService(SERVICE_MANAGEMENT_IMG_SAMPLER_ENABLE_DOCK,
                                                     &ImageSampler::EnableServiceDockCam,  this);
}

void ImageSampler::UpdateState(int camera, bool streaming, int width, int height, float rate) {
  if (streaming) {
    camera_states_[camera].stream_width = width;
    camera_states_[camera].stream_height = height;
    camera_states_[camera].stream_rate = rate;
  } else {
    camera_states_[camera].record_width = width;
    camera_states_[camera].record_height = height;
    camera_states_[camera].record_rate = rate;
  }
  ff_msgs::CameraStatesStamped state;
  state.header.stamp = ros::Time::now();
  state.states.resize(NUM_CAMERAS);
  state.states.assign(camera_states_, camera_states_ + NUM_CAMERAS);
  camera_state_pub_.publish(state);
}

bool ImageSampler::ConfigureServiceNavCam(ff_msgs::ConfigureCamera::Request& req,
                                          ff_msgs::ConfigureCamera::Response& res) {
  return ConfigureService(req, res, NAV_CAM_ID);
}

bool ImageSampler::ConfigureServiceDockCam(ff_msgs::ConfigureCamera::Request& req,
                                           ff_msgs::ConfigureCamera::Response& res) {
  return ConfigureService(req, res, DOCK_CAM_ID);
}

bool ImageSampler::ConfigureService(ff_msgs::ConfigureCamera::Request& req,
                                    ff_msgs::ConfigureCamera::Response& res,
                                    int camera) {
  assert(camera >= 0 && camera < NUM_CAMERAS);

  if (req.width <= 0 || req.height <= 0 || req.rate <= 0.0)
    return false;

  bool record = (req.mode == ff_msgs::EnableCamera::Request::RECORDING ||
      req.mode == ff_msgs::EnableCamera::Request::BOTH);
  bool stream = (req.mode == ff_msgs::EnableCamera::Request::STREAMING ||
      req.mode == ff_msgs::EnableCamera::Request::BOTH);

  if (stream)
    UpdateState(camera, true, req.width, req.height, req.rate);
  if (record)
    UpdateState(camera, false, req.width, req.height, req.rate);

  if (stream) {
    stream_publication_interval_[camera] = ros::Duration(1.0 / req.rate);
    stream_output_width_[camera] =  req.width;
    stream_output_height_[camera] = req.height;
  }
  if (record) {
    record_publication_interval_[camera] = ros::Duration(1.0 / req.rate);
    record_output_width_[camera] =  req.width;
    record_output_height_[camera] = req.height;
  }
  return true;
}

bool ImageSampler::EnableService(ff_msgs::EnableCamera::Request& req,
                                 ff_msgs::EnableCamera::Response& res, int camera, std::string topic,
                                    void (ImageSampler::*callback)(const sensor_msgs::ImageConstPtr &)) {
  bool record = (req.mode == ff_msgs::EnableCamera::Request::RECORDING ||
      req.mode == ff_msgs::EnableCamera::Request::BOTH);
  bool stream = (req.mode == ff_msgs::EnableCamera::Request::STREAMING ||
      req.mode == ff_msgs::EnableCamera::Request::BOTH);

  if (record)
    camera_states_[camera].recording = req.enable;
  if (stream)
    camera_states_[camera].streaming = req.enable;
  if (!camera_states_[camera].streaming && !camera_states_[camera].recording) {
    image_sub_[camera].shutdown();
  } else {
    // start the subscriber
    image_transport::ImageTransport img_transp(*GetPlatformHandle());
    image_sub_[camera] = img_transp.subscribe(topic, 1, callback, this);
  }

  ff_msgs::CameraStatesStamped state;
  state.header.stamp = ros::Time::now();
  state.states.resize(NUM_CAMERAS);
  state.states.assign(camera_states_, camera_states_ + NUM_CAMERAS);
  camera_state_pub_.publish(state);

  return true;
}

bool ImageSampler::EnableServiceNavCam(ff_msgs::EnableCamera::Request& req,
                                ff_msgs::EnableCamera::Response& res) {
  return EnableService(req, res, NAV_CAM_ID, TOPIC_HARDWARE_NAV_CAM, &ImageSampler::NavCamCallback);
}

bool ImageSampler::EnableServiceDockCam(ff_msgs::EnableCamera::Request& req,
                                 ff_msgs::EnableCamera::Response& res) {
  return EnableService(req, res, DOCK_CAM_ID, TOPIC_HARDWARE_DOCK_CAM, &ImageSampler::DockCamCallback);
}

void ImageSampler::NavCamCallback(const sensor_msgs::ImageConstPtr & msg) {
  ImageCallback(msg, NAV_CAM_ID);
}

void ImageSampler::DockCamCallback(const sensor_msgs::ImageConstPtr & msg) {
  ImageCallback(msg, DOCK_CAM_ID);
}

void ImageSampler::ImageCallback(const sensor_msgs::ImageConstPtr & msg, int camera) {
  assert(camera >= 0 && camera < NUM_CAMERAS);
  if (camera_states_[camera].recording &&
      msg->header.stamp - record_last_publish_time_[camera] >= record_publication_interval_[camera]) {
    record_last_publish_time_[camera] = msg->header.stamp;
    cv::Mat image_curr = cv_bridge::toCvShare(msg, msg->encoding)->image;

    cv::Mat rescaled_image;
    cv::resize(image_curr, rescaled_image, cv::Size(record_output_width_[camera], record_output_height_[camera]));

    sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding, rescaled_image).toImageMsg();
    sensor_msgs::CameraInfo cinfo;
    cinfo.header = msg->header;
    record_image_pub_[camera].publish(*out_img, cinfo);
  }
  if (camera_states_[camera].streaming &&
      msg->header.stamp - stream_last_publish_time_[camera] >= stream_publication_interval_[camera]) {
    stream_last_publish_time_[camera] = msg->header.stamp;
    cv::Mat image_curr = cv_bridge::toCvShare(msg, msg->encoding)->image;

    cv::Mat rescaled_image;
    cv::resize(image_curr, rescaled_image, cv::Size(stream_output_width_[camera], stream_output_height_[camera]));

    sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding, rescaled_image).toImageMsg();
    sensor_msgs::CameraInfo cinfo;
    cinfo.header = msg->header;
    stream_image_pub_[camera].publish(*out_img, cinfo);
  }
}

}  // namespace image_sampler

PLUGINLIB_EXPORT_CLASS(image_sampler::ImageSampler, nodelet::Nodelet)

