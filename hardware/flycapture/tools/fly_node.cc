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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ff_util/ff_names.h>
#include <cv_bridge/cv_bridge.h>

#include <flycapture/Camera.h>
#include <flycapture/Image.h>
#include <flycapture/Error.h>
#include <flycapture/BusManager.h>
#include <cv.h>

std::string DecoderRing(int code) {
  switch (code) {
    case FlyCapture2::VIDEOMODE_160x120YUV444:
     return "VIDEOMODE_160x120YUV444";
    case FlyCapture2::VIDEOMODE_320x240YUV422:
      return "VIDEOMODE_320x240YUV422";
    case FlyCapture2::VIDEOMODE_640x480YUV411:
      return "VIDEOMODE_640x480YUV411";
    case FlyCapture2::VIDEOMODE_640x480YUV422:
      return "VIDEOMODE_640x480YUV422";
    case FlyCapture2::VIDEOMODE_640x480RGB:
      return "VIDEOMODE_640x480RGB";
    case FlyCapture2::VIDEOMODE_640x480Y8:
      return "VIDEOMODE_640x480Y8";
    case FlyCapture2::VIDEOMODE_640x480Y16:
      return "VIDEOMODE_640x480Y16";
    case FlyCapture2::VIDEOMODE_800x600YUV422:
      return "VIDEOMODE_800x600YUV422";
    case FlyCapture2::VIDEOMODE_800x600RGB:
      return "VIDEOMODE_800x600RGB";
    case FlyCapture2::VIDEOMODE_800x600Y8:
      return "VIDEOMODE_800x600Y8";
    case FlyCapture2::VIDEOMODE_800x600Y16:
      return "VIDEOMODE_800x600Y16";
    case FlyCapture2::VIDEOMODE_1024x768YUV422:
      return "VIDEOMODE_1024x768YUV422";
    case FlyCapture2::VIDEOMODE_1024x768RGB:
      return "VIDEOMODE_1024x768RGB";
    case FlyCapture2::VIDEOMODE_1024x768Y8:
      return "VIDEOMODE_1024x768Y8";
    case FlyCapture2::VIDEOMODE_1024x768Y16:
      return "VIDEOMODE_1024x768Y16";
    case FlyCapture2::VIDEOMODE_1280x960YUV422:
      return "VIDEOMODE_1280x960YUV422";
    case FlyCapture2::VIDEOMODE_1280x960RGB:
      return "VIDEOMODE_1280x960RGB";
    case FlyCapture2::VIDEOMODE_1280x960Y8:
      return "VIDEOMODE_1280x960Y8";
    case FlyCapture2::VIDEOMODE_1280x960Y16:
      return "VIDEOMODE_1280x960Y16";
    case FlyCapture2::VIDEOMODE_1600x1200YUV422:
      return "VIDEOMODE_1600x1200YUV422";
    case FlyCapture2::VIDEOMODE_1600x1200RGB:
      return "VIDEOMODE_1600x1200RGB";
    case FlyCapture2::VIDEOMODE_1600x1200Y8:
     return "VIDEOMODE_1600x1200Y8";
    case FlyCapture2::VIDEOMODE_1600x1200Y16:
     return "VIDEOMODE_1600x1200Y16";
    case FlyCapture2::VIDEOMODE_FORMAT7:
     return "VIDEOMODE_FORMAT7";
    case FlyCapture2::NUM_VIDEOMODES:
      return "NUM_VIDEOMODES";
    case FlyCapture2::VIDEOMODE_FORCE_32BITS:
      return "VIDEOMODE_FORCE_32BITS";
    default:
      return "UNKNOWN";
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fly_node");
  ros::NodeHandle node_handle;

  // Let's find the point gray camera
  FlyCapture2::BusManager bus;
  if (bus.RescanBus() != FlyCapture2::PGRERROR_OK) {
    ROS_FATAL("Unable to scan flycapture bus.");
    ros::shutdown();
  }

  unsigned int number_of_cameras = 0;
  bus.GetNumOfCameras(&number_of_cameras);

  // Let's get the point gray camera
  FlyCapture2::PGRGuid guid;
  bus.GetCameraFromIndex(0, &guid);
  FlyCapture2::Camera* camera = new FlyCapture2::Camera();
  if (camera->Connect(&guid) != FlyCapture2::PGRERROR_OK) {
    ROS_FATAL("Unable to connect to overhead camera.");
    ros::shutdown();
  }

  // Set Resolution
  FlyCapture2::Format7Info info;
  bool supported;
  if (camera->GetFormat7Info(&info, &supported) !=
      FlyCapture2::PGRERROR_OK) {
    ROS_FATAL("Unable to FlyCapture2::GetFormat7Info");
    ros::shutdown();
  }
  camera->StopCapture();
  FlyCapture2::Format7ImageSettings settings;
  settings.width = 1280;
  settings.height = 960;
  settings.mode = FlyCapture2::MODE_0;
  settings.offsetX = (info.maxWidth - 1280)/2;
  settings.offsetY = (info.maxHeight - 960)/2;
  settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO8;
  bool valid;
  FlyCapture2::Format7PacketInfo pinfo;
  if (camera->ValidateFormat7Settings(&settings, &valid, &pinfo)
      != FlyCapture2::PGRERROR_OK) {
    ROS_FATAL("Unable to ValidateFormat7Settings");
    ros::shutdown();
  }
  if (!valid) {
    ROS_FATAL("Invalid requested camera image format");
    ros::shutdown();
  }
  unsigned int packet_size = pinfo.maxBytesPerPacket;
  if (camera->SetFormat7Configuration(&settings, packet_size) !=
      FlyCapture2::PGRERROR_OK) {
    ROS_FATAL("Unable to SetFormat7Configuration");
    ros::shutdown();
  }

  // Create a return frame
  IplImage* frame = cvCreateImage(cvSize(1280, 960), IPL_DEPTH_8U, 1);
  camera->StartCapture();

  // Let's create our image transport
  image_transport::ImageTransport it(node_handle);
  image_transport::Publisher pub = it.advertise(TOPIC_LOCALIZATION_OVERHEAD_IMAGE, 1);

  // Lets capture data
  FlyCapture2::Image image;
  ros::Rate loop_rate(30);
  uint32_t seq = 0;
  while (node_handle.ok()) {
    camera->RetrieveBuffer(&image);
    int64 length = 1280 * 960;
    memcpy(frame->imageData, image.GetData(), length);

    // I have a frame!
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                   "mono8", frame).toImageMsg();
    msg->header.stamp = ros::Time::now();
    msg->header.seq = seq++;
    pub.publish(msg);

    // Check thyself before thou wrecks thyself.
    ros::spinOnce();
    loop_rate.sleep();
  }
  delete camera;

  return 0;
}
