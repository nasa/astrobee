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

// Core ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Shared libraries
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>
#include <config_reader/config_reader.h>

// Royale SDK interface
#include <royale/CameraManager.hpp>
#include <royale/LensParameters.hpp>
#include <royale/Status.hpp>

// Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <ff_msgs/PicoflexxIntermediateData.h>

// C / C++ includes
#include <cstddef>
#include <string>

/**
 * \ingroup hardware
 */
namespace pico_driver {

// Base class from which all L1, L2, L3, L4 classes are derived...
class PicoDriver {
 public:
  // Constructor
  PicoDriver(royale::CameraManager & manager, std::string const& uuid, std::string const& use_case, uint32_t exposure)
    : device_(manager.createCamera(uuid.c_str())), exposure_(exposure), initialized_(false) {
    // Try and create the camera
    if (device_ == nullptr) {
      ROS_DEBUG_STREAM("Could not create the camera device");
      return;
    }
    // Initialize the camera
    if (device_->initialize() != royale::CameraStatus::SUCCESS) {
      ROS_DEBUG_STREAM("Could not initialize the camera device");
      return;
    }
    // Get the use cases supported by the camera and select the desired one
    royale::Vector<royale::String> use_cases;
    if (device_->getUseCases(use_cases) != royale::CameraStatus::SUCCESS) {
      ROS_DEBUG_STREAM("Could not find a list of use cases for the given camera");
      return;
    }
    // Try and select the use case
    royale::String use_case_sel = (use_case.empty() ? use_cases[0] : use_case.c_str());
    for (size_t i = 0; i < use_cases.size(); ++i) {
      if (use_cases[i] == use_case_sel && device_->setUseCase(use_case_sel) == royale::CameraStatus::SUCCESS) {
        initialized_ = true;
        break;
      }
    }
    // Select a default use case on failure
    if (!initialized_ && use_cases.size() > 0) {
      ROS_WARN_STREAM("Use case not supported. Switching to default use case.");
      device_->setUseCase(use_cases[0]);
      initialized_ = true;
    }

    // Print Factory Calibration
    royale::LensParameters lens;
    bool calibrated;
    device_->isCalibrated(calibrated);

    ROS_DEBUG_STREAM("Is Calibrated " << calibrated);

    if (calibrated) {
      device_->getLensParameters(lens);
      ROS_DEBUG_STREAM("Center: " << lens.principalPoint.first << ", " << lens.principalPoint.second);
      ROS_DEBUG_STREAM("Focal Length: " << lens.focalLength.first << ", " << lens.focalLength.second);
      ROS_DEBUG_STREAM("Tangential Distortion: " << lens.distortionTangential.first << ", " <<
                      lens.distortionTangential.second);
      ROS_DEBUG_STREAM("Radial Distortion: " << lens.distortionRadial[0] << ", " << lens.distortionRadial[1] << ", "
                    << lens.distortionRadial[2]);
    }
  }

  // Destructor
  virtual ~PicoDriver() {
    if (!Ready()) return;
    device_->stopCapture();
  }

  // Are we ready
  bool Ready() {
    return (initialized_ && device_ != nullptr);
  }

 protected:
  // Get the image plane width
  uint16_t GetWidth() {
    if (!Ready()) return 0;
    uint16_t val = 0;
    if (device_->getMaxSensorWidth(val) != royale::CameraStatus::SUCCESS)
      ROS_DEBUG_STREAM("Could not get the max sensor width");
    return val;
  }

  // Get the image plan height
  uint16_t GetHeight() {
    if (!Ready()) return 0;
    uint16_t val = 0;
    if (device_->getMaxSensorHeight(val) != royale::CameraStatus::SUCCESS)
      ROS_DEBUG_STREAM("Could not get the max sensor height");
    return val;
  }

  // Register or unregister an extended data listener
  bool Listener(royale::IDepthDataListener *listener = nullptr) {
    if (!Ready()) return 0;
    if (listener == nullptr)
      return (device_->unregisterDataListener() == royale::CameraStatus::SUCCESS);
    return (device_->registerDataListener(listener) == royale::CameraStatus::SUCCESS);
  }

  // Register or unregister an extended data listener
  bool ListenerExtended(royale::IExtendedDataListener *listener = nullptr) {
    if (!Ready()) return 0;
    if (listener == nullptr)
      return (device_->unregisterDataListenerExtended() == royale::CameraStatus::SUCCESS);
    return (device_->registerDataListenerExtended(listener) == royale::CameraStatus::SUCCESS);
  }

  // Register or unregister a depth image listener
  bool ListenerDepthImage(royale::IDepthImageListener *listener = nullptr) {
    if (!Ready()) return 0;
    if (listener == nullptr)
      return (device_->unregisterDepthImageListener() == royale::CameraStatus::SUCCESS);
    return (device_->registerDepthImageListener(listener) == royale::CameraStatus::SUCCESS);
  }

  // Turn the capture on or off
  void Power(bool on) {
    if (!Ready()) return;
    bool capturing;
    if (device_->isCapturing(capturing) != royale::CameraStatus::SUCCESS) {
      ROS_DEBUG_STREAM("Could not query the capture status");
      return;
    }
    // If we have subscribers and we aren't capturing, then we should start!
    if (on && !capturing) {
      if (device_->startCapture() != royale::CameraStatus::SUCCESS) {
        ROS_DEBUG_STREAM("Could not start capturing from the camera");
      } else {
        // Case 1: manual exposure
        if (exposure_ > 0) {
          royale::Pair<uint32_t, uint32_t> limits;
          if (device_->getExposureLimits(limits) != royale::CameraStatus::SUCCESS) {
            ROS_DEBUG_STREAM("Could not query the exposure limits");
            return;
          }
          if (exposure_ < limits.first || exposure_ > limits.second) {
            ROS_DEBUG_STREAM("Manual exposure outside of limits");
            return;
          }
          if (device_->setExposureMode(royale::ExposureMode::MANUAL) != royale::CameraStatus::SUCCESS
              ||     device_->setExposureTime(exposure_) != royale::CameraStatus::SUCCESS) {
            ROS_DEBUG_STREAM("Manual exposure cannot be set");
            return;
          }
          ROS_INFO_STREAM("Client connected. Switching to manual exposure mode with value " << exposure_);
        // Case 2 : Automatic exposure
        } else if (device_->setExposureMode(
          royale::ExposureMode::AUTOMATIC) != royale::CameraStatus::SUCCESS) {
          ROS_DEBUG_STREAM("Automatic exposure cannot be set");
          return;
        }
        ROS_INFO_STREAM("Client connected. Switching automatic exposure mode");
      }
    }
    // If we don't have subscribers and we aren't capturing, then we should stop!
    if (!on && capturing) {
      if (device_->stopCapture() != royale::CameraStatus::SUCCESS) {
        ROS_DEBUG_STREAM("Could not start capturing from the camera");
      }
    }
  }

 private:
  std::unique_ptr < royale::ICameraDevice > device_;
  uint32_t exposure_;
  bool initialized_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////

// L1 access to the pico driver
class PicoDriverL1 : public PicoDriver, public royale::IDepthDataListener, public royale::IDepthImageListener {
 public:
  // Constructor
  PicoDriverL1(royale::CameraManager & manager, std::string const& uuid, std::string const& use_case, uint32_t exposure,
    ros::NodeHandle *nh, std::string const& robot, std::string const& name, std::string const& topic)
      : PicoDriver(manager, uuid, use_case, exposure) {
    if (!Ready()) {
      ROS_DEBUG_STREAM("Device failed to initialize");
      return;
    }
    // Change the point cloud based on the frame id and sensor size
    cloud_ = sensor_msgs::PointCloud2();  // To keep the data vector contiguous.
    cloud_.header.frame_id = (robot.empty() ? name : robot + "/" + name);
    cloud_.width = this->GetWidth();
    cloud_.height = this->GetHeight();
    cloud_.is_bigendian = false;
    cloud_.is_dense = true;
    cloud_.point_step = sizeof(struct royale::DepthPoint);
    cloud_.row_step = cloud_.width * cloud_.point_step;
    cloud_.data.resize(cloud_.row_step * cloud_.height);
    // X, Y and Z
    sensor_msgs::PointField field;
    field.name = "x";
    field.offset = offsetof(struct royale::DepthPoint, x);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes!
    cloud_.fields.push_back(field);
    field.name = "y";
    field.offset = offsetof(struct royale::DepthPoint, y);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes!
    cloud_.fields.push_back(field);
    field.name = "z";
    field.offset = offsetof(struct royale::DepthPoint, z);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes!
    cloud_.fields.push_back(field);
    // Generate a nice readable name for the camera
    std::string topic_name_c = (std::string) TOPIC_HARDWARE_PICOFLEXX_PREFIX
                             + (std::string) topic
                             + (std::string) TOPIC_HARDWARE_PICOFLEXX_SUFFIX;
    pub_cloud_ = nh->advertise<sensor_msgs::PointCloud2>(topic_name_c, 1,
      boost::bind(&PicoDriverL1::ToggleCamera, this),
      boost::bind(&PicoDriverL1::ToggleCamera, this));
    // Register the data listener
    if (!this->Listener(this))
      ROS_WARN_STREAM("Could not register the data listener");
    // Set up depth image
    depth_image_ = sensor_msgs::Image();
    depth_image_.header.frame_id = (robot.empty() ? name : robot + "/" + name);
    depth_image_.width = this->GetWidth();
    depth_image_.height = this->GetHeight();
    depth_image_.is_bigendian = false;
    depth_image_.encoding = sensor_msgs::image_encodings::MONO16;
    depth_image_.step = depth_image_.width * sizeof(uint16_t);
    depth_image_.data.resize(depth_image_.height * depth_image_.step);
    std::string topic_name_d = (std::string) TOPIC_HARDWARE_PICOFLEXX_PREFIX
                             + (std::string) topic
                             + (std::string) TOPIC_HARDWARE_PICOFLEXX_SUFFIX_DEPTH_IMAGE;
    pub_depth_image_ = nh->advertise<sensor_msgs::Image>(topic_name_d, 1,
      boost::bind(&PicoDriverL1::ToggleCamera, this),
      boost::bind(&PicoDriverL1::ToggleCamera, this));
    // Register the depth image listener
    if (!this->ListenerDepthImage(this))
     ROS_WARN_STREAM("Could not register the depth image listener");
  }

  // Destructor
  ~PicoDriverL1() {
    this->Listener(nullptr);
  }

 protected:
  // Turn camera on or off based on topic subscription
  void ToggleCamera() {
    if (!Ready()) return;
    if (pub_cloud_.getNumSubscribers() > 0 || pub_depth_image_.getNumSubscribers() > 0)
      Power(true);
    else if (pub_cloud_.getNumSubscribers() == 0 && pub_depth_image_.getNumSubscribers() == 0)
      Power(false);
  }

  // Callback for new data arrival
  void onNewData(const royale::DepthData *data) {
    if (!Ready()) return;
    if (data == nullptr) {
      ROS_WARN("data pointer = nullptr");
      return;
    }
    if (data->points.size() != cloud_.width * cloud_.height) {
      ROS_WARN("data size incorrect");
      return;
    }
    // If we have depth data, use the same mechanism as L1 to push it
    if (pub_cloud_.getNumSubscribers() > 0) {
      cloud_.header.stamp.fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(data->timeStamp).count());
      std::copy(
        reinterpret_cast<const uint8_t*>(data->points.data()),
        reinterpret_cast<const uint8_t*>(data->points.data()) + cloud_.row_step * cloud_.height,
        cloud_.data.begin());
      pub_cloud_.publish(cloud_);
    }
  }

  // Callback for new depth image arrival
  void onNewData(const royale::DepthImage *data) {
    if (!Ready()) return;
    if (data == nullptr) {
      ROS_WARN("data pointer = nullptr");
      return;
    }
    if (data->data.size() != depth_image_.height * depth_image_.width) {
      ROS_WARN("data size incorrect");
      return;
    }
    // If we have depth data, use the same mechanism as L1 to push it
    if (pub_depth_image_.getNumSubscribers() > 0) {
      // units not documented in DepthImage.hpp, maybe usecs like DepthData?
      uint64_t stampUsecs = data->timestamp;
      depth_image_.header.stamp.fromNSec(stampUsecs * 1000);
      std::copy(
        reinterpret_cast<const uint8_t*>(data->data.data()),
        reinterpret_cast<const uint8_t*>(data->data.data()) + depth_image_.height * depth_image_.step,
        depth_image_.data.begin());
      pub_depth_image_.publish(depth_image_);
    }
  }

 private:
  sensor_msgs::PointCloud2 cloud_;                        // The point cloud
  ff_msgs::PicoflexxIntermediateData extended_;           // The extended data
  sensor_msgs::Image depth_image_;                        // The depth image
  ros::Publisher pub_cloud_;                              // The point cloud publisher
  ros::Publisher pub_depth_image_;                        // The depth image publisher
};

////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef std::pair < uint32_t, uint32_t > PFIndex;

// L2 access to the pico driver
class PicoDriverL2 : public PicoDriver, public royale::IExtendedDataListener {
 public:
  // Constructor
  PicoDriverL2(royale::CameraManager & manager, std::string const& uuid, std::string const& use_case, uint32_t exposure,
    ros::NodeHandle *nh, std::string const& robot, std::string const& name, std::string const& topic)
      : PicoDriver(manager, uuid, use_case, exposure) {
    if (!Ready()) {
      ROS_DEBUG_STREAM("Device failed to initialize");
      return;
    }
    // Setup the extended
    extended_.header.frame_id = (robot.empty() ? name : robot + "/" + name);
    // Generate a nice readable name for the camera
    std::string topic_name_e = (std::string) TOPIC_HARDWARE_PICOFLEXX_PREFIX
                             + (std::string) topic
                             + (std::string) TOPIC_HARDWARE_PICOFLEXX_SUFFIX_EXTENDED;
    pub_extended_ = nh->advertise<ff_msgs::PicoflexxIntermediateData>(topic_name_e, 1,
      boost::bind(&PicoDriverL2::ToggleCamera, this),
      boost::bind(&PicoDriverL2::ToggleCamera, this));
    // Change the point cloud based on the frame id and sensor size
    cloud_ = sensor_msgs::PointCloud2();  // To keep the data vector contiguous.
    cloud_.header.frame_id = (robot.empty() ? name : robot + "/" + name);
    cloud_.width = this->GetWidth();
    cloud_.height = this->GetHeight();
    cloud_.is_bigendian = false;
    cloud_.is_dense = true;
    cloud_.point_step = sizeof(struct royale::DepthPoint);
    cloud_.row_step = cloud_.width * cloud_.point_step;
    cloud_.data.resize(cloud_.row_step * cloud_.height);
    // X, Y and Z
    sensor_msgs::PointField field;
    field.name = "x";
    field.offset = offsetof(struct royale::DepthPoint, x);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes.
    cloud_.fields.push_back(field);
    field.name = "y";
    field.offset = offsetof(struct royale::DepthPoint, y);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes.
    cloud_.fields.push_back(field);
    field.name = "z";
    field.offset = offsetof(struct royale::DepthPoint, z);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;  // Number of ELEMENTS, not bytes.
    cloud_.fields.push_back(field);
    // Generate a nice readable name for the camera
    std::string topic_name_c = (std::string) TOPIC_HARDWARE_PICOFLEXX_PREFIX
                             + (std::string) topic
                             + (std::string) TOPIC_HARDWARE_PICOFLEXX_SUFFIX;
    pub_cloud_ = nh->advertise<sensor_msgs::PointCloud2>(topic_name_c, 1,
      boost::bind(&PicoDriverL2::ToggleCamera, this),
      boost::bind(&PicoDriverL2::ToggleCamera, this));
    // Register the data listener
    if (!this->ListenerExtended(this))
      ROS_WARN_STREAM("Could not register the extended data listener");
  }

  // Destructor
  ~PicoDriverL2() {
    this->ListenerExtended(nullptr);
  }

 protected:
  // Turn camera on or off based on topic subscription
  void ToggleCamera() {
    if (!Ready()) return;
    if ((pub_extended_.getNumSubscribers() > 0 || pub_cloud_.getNumSubscribers() > 0))
      Power(true);
    else if (pub_extended_.getNumSubscribers() == 0 && pub_cloud_.getNumSubscribers() == 0)
      Power(false);
  }

  // Callback for new data
  void onNewData(const royale::IExtendedData *data)  {
    if (!Ready()) return;
    if (data == nullptr) {
      ROS_WARN("data pointer = nullptr");
      return;
    }

    ros::Time commonStamp(0, 0);
    if (data->hasDepthData() && data->getDepthData() != nullptr) {
      commonStamp.fromNSec(
        std::chrono::duration_cast<std::chrono::nanoseconds>(data->getDepthData()->timeStamp).count());
    } else if (data->hasIntermediateData()
               && data->getIntermediateData() != nullptr) {
      commonStamp.fromNSec(
        std::chrono::duration_cast<std::chrono::nanoseconds>(data->getIntermediateData()->timeStamp).count());
    }

    // If we have depth data, use the same mechanism as L1 to push it
    if (data->hasDepthData() && pub_cloud_.getNumSubscribers() > 0
      && data->getDepthData() != nullptr) {
      cloud_.header.stamp = commonStamp;
       std::copy(
          reinterpret_cast<const uint8_t*>(data->getDepthData()->points.data()),
          reinterpret_cast<const uint8_t*>(data->getDepthData()->points.data()) + cloud_.row_step * cloud_.height,
          cloud_.data.begin());
      pub_cloud_.publish(cloud_);
    }
    // If we have a listener and the extended data contains intermediate data, publish it
    if (data->hasIntermediateData() && pub_extended_.getNumSubscribers() > 0
        && data->getIntermediateData() != nullptr) {
      extended_.header.stamp = commonStamp;
      // Populate the modulation frequencies and exposures used to produce this data
      extended_.frequency.resize(data->getIntermediateData()->modulationFrequencies.size());
      for (size_t i = 0; i < data->getIntermediateData()->modulationFrequencies.size(); i++)
        extended_.frequency[i] = data->getIntermediateData()->modulationFrequencies[i];
      extended_.exposure.resize(data->getIntermediateData()->exposureTimes.size());
      for (size_t i = 0; i < data->getIntermediateData()->exposureTimes.size(); i++)
        extended_.exposure[i] = data->getIntermediateData()->exposureTimes[i];
      // Copy the data itself
      extended_.raw.width = this->GetWidth();
      extended_.raw.height = this->GetHeight();
      extended_.raw.step = extended_.raw.width
        * sizeof(struct royale::IntermediatePoint);
      extended_.raw.encoding = sensor_msgs::image_encodings::TYPE_32FC4;
      extended_.raw.is_bigendian = false;
      extended_.raw.data.resize(extended_.raw.step * extended_.raw.height);
      std::copy(
        reinterpret_cast<const uint8_t*>(data->getIntermediateData()->points.data()),
        reinterpret_cast<const uint8_t*>(data->getIntermediateData()->points.data())
          + extended_.raw.step * extended_.raw.height,
        extended_.raw.data.begin());
      // Publish the extended data
      pub_extended_.publish(extended_);
    }
  }

 private:
  sensor_msgs::PointCloud2 cloud_;                     // The point cloud
  ff_msgs::PicoflexxIntermediateData extended_;        // The extended data
  ros::Publisher pub_extended_;                        // The cloud publisher
  ros::Publisher pub_cloud_;                           // The cloud publisher
};

////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef std::map < std::string, std::shared_ptr < PicoDriver > > PicoDeviceList;

// Factor
class PicoFactory {
 public:
  // Constructor
  PicoFactory(ros::NodeHandle *nh, std::string const& robot, std::string const& api_key)
    : nh_(nh), robot_(robot), manager_(api_key.c_str()), initialized_(false) {
    // Get the camera level
    level_ = manager_.getAccessLevel(api_key.c_str());
    switch (level_) {
    case royale::CameraAccessLevel::L4: ROS_DEBUG("Using API level L4"); break;
    case royale::CameraAccessLevel::L3: ROS_DEBUG("Using API level L3"); break;
    case royale::CameraAccessLevel::L2: ROS_DEBUG("Using API level L2"); break;
    case royale::CameraAccessLevel::L1: ROS_DEBUG("Using API level L1"); break;
    default:
      break;
    }
    // Print camera list
    royale::Vector<royale::String> cam_list = manager_.getConnectedCameraList();
    if (cam_list.empty())
      ROS_ERROR_STREAM("Could not find any cameras. Is the udev rule? In the plugdev group? Is the camera connected?");
    ROS_DEBUG_STREAM("Probed USB bus and found " << cam_list.size() << " cameras:");
    for (size_t i = 0; i < cam_list.size(); ++i)
      ROS_DEBUG_STREAM("- " << cam_list[i]);
    // We can now initialize
    initialized_ =  true;
  }

  // Destructor - make sure we clear up all pico devices
  ~PicoFactory() {}

  // Add a new device
  std::shared_ptr < PicoDriver > AddCamera(std::string const& uuid,       // Camera identifier
                                           std::string const& use_case,   // Camera use case
                                           uint32_t exposure,             // Camera exposure
                                           std::string const& name,       // Camera name
                                           std::string const& topic) {    // Camera topic
    // We can't add a camera
    if (!initialized_)
      return nullptr;
    // Create a new handle to the picoflexx camera based on the API level
    std::shared_ptr < PicoDriver > ptr;
    switch (level_) {
    case royale::CameraAccessLevel::L4:
    case royale::CameraAccessLevel::L3:
    case royale::CameraAccessLevel::L2:
      ptr = std::make_shared < PicoDriverL2 > (
        manager_, uuid, use_case, exposure, nh_, robot_, name, topic);
      break;
    case royale::CameraAccessLevel::L1:
    default:
      ptr = std::make_shared < PicoDriverL1 > (
        manager_, uuid, use_case, exposure, nh_, robot_, name, topic);
      break;
    }
    // If the device was allocated, save it
    if (ptr->Ready())
      return ptr;
    // This didn't work :()
    return nullptr;
  }

 private:
  ros::NodeHandle* nh_;                // Node Handle
  std::string robot_;                  // Robot name
  royale::CameraManager manager_;      // Camera manager
  royale::CameraAccessLevel level_;    // API level
  bool initialized_;                   // Are we initialized?
};

class PicoDriverNodelet : public ff_util::FreeFlyerNodelet  {
 public:
  PicoDriverNodelet() : ff_util::FreeFlyerNodelet() {}
  ~PicoDriverNodelet() {}

 protected:
  void Initialize(ros::NodeHandle *nh) {
    // Read the config file
    config_reader::ConfigReader config_params;
    config_params.AddFile("cameras.config");
    if (!config_params.ReadFiles())
      return InitFault("Lua: Could get read the config file");
    // Try and open the config file
    config_reader::ConfigReader::Table pconfig;
    if (!config_params.GetTable("picoflexx", &pconfig))
      return InitFault("Lua:Could not find the picoflexx table");
    // Initialize the pico factory class with an API key
    std::string api_key;
    if (!pconfig.GetStr("api_key", &api_key))
      return InitFault("Lua:Could get devices item in config file");
    PicoFactory factory(nh, GetPlatform(), api_key);
    // Read the device information from the config table
    config_reader::ConfigReader::Table devices;
    if (!pconfig.GetTable("devices", &devices))
      return InitFault("Lua:Could get devices item in config file");
    for (int i = 0; i < devices.GetSize(); i++) {
      // Get the device info
      config_reader::ConfigReader::Table device_info;
      if (!devices.GetTable(i + 1, &device_info))
        return InitFault("Lua:Could get row in table table");
      // Get the parameters
      std::string device_name, mode, name, topic;
      uint32_t exposure;
      bool required = true;
      if (!device_info.GetStr("name", &name))
        return InitFault("Lua:Could not find row 'name' in table");
      // Query all information about the camer
      if (!device_info.GetStr("device", &device_name))
        return InitFault("Lua:Could not find row 'device' in table");
      // Query all information about the camer
      if (!device_info.GetStr("topic", &topic))
        return InitFault("Lua:Could not find row 'topic' in table");
      // Get the use case
      if (!device_info.GetStr("mode", &mode))
        return InitFault("Lua:Could not find row 'mode' in table");
      // Get the exposure
      if (!device_info.GetUInt("exposure", &exposure))
        return InitFault("Lua:Could not find row 'exposure' in table");
      // Is the camera required (assume true for backward compatibility)
      if (!device_info.GetBool("required", &required))
        return InitFault("Lua:Could not find row 'required' in table");
      // Try and create the pico device, and optionally reqport an error
      std::shared_ptr < PicoDriver > ptr = factory.AddCamera(
        device_name, mode, exposure, name, topic);
      if (ptr != nullptr)
        devices_[device_name] = ptr;
      else if (required)
        InitFault("Lua:Could not start camera " + device_name);
    }
  }

  // Deal with a fault in a responsible manner
  void InitFault(std::string const& msg ) {
    NODELET_ERROR_STREAM(msg);
    AssertFault(ff_util::INITIALIZATION_FAILED, msg);
    return;
  }

 private:
  PicoDeviceList devices_;             // Devices
};

PLUGINLIB_EXPORT_CLASS(pico_driver::PicoDriverNodelet, nodelet::Nodelet);

}  // namespace pico_driver
