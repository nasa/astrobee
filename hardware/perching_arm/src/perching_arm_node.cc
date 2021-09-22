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

// Standard ROS includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>
#include <ff_util/ff_nodelet.h>

// Generic arm control messages
#include <sensor_msgs/JointState.h>

// Specific arm services
#include <ff_hw_msgs/SetEnabled.h>
#include <ff_hw_msgs/CalibrateGripper.h>
#include <ff_hw_msgs/SetJointMaxVelocity.h>

// interface class
#include <perching_arm/perching_arm.h>

/**
 * \ingroup hw
 */
namespace perching_arm {

class PerchingArmNode : public ff_util::FreeFlyerNodelet {
 public:
  PerchingArmNode() : ff_util::FreeFlyerNodelet(NODE_PERCHING_ARM) {}
  ~PerchingArmNode() { arm_.Disconnect(); }

 protected:
  // Called on flight software stack initialization - every NODELET_FATAIL
  // call below should be converted to an initialization fault...
  void Initialize(ros::NodeHandle *nh) {
     nh_ = *nh;
    // Read the configuration
    config_reader::ConfigReader config_params;
    config_params.AddFile("hw/perching_arm.config");
    if (!config_params.ReadFiles()) NODELET_FATAL("Couldn't read config file");

    // Read the device information from the config table
    config_reader::ConfigReader::Table devices;
    if (!config_params.GetTable("perching_arm", &devices))
      NODELET_FATAL("Could get perching_arm item in config file");

    // Reconnect to the arm service
    if (!initialized_) {
      srv_a_ =
          nh->advertiseService(SERVICE_HARDWARE_PERCHING_ARM_ENABLE,
                               &PerchingArmNode::EnableArmCallback,
                               this);
          initialized_ = true;
    }

    // Iterate over all devices
    for (int i = 0; i < devices.GetSize(); i++) {
      config_reader::ConfigReader::Table device_info;
      if (!devices.GetTable(i + 1, &device_info))
        NODELET_FATAL("Could get row in table table");

      // Get the name of the device
      std::string name;
      if (!device_info.GetStr("name", &name))
        NODELET_FATAL("Could not find row 'name' in table");

      // Check that the name matches the name of this node
      if (name == GetName()) {
        if (!device_info.GetStr("prefix", &prefix_))
          NODELET_FATAL("Could not read the prefix from the config");

        // Grab config parameters for the matched device
        config_reader::ConfigReader::Table serial;
        if (!device_info.GetTable("serial", &serial))
          NODELET_FATAL("Could not find table 'serial' in table");
        std::string port;
        if (!serial.GetStr("port", &port))
          NODELET_FATAL("Could not read the serial port from the config");
        uint32_t baud;
        if (!serial.GetUInt("baud", &baud))
          NODELET_FATAL("Could not read the serial baud from the config");

        // Setup some callbacks to handle events
        PerchingArmSleepMsCallback cb_sleep = std::bind(
            &PerchingArmNode::SleepCallback, this, std::placeholders::_1);
        PerchingArmRawDataCallback cb_data = std::bind(
            &PerchingArmNode::DataCallback, this, std::placeholders::_1);

        // Initialize the arm
        PerchingArmResult ret = arm_.Connect(port, baud, cb_sleep, cb_data);
        if (ret != RESULT_SUCCESS) {
          NODELET_WARN("Could not initialize the arm. It is attached?");
          return;
        }
        arm_connected_ = true;

        // Grab config parameters for the matched device
        config_reader::ConfigReader::Table config_list;
        if (!device_info.GetTable("config", &config_list))
          NODELET_FATAL("Could not find table 'config' in table");
        for (int j = 0; j < config_list.GetSize(); j++) {
          // Get a single configuration block
          config_reader::ConfigReader::Table config;
          if (!config_list.GetTable(j + 1, &config))
            NODELET_FATAL("Could not read row of transforms table");
          // Read the configuration
          int address, target, value;
          if (!config.GetInt("target", &target))
            NODELET_FATAL("Could not read config target");
          if (!config.GetInt("address", &address))
            NODELET_FATAL("Could not read config address");
          if (!config.GetInt("value", &value))
            NODELET_FATAL("Could not read config value");
          // Send the configuration
          if (RESULT_SUCCESS != arm_.SendCommand(static_cast<uint8_t>(target),
                                                 static_cast<int16_t>(address),
                                                 static_cast<int16_t>(value)))
            NODELET_FATAL_STREAM("Could not send configuration " << j);
        }

        // Force an enable of the gripper
        arm_.EnableGripper();

        // Create a joint state publisher for the arm, which matches the
        // structure expected by the robot_state publisher
        pub_ =
            nh->advertise<sensor_msgs::JointState>(TOPIC_JOINT_STATES, 1, true);

        // Call back with joint state goals from the high-level driver
        sub_ = nh->subscribe(TOPIC_JOINT_GOALS, 1,
                             &PerchingArmNode::GoalCallback, this);

        // Set the distal velocity
        srv_p_ =
            nh->advertiseService(SERVICE_HARDWARE_PERCHING_ARM_DIST_VEL,
                                 &PerchingArmNode::SetDistVelCallback, this);

        // Set the proximal velocity
        srv_t_ =
            nh->advertiseService(SERVICE_HARDWARE_PERCHING_ARM_PROX_VEL,
                                 &PerchingArmNode::SetProxVelCallback, this);

        // Enable/Disable the Proximal Joint Servo
        srv_ps_ =
            nh->advertiseService(SERVICE_HARDWARE_PERCHING_ARM_PROX_SERVO,
                                 &PerchingArmNode::EnableProximalServoCallback,
                                 this);
        // Enable/Disable the Distal Joint Servo
        srv_ds_ =
            nh->advertiseService(SERVICE_HARDWARE_PERCHING_ARM_DIST_SERVO,
                                 &PerchingArmNode::EnableDistalServoCallback,
                                 this);

        // Enable/Disable the Gripper Servo
        srv_gs_ =
            nh->advertiseService(SERVICE_HARDWARE_PERCHING_ARM_GRIP_SERVO,
                                 &PerchingArmNode::EnableGripperServoCallback,
                                 this);

        // Calibrate the arm
        srv_c_ =
            nh->advertiseService(SERVICE_HARDWARE_PERCHING_ARM_CALIBRATE,
                                 &PerchingArmNode::CalibrateGripperCallback,
                                this);

        // Exit once found
        return;
      }
    }
    // Catch all for device not found
    NODELET_ERROR("Device not found in config table!");
  }

  // CONVERSION UTILITIES

  // All possible conversion types
  enum ConversionType {
    POSITION,
    VELOCITY,
    PERCENTAGE,
    POWER,
    GRIPPER_L_PROX,
    GRIPPER_L_DIST,
    GRIPPER_R_PROX,
    GRIPPER_R_DIST
  };

  // Convert a value from the ROS convention to the firmware convention
  // - note that the firmware takes intermediary units, and not raw units
  int16_t CnvTo(ConversionType type, double value) {
    switch (type) {
      case POSITION:
        return static_cast<int16_t>(value * 57.2958);  // rads -> deg
      case VELOCITY:
        return static_cast<int16_t>(value * 9.5492);  // rads/sec -> rpm
      case PERCENTAGE:
        return static_cast<int16_t>(value);
      default:
        NODELET_WARN("Conversion to unknown data type");
        break;
    }
    return 0;
  }

  // Convert a value from the firmware convention to ROS convention
  // - note that the raw data must be scaled into intermediary units
  double CnvFrom(ConversionType type, int32_t value) {
    switch (type) {
      case POSITION:  // From raw format (0.1 degrees) to radians
        return static_cast<double>(value) / 10.0 * 0.0174533;
      case VELOCITY:  // From raw format (0.1 RPM) to radians / sec
        return static_cast<double>(value) / 10.0 * 0.10472;
      case POWER:  // From mA to power in watts
        return static_cast<double>(value) * PerchingArm::K_MOTOR_VOLTAGE /
               1000.0;
      case GRIPPER_L_PROX:
        return (PerchingArm::GRIP_L_P_MIN +
                static_cast<double>(value) / 100.0 *
                    (PerchingArm::GRIP_L_P_MAX - PerchingArm::GRIP_L_P_MIN)) /
               57.2958;
      case GRIPPER_L_DIST:
        return (PerchingArm::GRIP_L_D_MIN +
                static_cast<double>(value) / 100.0 *
                    (PerchingArm::GRIP_L_D_MAX - PerchingArm::GRIP_L_D_MIN)) /
               57.2958;
      case GRIPPER_R_PROX:
        return (PerchingArm::GRIP_R_P_MIN +
                static_cast<double>(value) / 100.0 *
                    (PerchingArm::GRIP_R_P_MAX - PerchingArm::GRIP_R_P_MIN)) /
               57.2958;
      case GRIPPER_R_DIST:
        return (PerchingArm::GRIP_R_D_MIN +
                static_cast<double>(value) / 100.0 *
                    (PerchingArm::GRIP_R_D_MAX - PerchingArm::GRIP_R_D_MIN)) /
               57.2958;
      default:
        NODELET_WARN("Conversion from unknown data type");
        break;
    }
    return 0;
  }

  // CALLBACK FUNCTIONS

  // ROS-aware blocking sleep, so that other nodes don't get blocked
  void SleepCallback(uint32_t ms) {
    ros::Duration(static_cast<double>(ms) / 1000.0).sleep();
  }

  // Asynchronous callback from the serial driver with feedback/result,
  // which we then push to the ROS messagign backbone as a joint state
  void DataCallback(PerchingArmRaw const &raw) {
    // Calculate the gripper percentage, but only if we have a maximum
    // value. ie. we have gone through the calibration procedure. If not
    // we must make the gripper joint value negative to indicate this!

    // IMPORTANT INFORMATION REGARDING UNITS
    // In a joint state message:
    // - position is in radians
    // - velocity is in radians per second
    // - power is in watts

    double perc = -100.0;
    if (raw.grip.maximum == -8000) {
      perc = static_cast<double>(raw.grip.position) /
             static_cast<double>(raw.grip.maximum) * 100.0;
      perc = 100 - perc;
    }

    // Allocate the joint state message
    msg_.name.resize(7);
    msg_.position.resize(7);
    msg_.velocity.resize(7);
    msg_.effort.resize(7);
    // Package all joint states, including the left and right proximal
    // and distal joints of the gripper (for visualization reasons)
    msg_.header.stamp = ros::Time::now();
    msg_.name[0] = prefix_ + "arm_proximal_joint";
    msg_.position[0] = CnvFrom(POSITION, raw.prox.position);
    msg_.velocity[0] = CnvFrom(VELOCITY, raw.prox.velocity);
    msg_.effort[0] = CnvFrom(POWER, raw.prox.load);
    msg_.name[1] = prefix_ + "arm_distal_joint";
    msg_.position[1] = CnvFrom(POSITION, raw.dist.position);
    msg_.velocity[1] = CnvFrom(VELOCITY, raw.dist.velocity);
    msg_.effort[1] = CnvFrom(POWER, raw.dist.load);
    // The joint state is in the range [0, 100] which would usually be
    // complete nonsense value if used in the context of ROS. Since our
    // URDF does not feature this joint, it will be quietly ignored. The
    // high-level driver will however pick it up and use the value.
    msg_.name[2] = prefix_ + "gripper_joint";
    msg_.position[2] = perc;
    msg_.velocity[2] = 0;
    msg_.effort[2] = CnvFrom(POWER, raw.grip.load);
    // Add some digit joints, which are driven directly from the gripper
    // joint status. We do this so that the RVIZ renders the full gripper
    // to the user, even though we don't actually know the digit joints.
    msg_.name[3] = prefix_ + "gripper_left_proximal_joint";
    msg_.position[3] = CnvFrom(GRIPPER_L_PROX, perc);
    msg_.velocity[3] = 0;
    msg_.effort[3] = 0;
    msg_.name[4] = prefix_ + "gripper_left_distal_joint";
    msg_.position[4] = CnvFrom(GRIPPER_L_DIST, perc);
    msg_.velocity[4] = 0;
    msg_.effort[4] = 0;
    msg_.name[5] = prefix_ + "gripper_right_proximal_joint";
    msg_.position[5] = CnvFrom(GRIPPER_R_PROX, perc);
    msg_.velocity[5] = 0;
    msg_.effort[5] = 0;
    msg_.name[6] = prefix_ + "gripper_right_distal_joint";
    msg_.position[6] = CnvFrom(GRIPPER_R_DIST, perc);
    msg_.velocity[6] = 0;
    msg_.effort[6] = 0;
    // Publish the message
    pub_.publish(msg_);
  }

  // Called whenever a new control command is available
  void GoalCallback(sensor_msgs::JointState const &msg) {
    for (size_t i = 0; i < msg.name.size(); i++) {
      if (msg.name[i] == prefix_ + "gripper_joint") {
        if (msg.position.size() > i)
          arm_.SetGripperPosition(CnvTo(PERCENTAGE, msg.position[i]));
        else
          NODELET_WARN("Gripper: only position control is supported");
        // Handle the proximal joint
      } else if (msg.name[i] == prefix_ + "arm_proximal_joint") {
        if (msg.position.size() > i)
          arm_.SetProximalPosition(CnvTo(POSITION, msg.position[i]));
        else
          NODELET_WARN("Proximal: only position control is supported");
        // Handle the distal joint
      } else if (msg.name[i] == prefix_ + "arm_distal_joint") {
        if (msg.position.size() > i)
          arm_.SetDistalPosition(CnvTo(POSITION, msg.position[i]));
        else
          NODELET_WARN("Distal: only position control is supported");
        // Catch all invalid joint states
      } else {
        NODELET_WARN_STREAM("Invalid joint: " << msg.name[i]);
      }
    }
  }

  // Enable arm
  bool EnableArmCallback(ff_hw_msgs::SetEnabled::Request &req,
                         ff_hw_msgs::SetEnabled::Response &res) {
    if (req.enabled && !arm_connected_) {
      Initialize(&nh_);
      res.success = arm_connected_;
    } else if (!req.enabled && arm_connected_) {
      res.success = false;
      NODELET_WARN("It is not possible to disable the arm");
    } else {
      res.success = true;
      NODELET_WARN("Already satisfies request");
    }
    return true;
  }

  // Set the distal velocity
  bool SetDistVelCallback(ff_hw_msgs::SetJointMaxVelocity::Request &req,
                          ff_hw_msgs::SetJointMaxVelocity::Response &res) {
    PerchingArmResult r = arm_.SetDistalVelocity(req.rpm);
    res.success = arm_.ResultToString(r, res.status_message);
    return true;
  }

  // Set the proximal velocity
  bool SetProxVelCallback(ff_hw_msgs::SetJointMaxVelocity::Request &req,
                          ff_hw_msgs::SetJointMaxVelocity::Response &res) {
    PerchingArmResult r = arm_.SetProximalVelocity(req.rpm);
    res.success = arm_.ResultToString(r, res.status_message);
    return true;
  }

  // Enable/Disable the proximal joint servo
  bool EnableProximalServoCallback(ff_hw_msgs::SetEnabled::Request &req,
                                   ff_hw_msgs::SetEnabled::Response &res) {
    // ROS_WARN("[Perching_arm] Enable/Disable the proximal joint servo callback");
    PerchingArmResult r;

    if (req.enabled)
      r = arm_.SetProximalEnabled();
    else
      r = arm_.SetProximalDisabled();
    res.success = arm_.ResultToString(r, res.status_message);
    return true;
  }

  // Enable/Disable the distal joint servo
  bool EnableDistalServoCallback(ff_hw_msgs::SetEnabled::Request &req,
                                 ff_hw_msgs::SetEnabled::Response &res) {
    // ROS_WARN("[Perching_arm] Enable/Disable the distal joint servo callback");
    PerchingArmResult r;

    if (req.enabled)
      r = arm_.SetDistalEnabled();
    else
      r = arm_.SetDistalDisabled();
    res.success = arm_.ResultToString(r, res.status_message);
    return true;
  }

  // Enable/Disable the gripper joint servo
  bool EnableGripperServoCallback(ff_hw_msgs::SetEnabled::Request &req,
                                  ff_hw_msgs::SetEnabled::Response &res) {
    // ROS_WARN("[Perching_arm] Enable/Disable the gripper joint servo callback");
    PerchingArmResult r;

    if (req.enabled)
      r = arm_.EnableGripper();
    else
      r = arm_.DisableGripper();
    res.success = arm_.ResultToString(r, res.status_message);
    return true;
  }

  // Calibrate the gripper
  bool CalibrateGripperCallback(ff_hw_msgs::CalibrateGripper::Request &req,
                                ff_hw_msgs::CalibrateGripper::Response &res) {
    // ROS_WARN("[Perching_arm] Calibrate Gripper Callback");
    PerchingArmResult r = arm_.CalibrateGripper();
    res.success = arm_.ResultToString(r, res.status_message);
    return true;
  }

 private:
  ros::NodeHandle nh_;
  bool initialized_ = false;
  bool arm_connected_ = false;
  PerchingArm arm_;              // Arm interface library
  ros::Subscriber sub_;          // Joint state subscriber
  ros::Publisher pub_;           // Joint state publisher
  ros::ServiceServer srv_a_;     // Enable the arm
  ros::ServiceServer srv_p_;     // Set max pan velocity
  ros::ServiceServer srv_t_;     // Set max tilt velocity
  ros::ServiceServer srv_ps_;    // Enable/Disable the proximal joint servo
  ros::ServiceServer srv_ds_;    // Enable/Disable the distal   joint servo
  ros::ServiceServer srv_gs_;    // Enable/Disable the gripper  joint servo
  ros::ServiceServer srv_c_;     // Calibrate gripper
  std::string prefix_;           // Joint prefix
  sensor_msgs::JointState msg_;  // Internal joint state
};

PLUGINLIB_EXPORT_CLASS(perching_arm::PerchingArmNode, nodelet::Nodelet);

}  // namespace perching_arm
