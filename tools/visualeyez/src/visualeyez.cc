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
#include <visualeyez/visualeyez.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * \ingroup tools
 */
namespace visualeyez {

  // Helper function to see if an LED value is valid.
  bool VisualeyezUtils::IsValidMarkerValue(geometry_msgs::Vector3 const& pt) {
    if (pt.x == 0.0 || pt.y == 0.0 || pt.z == 0.0)
      return false;
    return true;
  }

  // Helper function for grabbing marker information from a LUA config file
  VZMarkers VisualeyezUtils::GetMarkers(config_reader::ConfigReader::Table *table) {
    VZMarkers markers;
    for (int i = 0; i < table->GetSize(); i++) {
      // Grab the row in the table
      config_reader::ConfigReader::Table group;
      if (!table->GetTable(i + 1, &group)) {
        ROS_ERROR_STREAM("Could not read parameter table row" << i + 1);
        continue;
      }
      // Extract values from the table
      int ledid, tcmid;
      double x, y, z;
      if (!group.GetInt("tcm", &tcmid)) {
        ROS_ERROR_STREAM("Could not read parameter: tcm");
        continue;
      }
      if (!group.GetInt("led", &ledid)) {
        ROS_ERROR_STREAM("Could not read parameter: led");
        continue;
      }
      if (!group.GetReal("x", &x)) {
        ROS_ERROR_STREAM("Could not read parameter: x");
        continue;
      }
      if (!group.GetReal("y", &y)) {
        ROS_ERROR_STREAM("Could not read parameter: y");
        continue;
      }
      if (!group.GetReal("z", &z)) {
        ROS_ERROR_STREAM("Could not read parameter: z");
        continue;
      }
      // Generate a map index from the tcm and led id number
      VZIndex idx(tcmid, ledid);
      // If all parameters were grabbed, add the row
      markers[idx](0) = x;
      markers[idx](1) = y;
      markers[idx](2) = z;
    }
    return markers;
  }

  // Write calibration data to a binary file for a given set of targets
  bool VisualeyezUtils::WriteConfig(std::string file_name, VZTargetMarkers & targets) {
    // Convert the incoming targets to a calibration message
    ff_msgs::VisualeyezCalibration msg;
    for (VZTargetMarkers::iterator it = targets.begin(); it != targets.end(); it++) {
      ff_msgs::VisualeyezDataArray target;
      target.header.stamp = ros::Time::now();
      target.header.frame_id = it->first;
      for (VZMarkers::iterator jt = it->second.begin(); jt != it->second.end(); jt++) {
        ff_msgs::VisualeyezData measurement;
        measurement.tcmid = jt->first.first;      // TCMID
        measurement.ledid = jt->first.second;     // LEDID
        measurement.position.x = jt->second(0);   // X POS
        measurement.position.y = jt->second(1);   // Y POS
        measurement.position.z = jt->second(2);   // Z POS
        target.measurements.push_back(measurement);
      }
      msg.targets.push_back(target);
    }
    // Serialize the calibration data to file
    std::ofstream ofs(file_name, std::ios::out | std::ios::binary);
    if (!ofs.is_open()) {
      ROS_INFO_STREAM("Cannot write to file " << file_name);
      return false;
    }
    uint32_t serial_size = ros::serialization::serializationLength(msg);
    boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
    ros::serialization::OStream ostream(obuffer.get(), serial_size);
    ros::serialization::serialize(ostream, msg);
    ofs.write(reinterpret_cast<char*>(obuffer.get()), serial_size);
    ofs.close();
    // Success!
    return true;
  }

  // Read calibration data from a binary file for a given set of targets
  bool VisualeyezUtils::ReadConfig(std::string file_name, VZTargetMarkers & targets) {
    // Convert the calibration file into a calibration message
    ff_msgs::VisualeyezCalibration msg;
    std::ifstream ifs(file_name, std::ios::in | std::ios::binary);
    if (!ifs.good()) {
      ROS_INFO_STREAM("Cannot read from file " << file_name);
      return false;
    }
    ifs.seekg(0, std::ios::end);
    std::streampos end = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    std::streampos begin = ifs.tellg();
    uint32_t file_size = end - begin;
    boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
    ifs.read(reinterpret_cast<char*>(ibuffer.get()), file_size);
    ros::serialization::IStream istream(ibuffer.get(), file_size);
    ros::serialization::deserialize(istream, msg);
    ifs.close();
    // Push all the data into a temporary data structure
    VZTargetMarkers tmp;
    for (size_t i = 0; i < msg.targets.size(); i++) {
      std::string name = msg.targets[i].header.frame_id;
      for (size_t j = 0; j < msg.targets[i].measurements.size(); j++) {
        VZIndex index(msg.targets[i].measurements[j].tcmid,                 // TCMID
          msg.targets[i].measurements[j].ledid);                            // LEDID
        tmp[name][index](0) = msg.targets[i].measurements[j].position.x;    // X POS
        tmp[name][index](1) = msg.targets[i].measurements[j].position.y;    // Y POS
        tmp[name][index](2) = msg.targets[i].measurements[j].position.z;    // Z POS
      }
    }
    // Now check that we have a record for each input
    for (VZTargetMarkers::iterator it = targets.begin(); it != targets.end(); it++) {
      if (tmp.find(it->first) == tmp.end()) {
        ROS_WARN_STREAM("Cannot locate target " << it->first);
        return false;
      }
      targets[it->first] = tmp[it->first];
    }
    // Success!
    return true;
  }

  VisualeyezClient::VisualeyezClient() : socket_(io_service_) {
    // Extract the port from the config file
    std::string host;
    unsigned int port;
    config_reader::ConfigReader config_params;
    config_params.AddFile("tools/visualeyez.config");
    if (!config_params.ReadFiles())
      ROS_FATAL("Couldn't read config file");
    if (!config_params.GetUInt("client_port", &port))
      ROS_FATAL("Server port not specified in config file.");
    if (!config_params.GetStr("client_host", &host))
      ROS_FATAL("Server address not specified in config file.");
    // Open the socket
    socket_ = udp::socket(io_service_, udp::endpoint(udp::v4(), 0));
    // Bind the socket to a specific endpoint
    udp::resolver resolver(io_service_);
    udp::resolver::query query(
      udp::v4(), host, std::to_string(port));
    udp::resolver::iterator iterator = resolver.resolve(query);
    // Use the first remote endpoint ad the one we want
    remote_endpoint_ = *iterator;
    // Start a listen thread
    thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
  }

  // Destructor
  VisualeyezClient::~VisualeyezClient() {}

  // Sends a single message
  void VisualeyezClient::Send(ff_msgs::VisualeyezDataArray const& msg) {
    // Avoid a buffer overrun
    if (msg.measurements.size() > MAX_NUM_PACKETS || msg.measurements.empty()) {
      ROS_WARN("Cannot process more than MAX_NUM_PACKETS at once");
      return;
    }
    // Prepare the header
    tx_buffer_[HEADER_DATE] = msg.header.stamp.toSec();
    // Add the data
    for (size_t i = 0; i < msg.measurements.size(); i++) {
      tx_buffer_[HEADER_LEN+i*PACKET_LEN+PACKET_TCMID] = msg.measurements[i].tcmid;
      tx_buffer_[HEADER_LEN+i*PACKET_LEN+PACKET_LEDID] = msg.measurements[i].ledid;
      tx_buffer_[HEADER_LEN+i*PACKET_LEN+PACKET_POS_X] = msg.measurements[i].position.x;
      tx_buffer_[HEADER_LEN+i*PACKET_LEN+PACKET_POS_Y] = msg.measurements[i].position.y;
      tx_buffer_[HEADER_LEN+i*PACKET_LEN+PACKET_POS_Z] = msg.measurements[i].position.z;
    }
    // Calculate the final length
    size_t len = HEADER_LEN + msg.measurements.size() * PACKET_LEN;
    // Send the message
    socket_.async_send_to(boost::asio::buffer(tx_buffer_, sizeof(double) * len), remote_endpoint_,
        boost::bind(&VisualeyezClient::SendCallback, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

  // Called when messahe is sent
  void VisualeyezClient::SendCallback(const boost::system::error_code& error,
    std::size_t bytes_transferred) {
    /* Do nothing */
  }

}  // namespace visualeyez
