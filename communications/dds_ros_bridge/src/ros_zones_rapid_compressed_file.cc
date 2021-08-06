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

#include "dds_ros_bridge/ros_zones_rapid_compressed_file.h"

ff::RosZonesToRapidCompressedFile::RosZonesToRapidCompressedFile(
                                            const std::string& subscribe_topic,
                                            const std::string& get_zones_srv,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle &nh,
                                            const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  file_supplier_.reset(
    new ff::RosZonesToRapidCompressedFile::Supplier(
      rapid::ext::astrobee::COMPRESSED_FILE_TOPIC + pub_topic,
      "", "AstrobeeCurrentCompressedPlanProfile", ""));

  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosZonesToRapidCompressedFile::Callback,
                       this);

  get_zones_client_ = nh_.serviceClient<ff_msgs::GetZones>(get_zones_srv);

  rapid::RapidHelper::initHeader(file_supplier_->event().hdr);
}

void ff::RosZonesToRapidCompressedFile::Callback(
                      visualization_msgs::MarkerArray::ConstPtr const& zones) {
  // The zones marker array seems to only be used for visualization and the data
  // in the message isn't in the right format to send to the ground. However the
  // message tells the bridge that the zones were loaded/changed and that it
  // needs to query the choreographer for the zones.
  ff_msgs::GetZones get_zones_srv;

  // Check to make sure the get zones service is running. If it isn't, all we
  // can do is output the error.
  if (!get_zones_client_.exists()) {
    ROS_ERROR("Get zones service isn't running! Choreographer may have died!");
    return;
  }

  // Check to make sure the service call worked
  if (!get_zones_client_.call(get_zones_srv)) {
    ROS_ERROR("Get zones service call returned false.");
    return;
  }

  // Sort zones first. The zones file that comes from GDS can contain multiple
  // sets of keepins and keepouts and each keepin and keepout can contain
  // multiple boxes. Thus we need to find all the keepins and keepouts that have
  // the same name and we need to sort the boxes by their index. Since this code
  // will run very infrequently, I'm okay with the sort taking time.
  std::map<std::string, std::vector<ff_msgs::Zone>> sorted_zones;
  std::vector<ff_msgs::Zone>::iterator zone_it, zone_begin, zone_end, sort_it;

  zone_begin = get_zones_srv.response.zones.begin();
  zone_end = get_zones_srv.response.zones.end();
  for (zone_it = zone_begin; zone_it != zone_end; zone_it++) {
    // Check to see if the zone is already in the map
    if (sorted_zones.count(zone_it->name) > 0) {
      sort_it = sorted_zones[zone_it->name].begin();
      // Use box index to insert the zone into the correct location in the
      // vector
      sorted_zones[zone_it->name].insert((sort_it + zone_it->index), *zone_it);
    } else {
      sorted_zones[zone_it->name].push_back(*zone_it);
    }
  }

  // Convert zones to json string/file
  std::string zones_file_content = "{\n";
  
  // Put timestamp in file content
  // Convert timestamp from seconds to milliseconds
  uint64_t time_milli = (uint64_t)get_zones_srv.response.timestamp.sec * 1000;
  time_milli += (uint64_t)get_zones_srv.response.timestamp.nsec / 1000000;
  zones_file_content += "\t\"timestamp\" : ";
  zones_file_content += std::to_string(time_milli);
  zones_file_content += "\",\n";
  zones_file_content += "\t\"zones\" : [";

  std::map<std::string, std::vector<ff_msgs::Zone>>::iterator map_it;
  for (map_it = sorted_zones.begin(); map_it != sorted_zones.end(); map_it++) {
    zones_file_content += " {\n\t\t\"sequence\": [";
    for (sort_it = map_it->second.begin();
         sort_it != map_it->second.end();
         sort_it++) {
      zones_file_content += " [ ";
      zones_file_content += std::to_string(sort_it->min.x) + ", ";
      zones_file_content += std::to_string(sort_it->min.y) + ", ";
      zones_file_content += std::to_string(sort_it->min.z) + ", ";
      zones_file_content += std::to_string(sort_it->max.x) + ", ";
      zones_file_content += std::to_string(sort_it->max.y) + ", ";
      zones_file_content += std::to_string(sort_it->max.z);
      zones_file_content += " ],";
    }

    // Remove comma
    zones_file_content.pop_back();
    zones_file_content += " ], \n\t\t\"safe\" : ";
    // The type and name will be the same for all zones so we can just use the
    // first one. Also the type clutter is never used in the GDS zones so I
    // don't think it is being used but we will mark it as a keepout just in
    // case it is
    if (map_it->second.begin()->type == ff_msgs::Zone::KEEPOUT ||
        map_it->second.begin()->type == ff_msgs::Zone::CLUTTER) {
      zones_file_content += "false,\n";
    } else if (map_it->second.begin()->type == ff_msgs::Zone::KEEPIN) {
      zones_file_content += "true,\n";
    } else {
      ROS_ERROR("DDS ROS bridge: Unknown zone type in zone to compressed file");
      return;
    }

    // Add zone name
    zones_file_content += "\t\t\"name\" : \"" + map_it->second.begin()->name;
    zones_file_content += "\"\n\t},";
  }

  // Remove comma
  zones_file_content.pop_back();
  zones_file_content += " ]\n}";

  // Fill rapid compressed file message
  rapid::ext::astrobee::CompressedFile &msg = file_supplier_->event();
  msg.hdr.timeStamp = util::RosTime2RapidTime(get_zones_srv.response.timestamp);

  // Leave the id blank. This is only used when GDS sends a compressed file to
  // Astrobee.

  // Set compression type
  msg.compressionType = rapid::ext::astrobee::COMPRESSION_TYPE_DEFLATE;

  // Set data
  // First compress the data
  boost::iostreams::filtering_ostream out;
  boost::iostreams::basic_array_source<char> source(zones_file_content.c_str(),
                                                    zones_file_content.size());

  std::vector<char> data;
  // This is the max size the compressed file can be in the rapid message
  uintmax_t max_cf_size = 128 * 1024;
  data.reserve(max_cf_size);

  out.push(boost::iostreams::zlib_compressor());
  out.push(boost::iostreams::back_inserter(data));

  boost::iostreams::copy(source, out);

  if (data.size() >= max_cf_size) {
    ROS_ERROR_STREAM("DDS ROS Bridge: The compressed file for the zones is " <<
                     "too big to send to the ground.");
    return;
  }

  // Next set the data in the rapid message
  msg.compressedFile.ensure_length(data.size(), data.size());
  unsigned char* buf = msg.compressedFile.get_contiguous_buffer();
  for (unsigned int i = 0; i < data.size(); i++) {
    buf[i] = data[i];
  }

  file_supplier_->sendEvent();
}
