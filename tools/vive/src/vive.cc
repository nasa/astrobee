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
#include <vive/vive.h>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>


/**
 * \ingroup tools
 */
namespace vive {

  bool ViveUtils::WriteConfig(std::string file_name, Calibration const& calibration) {
    std::ofstream ofs(file_name, std::ios::out | std::ios::binary);
    if (!ofs.is_open()) {
      ROS_INFO_STREAM("Cannot write to file " << file_name);
      return false;
    }

    ff_msgs::ViveCalibration vive_calibration;

    // Environment
    vive_calibration.calibration.translation.x = calibration.environment.vive.translation.x;
    vive_calibration.calibration.translation.y = calibration.environment.vive.translation.y;
    vive_calibration.calibration.translation.z = calibration.environment.vive.translation.z;
    vive_calibration.calibration.rotation.w = calibration.environment.vive.rotation.w;
    vive_calibration.calibration.rotation.x = calibration.environment.vive.rotation.x;
    vive_calibration.calibration.rotation.y = calibration.environment.vive.rotation.y;
    vive_calibration.calibration.rotation.z = calibration.environment.vive.rotation.z;

    // Lighthouses
    for (std::map<std::string, Transform>::const_iterator lh_it = calibration.environment.lighthouses.begin();
      lh_it != calibration.environment.lighthouses.end(); lh_it++) {
      geometry_msgs::TransformStamped lighthouse;
      lighthouse.header.frame_id = lh_it->second.parent_frame;
      lighthouse.child_frame_id = lh_it->second.child_frame;
      lighthouse.transform.translation.x = lh_it->second.translation.x;
      lighthouse.transform.translation.y = lh_it->second.translation.y;
      lighthouse.transform.translation.z = lh_it->second.translation.z;
      lighthouse.transform.rotation.w = lh_it->second.rotation.w;
      lighthouse.transform.rotation.x = lh_it->second.rotation.x;
      lighthouse.transform.rotation.y = lh_it->second.rotation.y;
      lighthouse.transform.rotation.z = lh_it->second.rotation.z;
      vive_calibration.lighthouse.push_back(lighthouse);
    }

    uint32_t serial_size = ros::serialization::serializationLength(vive_calibration);
    boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
    ros::serialization::OStream ostream(obuffer.get(), serial_size);
    ros::serialization::serialize(ostream, vive_calibration);
    ofs.write(reinterpret_cast<char*>(obuffer.get()), serial_size);
    ofs.close();
    return true;
  }

  bool ViveUtils::ReadConfig(std::string file_name, Calibration * calibration) {
    std::ifstream ifs(file_name, std::ios::in | std::ios::binary);
    ff_msgs::ViveCalibration vive_calibration;
    if (!ifs.good()) {
      ROS_INFO_STREAM("Cannot read from file " << file_name);
      return false;
    }
    // Read data
    ifs.seekg(0, std::ios::end);
    std::streampos end = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    std::streampos begin = ifs.tellg();
    uint32_t file_size = end - begin;
    boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
    ifs.read(reinterpret_cast<char*>(ibuffer.get()), file_size);
    ros::serialization::IStream istream(ibuffer.get(), file_size);
    ros::serialization::deserialize(istream, vive_calibration);
    ifs.close();

    // Converting

    // Environment
    (*calibration).environment.vive.parent_frame = "world";
    (*calibration).environment.vive.child_frame = "vive";
    (*calibration).environment.vive.translation.x = vive_calibration.calibration.translation.x;
    (*calibration).environment.vive.translation.y = vive_calibration.calibration.translation.y;
    (*calibration).environment.vive.translation.z = vive_calibration.calibration.translation.z;
    (*calibration).environment.vive.rotation.w = vive_calibration.calibration.rotation.w;
    (*calibration).environment.vive.rotation.x = vive_calibration.calibration.rotation.x;
    (*calibration).environment.vive.rotation.y = vive_calibration.calibration.rotation.y;
    (*calibration).environment.vive.rotation.z = vive_calibration.calibration.rotation.z;

    // Lighthouses
    for (std::vector<geometry_msgs::TransformStamped>::iterator lh_it = vive_calibration.lighthouse.begin();
      lh_it != vive_calibration.lighthouse.end(); lh_it++) {
      Transform lighthouse;
      lighthouse.parent_frame = lh_it->header.frame_id;
      lighthouse.child_frame = lh_it->child_frame_id;
      lighthouse.translation.x = lh_it->transform.translation.x;
      lighthouse.translation.y = lh_it->transform.translation.y;
      lighthouse.translation.z = lh_it->transform.translation.z;
      lighthouse.rotation.w = lh_it->transform.rotation.w;
      lighthouse.rotation.x = lh_it->transform.rotation.x;
      lighthouse.rotation.y = lh_it->transform.rotation.y;
      lighthouse.rotation.z = lh_it->transform.rotation.z;
      (*calibration).environment.lighthouses[lh_it->child_frame_id] = lighthouse;
    }
    return true;
  }

  bool ViveUtils::GetLighthouses(config_reader::ConfigReader::Table lighthouses,
    Calibration * calibration) {
    config_reader::ConfigReader::Table tmp;
    for (int i = 0; i < lighthouses.GetSize(); i++) {
      config_reader::ConfigReader::Table lighthouse;
      geometry_msgs::TransformStamped lighthouse_transform;
        if (!lighthouses.GetTable(i+1, &lighthouse)
        || !lighthouse.GetStr("parent", &lighthouse_transform.header.frame_id)
        || !lighthouse.GetStr("frame", &lighthouse_transform.child_frame_id)
        || !lighthouse.GetTable("t", &tmp)
        || !msg_conversions::config_read_vector(&tmp, &lighthouse_transform.transform.translation)
        || !lighthouse.GetTable("r", &tmp)
        || !msg_conversions::config_read_quat(&tmp, &lighthouse_transform.transform.rotation)) {
        return false;
      }
      // Converting to my structs
      Transform my_lighthouse;
      my_lighthouse.parent_frame = lighthouse_transform.header.frame_id;
      my_lighthouse.child_frame = lighthouse_transform.child_frame_id;
      my_lighthouse.translation.x = lighthouse_transform.transform.translation.x;
      my_lighthouse.translation.y = lighthouse_transform.transform.translation.y;
      my_lighthouse.translation.z = lighthouse_transform.transform.translation.z;

      my_lighthouse.rotation.w = lighthouse_transform.transform.rotation.w;
      my_lighthouse.rotation.x = lighthouse_transform.transform.rotation.x;
      my_lighthouse.rotation.y = lighthouse_transform.transform.rotation.y;
      my_lighthouse.rotation.z = lighthouse_transform.transform.rotation.z;

      (*calibration).environment.lighthouses[lighthouse_transform.child_frame_id] = my_lighthouse;
    }
    return true;
  }

  bool ViveUtils::GetTrackers(config_reader::ConfigReader::Table trackers,
    Calibration * calibration) {
    config_reader::ConfigReader::Table tmp;
    for (int i = 0; i < trackers.GetSize(); ++i) {
      Tracker tracker;
      config_reader::ConfigReader::Table tracker_table, sensors;
      if (!trackers.GetTable(i+1, &tracker_table)
        || !tracker_table.GetStr("parent", &tracker.serial)
        || !tracker_table.GetTable("extrinsics", &sensors)) {
        return false;
      }
      std::vector<geometry_msgs::PointStamped> sensor_vec;
      // Reading sensors from each tracker
      for (int i = 0; i < sensors.GetSize(); i++) {
        int sensor_id;
        Sensor sensor;
        if (!sensors.GetTable(i+1, &tmp)
          || !tmp.GetInt("sensor", &sensor_id)
          || !tmp.GetReal("x", &sensor.position.x)
          || !tmp.GetReal("y", &sensor.position.y)
          || !tmp.GetReal("z", &sensor.position.z)) {
          return false;
        }
        tracker.sensors[unsigned(sensor_id)] = sensor;
      }
      (*calibration).trackers[tracker.serial] = tracker;
    }
    return true;
  }

  bool ViveUtils::GetBodies(config_reader::ConfigReader::Table bodies,
    Calibration * calibration) {
    config_reader::ConfigReader::Table tmp;

    for (int i = 0; i < bodies.GetSize(); i++) {
      config_reader::ConfigReader::Table body_table, parents;
      std::string body_name;
      if (!bodies.GetTable(i+1, &body_table)
        || !body_table.GetStr("frame", &body_name)
        || !body_table.GetTable("parents", &parents)) {
        return false;
      }
      for (int j = 0; j < parents.GetSize(); j++) {
        config_reader::ConfigReader::Table body_tracker;
        geometry_msgs::Transform tf;
        Transform body;
        if (!parents.GetTable(j+1, &body_tracker)
          || !body_tracker.GetStr("frame", &body.parent_frame)
          || !body_tracker.GetTable("t", &tmp)
          || !msg_conversions::config_read_vector(&tmp, &tf.translation)
          || !body_tracker.GetTable("r", &tmp)
          || !msg_conversions::config_read_quat(&tmp, &tf.rotation)) {
          return false;
        } else {
          // Convertion
          body.child_frame = body_name;
          body.translation.x = tf.translation.x;
          body.translation.y = tf.translation.y;
          body.translation.z = tf.translation.z;
          body.rotation.w = tf.rotation.w;
          body.rotation.x = tf.rotation.x;
          body.rotation.y = tf.rotation.y;
          body.rotation.z = tf.rotation.z;
          (*calibration).environment.bodies[body.parent_frame] = body;
        }
      }
      config_reader::ConfigReader::Table offset_table;
      geometry_msgs::Transform tf;
      if (!body_table.GetTable("offset", &offset_table)
        || !offset_table.GetTable("t", &tmp)
        || !msg_conversions::config_read_vector(&tmp, &tf.translation)
        || !offset_table.GetTable("r", &tmp)
        || !msg_conversions::config_read_quat(&tmp, &tf.rotation)) {
        return false;
      }
      // Convertion
      (*calibration).environment.offset.translation.x = tf.translation.x;
      (*calibration).environment.offset.translation.y = tf.translation.y;
      (*calibration).environment.offset.translation.z = tf.translation.z;
      (*calibration).environment.offset.rotation.w = tf.rotation.w;
      (*calibration).environment.offset.rotation.x = tf.rotation.x;
      (*calibration).environment.offset.rotation.y = tf.rotation.y;
      (*calibration).environment.offset.rotation.z = tf.rotation.z;
    }
    return true;
  }

  bool ViveUtils::GetCalibration(config_reader::ConfigReader::Table calibration_table,
    Calibration * calibration) {
    config_reader::ConfigReader::Table tmp;
    geometry_msgs::Transform tf;
    if (!calibration_table.GetTable("t", &tmp)
      || !msg_conversions::config_read_vector(&tmp, &tf.translation)
      || !calibration_table.GetTable("r", &tmp)
      || !msg_conversions::config_read_quat(&tmp, &tf.rotation)) {
      return false;
    }

    (*calibration).environment.vive.translation.x = tf.translation.x;
    (*calibration).environment.vive.translation.y = tf.translation.y;
    (*calibration).environment.vive.translation.z = tf.translation.z;
    (*calibration).environment.vive.rotation.w = tf.rotation.w;
    (*calibration).environment.vive.rotation.x = tf.rotation.x;
    (*calibration).environment.vive.rotation.y = tf.rotation.y;
    (*calibration).environment.vive.rotation.z = tf.rotation.z;
    (*calibration).environment.vive.parent_frame = "world";
    (*calibration).environment.vive.child_frame = "vive";

    return true;
  }

  bool ViveUtils::SendTransforms(Calibration const& calibration_data) {
    // Broadcaster of all the trasnforms read from the config file
    static tf2_ros::StaticTransformBroadcaster static_br;
    geometry_msgs::TransformStamped world_tf;
    world_tf.header.stamp = ros::Time::now();
    world_tf.header.frame_id = calibration_data.environment.vive.parent_frame;
    world_tf.child_frame_id = calibration_data.environment.vive.child_frame;
    world_tf.transform.translation.x = calibration_data.environment.vive.translation.x;
    world_tf.transform.translation.y = calibration_data.environment.vive.translation.y;
    world_tf.transform.translation.z = calibration_data.environment.vive.translation.z;
    world_tf.transform.rotation.w = calibration_data.environment.vive.rotation.w;
    world_tf.transform.rotation.x = calibration_data.environment.vive.rotation.x;
    world_tf.transform.rotation.y = calibration_data.environment.vive.rotation.y;
    world_tf.transform.rotation.z = calibration_data.environment.vive.rotation.z;
    static_br.sendTransform(world_tf);

    for (std::map<std::string, Transform>::const_iterator lh_it = calibration_data.environment.lighthouses.begin();
      lh_it != calibration_data.environment.lighthouses.end(); lh_it++) {
      geometry_msgs::TransformStamped lighthouse_tf;
      lighthouse_tf.header.stamp = ros::Time::now();
      lighthouse_tf.header.frame_id = lh_it->second.parent_frame;
      lighthouse_tf.child_frame_id = lh_it->second.child_frame;
      lighthouse_tf.transform.translation.x = lh_it->second.translation.x;
      lighthouse_tf.transform.translation.y = lh_it->second.translation.y;
      lighthouse_tf.transform.translation.z = lh_it->second.translation.z;
      lighthouse_tf.transform.rotation.w = lh_it->second.rotation.w;
      lighthouse_tf.transform.rotation.x = lh_it->second.rotation.x;
      lighthouse_tf.transform.rotation.y = lh_it->second.rotation.y;
      lighthouse_tf.transform.rotation.z = lh_it->second.rotation.z;
      static_br.sendTransform(lighthouse_tf);
    }
    return true;
  }

  size_t ViveUtils::ConvertExtrinsics(Tracker const& tracker, double * extrinsics) {
    for (std::map<uint8_t, Sensor>::const_iterator sn_it = tracker.sensors.begin();
      sn_it != tracker.sensors.end(); sn_it++) {
      if (unsigned(sn_it->first) >= TRACKER_SENSORS_NUMBER) {
        return -1;
      }
      extrinsics[3 * unsigned(sn_it->first)] =     sn_it->second.position.x;
      extrinsics[3 * unsigned(sn_it->first) + 1] = sn_it->second.position.y;
      extrinsics[3 * unsigned(sn_it->first) + 2] = sn_it->second.position.z;
    }
    return tracker.sensors.size();
  }

  bool Calibration::SetEnvironment(ff_msgs::ViveCalibration const& msg) {
    environment.vive.parent_frame = "world";
    environment.vive.child_frame = "vive";
    environment.vive.translation.x = msg.calibration.translation.x;
    environment.vive.translation.y = msg.calibration.translation.y;
    environment.vive.translation.z = msg.calibration.translation.z;
    environment.vive.rotation.w = msg.calibration.rotation.w;
    environment.vive.rotation.x = msg.calibration.rotation.x;
    environment.vive.rotation.y = msg.calibration.rotation.y;
    environment.vive.rotation.z = msg.calibration.rotation.z;

    // Lighthouse
    for (std::vector<geometry_msgs::TransformStamped>::const_iterator lh_it = msg.lighthouse.begin();
      lh_it != msg.lighthouse.end(); lh_it++) {
      environment.lighthouses[lh_it->child_frame_id].parent_frame = lh_it->header.frame_id;
      environment.lighthouses[lh_it->child_frame_id].child_frame = lh_it->child_frame_id;

      environment.lighthouses[lh_it->child_frame_id].translation.x = lh_it->transform.translation.x;
      environment.lighthouses[lh_it->child_frame_id].translation.y = lh_it->transform.translation.y;
      environment.lighthouses[lh_it->child_frame_id].translation.z = lh_it->transform.translation.z;

      environment.lighthouses[lh_it->child_frame_id].rotation.w = lh_it->transform.rotation.w;
      environment.lighthouses[lh_it->child_frame_id].rotation.x = lh_it->transform.rotation.x;
      environment.lighthouses[lh_it->child_frame_id].rotation.y = lh_it->transform.rotation.y;
      environment.lighthouses[lh_it->child_frame_id].rotation.z = lh_it->transform.rotation.z;
    }

    // Body
    for (std::vector<geometry_msgs::TransformStamped>::const_iterator bd_it = msg.body.begin();
      bd_it != msg.body.end(); bd_it++) {
      environment.bodies[bd_it->header.frame_id].parent_frame = bd_it->header.frame_id;
      environment.bodies[bd_it->header.frame_id].child_frame = bd_it->child_frame_id;

      environment.bodies[bd_it->header.frame_id].translation.x = bd_it->transform.translation.x;
      environment.bodies[bd_it->header.frame_id].translation.y = bd_it->transform.translation.y;
      environment.bodies[bd_it->header.frame_id].translation.z = bd_it->transform.translation.z;

      environment.bodies[bd_it->header.frame_id].rotation.w = bd_it->transform.rotation.w;
      environment.bodies[bd_it->header.frame_id].rotation.x = bd_it->transform.rotation.x;
      environment.bodies[bd_it->header.frame_id].rotation.y = bd_it->transform.rotation.y;
      environment.bodies[bd_it->header.frame_id].rotation.z = bd_it->transform.rotation.z;
    }
    return true;
  }

  bool Calibration::GetEnvironment(ff_msgs::ViveCalibration * msg) {
    (*msg).calibration.translation.x = environment.vive.translation.x;
    (*msg).calibration.translation.y = environment.vive.translation.y;
    (*msg).calibration.translation.z = environment.vive.translation.z;
    (*msg).calibration.rotation.w = environment.vive.rotation.w;
    (*msg).calibration.rotation.x = environment.vive.rotation.x;
    (*msg).calibration.rotation.y = environment.vive.rotation.y;
    (*msg).calibration.rotation.z = environment.vive.rotation.z;

    // Lighthouse
    for (std::map<std::string, Transform>::iterator lh_it = environment.lighthouses.begin();
      lh_it != environment.lighthouses.end(); lh_it++) {
      geometry_msgs::TransformStamped lh_msg;
      lh_msg.header.frame_id = lh_it->second.parent_frame;
      lh_msg.child_frame_id = lh_it->second.child_frame;

      lh_msg.transform.translation.x = lh_it->second.translation.x;
      lh_msg.transform.translation.y = lh_it->second.translation.y;
      lh_msg.transform.translation.z = lh_it->second.translation.z;

      lh_msg.transform.rotation.w = lh_it->second.rotation.w;
      lh_msg.transform.rotation.x = lh_it->second.rotation.x;
      lh_msg.transform.rotation.y = lh_it->second.rotation.y;
      lh_msg.transform.rotation.z = lh_it->second.rotation.z;
      (*msg).lighthouse.push_back(lh_msg);
    }

    // Body
    for (std::map<std::string, Transform>::iterator bd_it = environment.bodies.begin();
      bd_it != environment.bodies.end(); bd_it++) {
      geometry_msgs::TransformStamped bd_msg;
      bd_msg.header.frame_id = bd_it->second.parent_frame;
      bd_msg.child_frame_id = bd_it->second.child_frame;

      bd_msg.transform.translation.x = bd_it->second.translation.x;
      bd_msg.transform.translation.y = bd_it->second.translation.y;
      bd_msg.transform.translation.z = bd_it->second.translation.z;

      bd_msg.transform.rotation.w = bd_it->second.rotation.w;
      bd_msg.transform.rotation.x = bd_it->second.rotation.x;
      bd_msg.transform.rotation.y = bd_it->second.rotation.y;
      bd_msg.transform.rotation.z = bd_it->second.rotation.z;
      (*msg).body.push_back(bd_msg);
    }
    return true;
  }

  bool Calibration::SetLightSpecs(ff_msgs::ViveCalibrationGeneral const& msg) {
    light_specs.timebase_hz = msg.timebase_hz;
    light_specs.timecenter_ticks = msg.timecenter_ticks;
    light_specs.pulsedist_max_ticks = msg.pulsedist_max_ticks;
    light_specs.pulselength_min_sync = msg.pulselength_min_sync;
    light_specs.pulse_in_clear_time = msg.pulse_in_clear_time;
    light_specs.pulse_max_for_sweep = msg.pulse_max_for_sweep;
    light_specs.pulse_synctime_offset = msg.pulse_synctime_offset;
    light_specs.pulse_synctime_slack = msg.pulse_synctime_slack;
    return true;
  }

  bool Calibration::GetLightSpecs(ff_msgs::ViveCalibrationGeneral * msg) {
    (*msg).timebase_hz = light_specs.timebase_hz;
    (*msg).timecenter_ticks = light_specs.timecenter_ticks;
    (*msg).pulsedist_max_ticks = light_specs.pulsedist_max_ticks;
    (*msg).pulselength_min_sync = light_specs.pulselength_min_sync;
    (*msg).pulse_in_clear_time = light_specs.pulse_in_clear_time;
    (*msg).pulse_max_for_sweep = light_specs.pulse_max_for_sweep;
    (*msg).pulse_synctime_offset = light_specs.pulse_synctime_offset;
    (*msg).pulse_synctime_slack = light_specs.pulse_synctime_slack;
    return true;
  }

  bool Calibration::SetLighthouses(ff_msgs::ViveCalibrationLighthouseArray const& msg) {
    for (std::vector<ff_msgs::ViveCalibrationLighthouse>::const_iterator lh_it = msg.lighthouses.begin();
      lh_it != msg.lighthouses.end(); lh_it++) {
      lighthouses[lh_it->serial].serial = lh_it->serial;
      lighthouses[lh_it->serial].id = lh_it->id;
      // Vertical Motor
      lighthouses[lh_it->serial].vertical_motor.phase = lh_it->vertical.phase;
      lighthouses[lh_it->serial].vertical_motor.tilt = lh_it->vertical.tilt;
      lighthouses[lh_it->serial].vertical_motor.gib_phase = lh_it->vertical.gibphase;
      lighthouses[lh_it->serial].vertical_motor.gib_magnitude = lh_it->vertical.gibmag;
      lighthouses[lh_it->serial].vertical_motor.curve = lh_it->vertical.curve;
      // Horizontal Motor
      lighthouses[lh_it->serial].horizontal_motor.phase = lh_it->horizontal.phase;
      lighthouses[lh_it->serial].horizontal_motor.tilt = lh_it->horizontal.tilt;
      lighthouses[lh_it->serial].horizontal_motor.gib_phase = lh_it->horizontal.gibphase;
      lighthouses[lh_it->serial].horizontal_motor.gib_magnitude = lh_it->horizontal.gibmag;
      lighthouses[lh_it->serial].horizontal_motor.curve = lh_it->horizontal.curve;
    }
    return true;
  }

  bool Calibration::GetLighthouses(ff_msgs::ViveCalibrationLighthouseArray * msg) {
    for (std::map<std::string, Lighthouse>::iterator lh_it = lighthouses.begin();
      lh_it != lighthouses.end(); lh_it++) {
      ff_msgs::ViveCalibrationLighthouse lh_msg;
      lh_msg.serial = lh_it->second.serial;
      lh_msg.id = lh_it->second.id;
      // Vertical Motor
      lh_msg.vertical.phase = lh_it->second.vertical_motor.phase;
      lh_msg.vertical.tilt = lh_it->second.vertical_motor.tilt;
      lh_msg.vertical.gibphase = lh_it->second.vertical_motor.gib_phase;
      lh_msg.vertical.gibmag = lh_it->second.vertical_motor.gib_magnitude;
      lh_msg.vertical.curve = lh_it->second.vertical_motor.curve;
      // Horizontal Motor
      lh_msg.horizontal.phase = lh_it->second.horizontal_motor.phase;
      lh_msg.horizontal.tilt = lh_it->second.horizontal_motor.tilt;
      lh_msg.horizontal.gibphase = lh_it->second.horizontal_motor.gib_phase;
      lh_msg.horizontal.gibmag = lh_it->second.horizontal_motor.gib_magnitude;
      lh_msg.horizontal.curve = lh_it->second.horizontal_motor.curve;
      (*msg).lighthouses.push_back(lh_msg);
    }
    return true;
  }

  bool Calibration::SetTrackers(ff_msgs::ViveCalibrationTrackerArray const& msg) {
    for (std::vector<ff_msgs::ViveCalibrationTracker>::const_iterator tr_it = msg.trackers.begin();
      tr_it != msg.trackers.end(); tr_it++) {
      // trackers[tr_it->serial].frame = tr_it->serial;
      trackers[tr_it->serial].serial = tr_it->serial;

      // Acc bias
      trackers[tr_it->serial].acc_bias.x = tr_it->acc_bias.x;
      trackers[tr_it->serial].acc_bias.y = tr_it->acc_bias.y;
      trackers[tr_it->serial].acc_bias.z = tr_it->acc_bias.z;

      // Acc scale
      trackers[tr_it->serial].acc_scale.x = tr_it->acc_scale.x;
      trackers[tr_it->serial].acc_scale.y = tr_it->acc_scale.y;
      trackers[tr_it->serial].acc_scale.z = tr_it->acc_scale.z;

      // Gyr bias
      trackers[tr_it->serial].gyr_bias.x = tr_it->gyr_bias.x;
      trackers[tr_it->serial].gyr_bias.y = tr_it->gyr_bias.y;
      trackers[tr_it->serial].gyr_bias.z = tr_it->gyr_bias.z;

      // Gyr scale
      trackers[tr_it->serial].gyr_scale.x = tr_it->gyr_scale.x;
      trackers[tr_it->serial].gyr_scale.y = tr_it->gyr_scale.y;
      trackers[tr_it->serial].gyr_scale.z = tr_it->gyr_scale.z;

      for (std::vector<ff_msgs::ViveExtrinsics>::const_iterator ss_it = tr_it->extrinsics.begin();
        ss_it != tr_it->extrinsics.end(); ss_it++) {
        // Sensor positions
        trackers[tr_it->serial].sensors[ss_it->id].position.x = ss_it->position.x;
        trackers[tr_it->serial].sensors[ss_it->id].position.y = ss_it->position.y;
        trackers[tr_it->serial].sensors[ss_it->id].position.z = ss_it->position.z;
        // Sensors normal
        trackers[tr_it->serial].sensors[ss_it->id].normal.x = ss_it->normal.x;
        trackers[tr_it->serial].sensors[ss_it->id].normal.y = ss_it->normal.y;
        trackers[tr_it->serial].sensors[ss_it->id].normal.z = ss_it->normal.z;
      }
    }
    return true;
  }

  bool Calibration::GetTrackers(ff_msgs::ViveCalibrationTrackerArray * msg) {
    for (std::map<std::string, Tracker>::iterator tr_it = trackers.begin();
      tr_it != trackers.end(); tr_it++) {
      ff_msgs::ViveCalibrationTracker tracker_msg;
      tracker_msg.serial = tr_it->first;

      // Acc bias
      tracker_msg.acc_bias.x = tr_it->second.acc_bias.x;
      tracker_msg.acc_bias.y = tr_it->second.acc_bias.y;
      tracker_msg.acc_bias.z = tr_it->second.acc_bias.z;

      // Acc scale
      tracker_msg.acc_scale.x = tr_it->second.acc_scale.x;
      tracker_msg.acc_scale.y = tr_it->second.acc_scale.y;
      tracker_msg.acc_scale.z = tr_it->second.acc_scale.z;

      // Gyr bias
      tracker_msg.gyr_bias.x = tr_it->second.gyr_bias.x;
      tracker_msg.gyr_bias.y = tr_it->second.gyr_bias.y;
      tracker_msg.gyr_bias.z = tr_it->second.gyr_bias.z;

      // Gyr scale
      tracker_msg.gyr_scale.x = tr_it->second.gyr_scale.x;
      tracker_msg.gyr_scale.y = tr_it->second.gyr_scale.y;
      tracker_msg.gyr_scale.z = tr_it->second.gyr_scale.z;

      for (std::map<uint8_t, Sensor>::iterator ss_it = tr_it->second.sensors.begin();
        ss_it != tr_it->second.sensors.end(); ss_it++) {
        ff_msgs::ViveExtrinsics sensor_msg;
        sensor_msg.id = ss_it->first;
        // Sensor positions
        sensor_msg.position.x = ss_it->second.position.x;
        sensor_msg.position.y = ss_it->second.position.y;
        sensor_msg.position.z = ss_it->second.position.z;
        // Sensor normal
        sensor_msg.normal.x = ss_it->second.normal.x;
        sensor_msg.normal.y = ss_it->second.normal.y;
        sensor_msg.normal.z = ss_it->second.normal.z;
        tracker_msg.extrinsics.push_back(sensor_msg);
      }
      (*msg).trackers.push_back(tracker_msg);
    }
    return true;
  }

}  // namespace vive
