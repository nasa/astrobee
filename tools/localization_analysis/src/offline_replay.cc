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

#include <ff_common/ff_names.h>
#include <ff_common/utils.h>
#include <localization_analysis/feature_track_image_adder.h>
#include <localization_analysis/offline_replay.h>
#include <localization_analysis/parameter_reader.h>
#include <localization_analysis/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Core>

#include <chrono>
#include <cstdlib>
#include <vector>

namespace localization_analysis {
namespace gl = graph_localizer;
namespace gv = graph_vio;
namespace lc = localization_common;
namespace mc = msg_conversions;

OfflineReplay::OfflineReplay(const std::string& bag_name, const std::string& map_file, const std::string& image_topic,
                             const std::string& results_bag, const std::string& output_stats_file,
                             const bool use_bag_image_feature_msgs, const std::string& graph_config_path_prefix)
    : results_bag_(results_bag, rosbag::bagmode::Write),
      // imu_bias_tester_wrapper_(graph_config_path_prefix),
      pose_extrapolator_wrapper_(graph_config_path_prefix),
      output_stats_file_(output_stats_file) {
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");
  config.AddFile("tools/offline_replay.config");
  lc::LoadGraphLocalizerConfig(config, graph_config_path_prefix);
  lc::LoadGraphVIOConfig(config, graph_config_path_prefix);
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }

  LiveMeasurementSimulatorParams params;
  LoadLiveMeasurementSimulatorParams(config, bag_name, map_file, image_topic, params);
  // Load this seperately so this can be set independently of config file,
  // i.e. when running a bag sweep or param sweep
  // TODO(rsoussan): clean this up
  params.use_bag_image_feature_msgs = use_bag_image_feature_msgs;
  live_measurement_simulator_.reset(new LiveMeasurementSimulator(params));

  GraphLocalizerSimulatorParams graph_loc_params;
  LoadGraphLocalizerSimulatorParams(config, graph_loc_params);
  graph_localizer_simulator_.reset(new GraphLocalizerSimulator(graph_loc_params, graph_config_path_prefix));

  GraphVIOSimulatorParams graph_vio_params;
  LoadGraphVIOSimulatorParams(config, graph_vio_params);
  graph_vio_simulator_.reset(new GraphVIOSimulator(graph_vio_params, graph_config_path_prefix));
  LoadOfflineReplayParams(config, params_);
}

void OfflineReplay::Run() {
  // Required to start bias estimation
  graph_vio_simulator_->ResetBiasesAndVIO();
  graph_localizer_simulator_->ResetLocalizer();
  lc::Timer offline_replay_timer("Offline Replay Timer");
  offline_replay_timer.Start();
  const double start_time = live_measurement_simulator_->CurrentTime();
  while (live_measurement_simulator_->ProcessMessage()) {
    const lc::Time current_time = live_measurement_simulator_->CurrentTime();
    if (params_.log_relative_time) LogInfo("Run: Rel t: " << current_time - start_time);
    const auto flight_mode_msg = live_measurement_simulator_->GetFlightModeMessage(current_time);
    if (flight_mode_msg) {
      graph_vio_simulator_->BufferFlightModeMsg(*flight_mode_msg);
      graph_localizer_simulator_->BufferFlightModeMsg(*flight_mode_msg);
      pose_extrapolator_wrapper_.FlightModeCallback(*flight_mode_msg);
    }
    const auto imu_msg = live_measurement_simulator_->GetImuMessage(current_time);
    if (imu_msg) {
      graph_vio_simulator_->BufferImuMsg(*imu_msg);
      graph_localizer_simulator_->BufferImuMsg(*imu_msg);
      pose_extrapolator_wrapper_.ImuCallback(*imu_msg);
      SaveMsg(*imu_msg, "hw/imu", results_bag_);

      // Save extrapolated loc msg if available
      const auto extrapolated_loc_msg = pose_extrapolator_wrapper_.LatestExtrapolatedLocalizationMsg();
      if (!extrapolated_loc_msg) {
        LogWarningEveryN(500, "Run: Failed to get latest extrapolated loc msg.");
      } else {
        SaveMsg(*extrapolated_loc_msg, TOPIC_GNC_EKF, results_bag_);
      }
    }
    const auto depth_odometry_msg = live_measurement_simulator_->GetDepthOdometryMessage(current_time);
    if (depth_odometry_msg) {
      graph_vio_simulator_->BufferDepthOdometryMsg(*depth_odometry_msg);
      SaveMsg(*depth_odometry_msg, TOPIC_LOCALIZATION_DEPTH_ODOM, results_bag_);
    }
    const auto of_msg = live_measurement_simulator_->GetOFMessage(current_time);
    if (of_msg) {
      const lc::Time timestamp = lc::TimeFromHeader(of_msg->header);
      graph_vio_simulator_->BufferOpticalFlowMsg(*of_msg);
    }
    const auto vl_msg = live_measurement_simulator_->GetVLMessage(current_time);
    if (vl_msg) {
      const lc::Time timestamp = lc::TimeFromHeader(vl_msg->header);
      graph_localizer_simulator_->BufferVLVisualLandmarksMsg(*vl_msg);
      if (static_cast<int>(vl_msg->landmarks.size()) >= params_.sparse_mapping_min_num_landmarks) {
        const gtsam::Pose3 sparse_mapping_global_T_body =
          lc::PoseFromMsgWithExtrinsics(vl_msg->pose, params_.body_T_nav_cam.inverse());
        const lc::Time timestamp = lc::TimeFromHeader(vl_msg->header);
        SaveMsg(PoseMsg(sparse_mapping_global_T_body, timestamp), TOPIC_SPARSE_MAPPING_POSE, results_bag_);
      }
    }

    const auto ar_msg = live_measurement_simulator_->GetARMessage(current_time);
    if (ar_msg) {
      graph_localizer_simulator_->BufferARVisualLandmarksMsg(*ar_msg);
      latest_ar_msg_ = ar_msg;
    }

    const bool updated_vio_graph = graph_vio_simulator_->AddMeasurementsAndUpdateIfReady(current_time);
    if (updated_vio_graph) {
      // Pass pose covariance interpolater used for relative factors
      // from graph vio to graph localizer
      if (graph_vio_simulator_->Initialized() && graph_localizer_simulator_->Initialized()) {
        graph_localizer_simulator_->graph_localizer()->SetPoseCovarianceInterpolater(
          graph_vio_simulator_->graph_vio()->MarginalsPoseCovarianceInterpolater());
      }
      const auto vio_msg = graph_vio_simulator_->GraphVIOStateMsg();
      if (!vio_msg) {
        LogWarningEveryN(200, "Run: Failed to get vio msg.");
      } else {
        pose_extrapolator_wrapper_.GraphVIOStateCallback(*vio_msg);
        // TODO(rsoussan): Pass this to live measurement simulator? allow for simulated delay?
        graph_localizer_simulator_->BufferGraphVIOStateMsg(*vio_msg);
        SaveMsg(*vio_msg, TOPIC_GRAPH_VIO_STATE, results_bag_);
        if (params_.save_optical_flow_images) {
          const auto& graph_vio = graph_vio_simulator_->graph_vio();
          // Use spaced feature tracks so points only drawn when they are included in the localizer
          const auto latest_time = graph_vio->feature_tracker().SpacedFeatureTracks().crbegin()->crbegin()->timestamp;
          // const auto latest_time =
          // *(graph_vio->feature_tracker().feature_tracks().crbegin()->second.LatestTimestamp());
          const auto img_msg = live_measurement_simulator_->GetImageMessage(latest_time);
          if (img_msg && graph_vio) {
            const auto smart_factors = graph_vio->Factors<factor_adders::RobustSmartFactor>();
            const auto feature_track_image_msg =
              CreateFeatureTrackImage(*img_msg, graph_vio->feature_tracker(), *params_.nav_cam_params, smart_factors,
                                      ((const graph_vio::GraphVIO*)graph_vio.get())->gtsam_values());
            if (!feature_track_image_msg) return;
            SaveMsg(**feature_track_image_msg, kFeatureTracksImageTopic_, results_bag_);
          }
        }
      }
    }
    const bool updated_localizer_graph = graph_localizer_simulator_->AddMeasurementsAndUpdateIfReady(current_time);
    if (updated_localizer_graph) {
      // Save latest graph localization msg
      // Pass latest loc state to imu augmentor if it is available.
      const auto localization_msg = graph_localizer_simulator_->GraphLocStateMsg();
      if (!localization_msg) {
        LogWarningEveryN(1000, "Run: Failed to get localization msg.");
      } else {
        pose_extrapolator_wrapper_.LocalizationStateCallback(*localization_msg);
        SaveMsg(*localization_msg, TOPIC_GRAPH_LOC_STATE, results_bag_);
      }
      if (latest_ar_msg_ && graph_localizer_simulator_->WorldTDock()) {
        if (static_cast<int>(ar_msg->landmarks.size()) >= params_.ar_min_num_landmarks) {
          const gtsam::Pose3 world_T_body =
            (*graph_localizer_simulator_->WorldTDock()) *
            lc::PoseFromMsgWithExtrinsics(ar_msg->pose, params_.body_T_dock_cam.inverse());
          const lc::Time timestamp = lc::TimeFromHeader(ar_msg->header);
          SaveMsg(PoseMsg(world_T_body, timestamp), TOPIC_AR_TAG_POSE, results_bag_);
          latest_ar_msg_ = boost::none;
        }
      }
    }
  }
  offline_replay_timer.Stop();
  offline_replay_timer.Log();
  /*const auto graph_stats = graph_localizer_simulator_->graph_localizer_stats();
  if (!graph_stats) {
    LogError("Run: Failed to get graph stats");
  } else {
    std::ofstream log_file;
    log_file.open(output_stats_file_);
    graph_stats->LogToCsv(log_file);
    offline_replay_timer.LogToCsv(log_file);
  }*/
}
}  // namespace localization_analysis
