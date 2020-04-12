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

#include <common/init.h>
#include <ekf_bag/ekf_bag_csv.h>

DEFINE_bool(gen_features, false,
            "If true, generate features from image, otherwise use from bag.");
DEFINE_bool(run_ekf, false,
            "If true, run EKF, otherwise read messages from bag.");
DEFINE_string(image_topic, TOPIC_HARDWARE_NAV_CAM,
              "The topic to get images from..");

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);

  ros::Time::init();

  if (argc < 3) {
    LOG(INFO) << "Usage: " << argv[0] << " map.map bag.bag output.txt";
    exit(0);
  }

  const char* biasfile = NULL;
  if (argc >= 5) biasfile = argv[4];

  ekf_bag::EkfBagCsv bag(argv[2], argv[1], argv[3], FLAGS_run_ekf,
                         FLAGS_gen_features, biasfile, FLAGS_image_topic);

  bag.Run();
}
