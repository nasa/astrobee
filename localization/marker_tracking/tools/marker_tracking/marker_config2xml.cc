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

#include <marker_tracking/arconfigio.h>

int main(int argc, char* argv[]) {
  if (argc < 2) {
    printf("Usage: marker_config2xml input_config_file\n");
    exit(-1);
  }

  marker_tracking::ARTagMap ar_tags;
  config_reader::ConfigReader config;
  config.AddFile(argv[1]);
  marker_tracking::LoadARTagsConfig(&config, &ar_tags);

  for (std::pair<int, Eigen::Matrix<float, 4, 3>> marker : ar_tags) {
    printf("  <ar id=\"%d\"\n", marker.first);
    // corner ordering : 0=BL - 1=BR - 2=TR - 3=TL
    printf("    topleft=\"%.4f %.4f %.4f\"\n", marker.second(3, 0),
           marker.second(3, 1), marker.second(3, 2));
    printf("    topright=\"%.4f %.4f %.4f\"\n", marker.second(2, 0),
           marker.second(2, 1), marker.second(2, 2));
    printf("    bottomleft=\"%.4f %.4f %.4f\"\n", marker.second(0, 0),
           marker.second(0, 1), marker.second(0, 2));
    printf("  />\n");
  }
}
