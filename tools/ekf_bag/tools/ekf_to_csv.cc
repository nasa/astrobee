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

int main(int argc, char ** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);

  if (argc < 3) {
    LOG(INFO) << "Usage: " << argv[0] << " map.map bag.bag output.txt";
    exit(0);
  }

  ekf_bag::EkfBagCsv bag(argv[2], argv[1], argv[3]);

  bag.Run();
}

