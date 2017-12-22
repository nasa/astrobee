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

// Royale SDK interface
#include <royale/CameraManager.hpp>

// C / C++ includes
#include <iostream>

int main(int argc, const char* argv[]) {
  // Print camera list
  royale::CameraManager manager;
  royale::Vector<royale::String> cam_list = manager.getConnectedCameraList();
  if (cam_list.empty()) {
    std::cerr << "Could not find any cameras" << std::endl;
    return 1;
  }
  std::cout << "Found " << cam_list.size() << " cameras:" << std::endl;
  for (size_t i = 0; i < cam_list.size(); ++i)
    std::cout << "- " << cam_list[i] << std::endl;
  return 0;
}
