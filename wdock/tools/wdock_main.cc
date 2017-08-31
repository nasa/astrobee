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


#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "wdock/wdock.h"

int main(int argc, char** argv) {
  w_dock::WDock wan_dock(argc, argv, "RapidCommand");

  wan_dock.SetBerthOne();
  wan_dock.SetBerthTwo();
  while (true) {
    wan_dock.ProcessDdsEventLoop();
    sleep(1);
  }

  return 1;
}
