# Astrobee Robot Software - Flight Software repository

## About

<p>
<img src="doc/images/astrobee.png" srcset="../images/astrobee.png 1x" title="Astrobee" align="right" style="display: inline"/>
Astrobee is a free-flying robot that is designed to operate as a payload inside
the International Space Station (ISS). The Astrobee Robot Software consists of
embedded (on-board) software, supporting tools and a simulator. The Astrobee
Robot Software operates on Astrobee's three internal single board computers and
uses the open-source Robot Operating System (ROS) framework as message-passing
middleware. The Astrobee Robot Software performs vision-based localization,
provides autonomous navigation, docking and perching, manages various sensors
and actuators, and supports user interaction via screen-based displays, light
signaling, and sound. The Astrobee Robot Software enables Astrobee to be
operated in multiple modes: plan-based task execution (command sequencing),
teleoperation, or autonomously through execution of hosted code uploaded by
project partners (guest science). The software simulator enables Astrobee Robot
Software to be evaluated without the need for robot hardware.
</p>

This repository provides flight software and a simulator, both primarily written
in C++. The repository also provides several other utilities, including a tool
for creating maps for localization. A separate repository,
[`astrobee_android`](https://github.com/nasa/astrobee_android), contains the
Java API, which uses the ROS messaging system to communicate with flight
software.

Note that the Astrobee Robot Software is in beta stage. This means that some
features are missing and some functionalities are incomplete. Please consult
[RELEASE](RELEASE.md) for the current list of features and limitations.

## License

Copyright (c) 2017, United States Government, as represented by the
Administrator of the National Aeronautics and Space Administration.
All rights reserved.

The Astrobee platform is licensed under the Apache License, Version 2.0 (the
"License"); you may not use this file except in compliance with the License. You
may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0.

Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the
specific language governing permissions and limitations under the License.

## Documentation
If you would like to generate the astrobee documentation, please run

    doxygen freeflyer.doxyfile

after checking out the code. The documentation will be generated in the doc/html
folder. Open the index.html to get to the main page of the documentation.

## Usage instructions

If you don't have access to NASA's internal network, please see the
[usage instructions.](INSTALL.md)

If you do, please see the [NASA usage instructions.](NASA_INSTALL.md)

## Guest Science

If you are interested in guest science, please do one of the following for more
information.

If you are a non-NASA user, please checkout the astrobee_android nasa github
project (if you followed the usage instructions, you should have checked this
out already). Once that is checked out, please see
[`guest_science/readme.md`](https://github.com/nasa/astrobee_android/blob/HEAD/guest_science/readme.md)
located in the `astrobee_android/guest_science folder`.

If you are a NASA user, please make sure the `android` submodule was checked out
and then visit `submodules/android/guest_science/readme.md`.
