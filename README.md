# Astrobee Robot Software

### About

<p>
<img src="doc/images/astrobee.png" srcset="../images/astrobee.png 1x" 
  title="Astrobee" align="right" style="display: inline"/>
Astrobee is a free-flying robot designed to operate as a payload inside
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

The Astrobee Robot Software is in a beta stage. This means that some
features are incomplete, and extensive changes can be expected. Please consult the
[release](https://nasa.github.io/astrobee/html/md_RELEASE.html) for the current list of features and limitations.

### Usage instructions

To install and use astrobee, please see the
[usage instructions](https://nasa.github.io/astrobee/html/md_INSTALL.html).

### Contributors

The Astrobee Robot Software is open source, and we welcome contributions
from the public. Please submit pull requests to the develop branch.
For us to merge any pull requests, we must request that contributors sign and submit a
[Contributor License Agreement](https://www.nasa.gov/sites/default/files/atoms/files/astrobee_individual_contributor_license_agreement.pdf)
due to NASA legal requirements. Thank you for your understanding.

### Documentation

To view all the Astrobee documentation, please visit [documentation](https://nasa.github.io/astrobee/documentation.html).

If you want to perform research using the astrobee platform, a good tutorial guide is ["A Brief Guide to Astrobee’s Flight Software"](https://github.com/albee/a-brief-guide-to-astrobee/raw/master/a_brief_guide_to_astrobee_v1.0.pdf). This will teach you what Astrobee is, how the robot works, how to make your own package, and much more!

For more information, read the Astrobee [publications](https://www.nasa.gov/content/research-publications-0).
Learning about the Astrobee [platform](https://www.nasa.gov/sites/default/files/atoms/files/bualat_spaceops_2018_paper.pdf),
[software](https://www.nasa.gov/sites/default/files/atoms/files/fluckiger2018astrobee.pdf),
and [localization](https://www.nasa.gov/sites/default/files/atoms/files/coltin2016localization.pdf)
are good starting points.

### Guest Science

If you are interested in guest science, please checkout the astrobee_android nasa github
project (if you followed the usage instructions, you should have checked this
out already). Once that is checked out, please see
[`astrobee_android/README.md`](https://github.com/nasa/astrobee_android/blob/master/README.md)
located in the `astrobee_android/` folder.

### License

Copyright (c) 2017, United States Government, as represented by the
Administrator of the National Aeronautics and Space Administration.
All rights reserved.

The Astrobee platform is licensed under the Apache License, Version 2.0 (the
"License"); you may not use this file except in compliance with the License. You
may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0.

### Sema Comment

This code is wonderful.

Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the
specific language governing permissions and limitations under the License.
