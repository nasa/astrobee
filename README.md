# Astrobee Robot Software

### About

<img src="doc/images/astrobee.png" srcset="../images/astrobee.png 1x"
  title="Astrobee" align="right" style="display: inline"/>
Three NASA [Astrobee free-flying robots](https://nasa.gov/astrobee/) have been operating inside
the International Space Station (ISS) since 2019. This [`astrobee`](https://github.com/nasa/astrobee) repository contains source code for the Astrobee Robot Software, consisting of
the flight software that runs onboard the Astrobee robots, a software simulator, and supporting tools, primarily written in C++.

Astrobee's flight software performs vision-based localization,
provides autonomous navigation, docking and perching, manages various sensors
and actuators, and supports human-robot interaction with co-located astronauts via screen-based displays, light
signaling, and sound. The flight software is hosted on each Astrobee's three internal single board computers and
uses the open-source [Robot Operating System (ROS)](https://www.ros.org/) framework as message-passing
middleware.  It provides a high-level [Astrobee Command API](https://nasa.github.io/astrobee/html/command_dictionary.html) for controlling the robot and has multiple operating modes. It can execute a plan (command sequence), individual operator commands (teleoperation), or commands from guest science code running onboard Astrobee.

The Astrobee Robot Software simulator, built using [ROS](https://www.ros.org/) and [Gazebo](http://gazebosim.org/), enables the flight software to be evaluated without the need for robot hardware. The supporting tools include a tool that processes ISS imagery to [build maps for Astrobee localization](https://nasa.github.io/astrobee/html/sparsemapping.html), along with many others.

Released separately, the Astrobee ground data system (GDS) includes Astrobee control station software that communicates with Astrobee flight software via the Data Distribution Service (DDS) network protocol over the ISS Ku-IP space-to-ground link. The control station is written primarily in Java using the Eclipse RCP framework. Source code is in the [`astrobee_gds`](https://github.com/nasa/astrobee_gds) repository, or you can download the [binary release](https://software.nasa.gov/software/ARC-17994-1B).

Together, the Astrobee Robot and Ground Software were the NASA Software of the Year Award Runner-Up in 2020!

The Astrobee Robot Software remains a work in progress. Please consult the
[release notes](https://nasa.github.io/astrobee/html/md_RELEASE.html) for the current list of features and limitations.

### Usage

[Instructions on installing and using the Astrobee Robot Software](https://nasa.github.io/astrobee/html/md_INSTALL.html).

### Contributing

The Astrobee Robot Software is open source, and we welcome contributions
from the public. Please submit pull requests to the [`develop`](https://github.com/nasa/astrobee/tree/develop) branch. The code must follow the [Astrobee code style](https://nasa.github.io/astrobee/html/astrobee-code-style.html).
For us to merge any pull requests, we must request that contributors sign and submit a
[Contributor License Agreement](https://www.nasa.gov/sites/default/files/atoms/files/astrobee_individual_contributor_license_agreement.pdf)
due to NASA legal requirements. Thank you for your understanding.

### Documentation

[Extensive documentation is auto-generated from the contents of this repository.](https://nasa.github.io/astrobee/documentation.html)

["A Brief Guide to Astrobee’s Flight Software"](https://github.com/albee/a-brief-guide-to-astrobee/raw/master/a_brief_guide_to_astrobee_latest.pdf) is a good tutorial, with a particular emphasis on the advanced topic of modifying Astrobee's flight software to enable Guidance, Navigation, & Control (GN&C) research. (Note that most guest science can be implemented as an app that uses the [Astrobee Command API](https://nasa.github.io/astrobee/html/command_dictionary.html) without modifying the flight software.)

For more information, read [Astrobee-related publications](https://www.nasa.gov/content/research-publications-0).
Learning about the Astrobee [platform](https://www.nasa.gov/sites/default/files/atoms/files/bualat_spaceops_2018_paper.pdf),
[software](https://www.nasa.gov/sites/default/files/atoms/files/fluckiger2018astrobee.pdf),
and [localization](https://www.nasa.gov/sites/default/files/atoms/files/coltin2016localization.pdf)
are good starting points.

### Guest Science

The ISS Astrobee Facility maintains a collection of [resources for guest scientists](https://www.nasa.gov/content/guest-science-resources) interested in conducting research with Astrobee, including the [Astrobee Guest Science Guide](https://www.nasa.gov/sites/default/files/atoms/files/irg-ff029-astrobee-guest-science-guide.pdf) overview.

Guest science code that runs onboard Astrobee is usually hosted as an Android Java app running on Astrobee's high-level processor (HLP). Guest science apps can use the [Astrobee Command API](https://nasa.github.io/astrobee/html/command_dictionary.html) through its Java bindings. The [`astrobee_android`](https://github.com/nasa/astrobee_android) repository contains source code for Astrobee services that run on the HLP, example guest science apps, as well as a [Guest Science Readme](https://github.com/nasa/astrobee_android/blob/master/guest_science_readme.md) and [Guest Science Developer Guide](https://github.com/nasa/astrobee_android/blob/master/gs_developer_guide.md) focused on HLP guest science apps.

### License

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
