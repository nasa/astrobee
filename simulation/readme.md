\page sim-readme Simulation

This package provides the code required to simulate multiple Free-Flyers operating within the International Space Station (ISS). Our code is written as plugins for Gazebo, an open source robot simulator. In essence the plugins mimic the ros messages and services provided by real hardware, and as a result they act as drop-in replacements for the true hardware. Running a simulation is therefore as simple as loading all flight software subsystems except the hardware drivers, and spawning simulated hardware in stead of real drivers.

There is once key exception to the rule above: although we do simulate cameras, we do not simulate any vision-based localization. That is, for sparse mapping, optical flow and AR target tracking, instead of running the feature detectors and localization algorithms on the images produced by the simulator, we draw features from the intersection of random rays cast through a virtual image plane with the environment. This approach provides immense performance gains that enables us to run simulations at many times wall clock speed.

At its core, Gazebo provides a headless server which loads a collection of shared library plugins and performs the simulation itself. One can optionally load the client interface which provides a high-fidelity rendering of the world, and an interface that enables basic interaction with the elements.

Note that if the simulation is run any speed over real-time then the NavCam, DockCam, and SciCam plugins are disabled. We do this because there is large computational overhead required to generate the images: they are wide-angle cameras and hence require multiple projections to be taken and fused, which does not scale well as you increase the requested speed multiplier.

## Obtaining media

If you you are not cross-compiling, then the build process started with "make" will download and extract a correct version of the media automatically for you. This is done by the *description* package [\ref media], as the media is shared between rviz and Gazebo. To see if the media was installed correctly, check that there are subdirectories in the *description/media* directory.

## Running the Simulator

Please see the \subpage running-the-sim documentation.


## Advanced Simulator Information

If you would like more information on running localization only mode,
collisions, performance, frame consistency, debugging, changing properties, or
the Astrobee specific plugins, please see the \subpage advanced-sim-info.

For common simulation issues and workarounds, check \subpage sim-issues
