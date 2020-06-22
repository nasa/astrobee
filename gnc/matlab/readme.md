\page matlab GNC Matlab Code

In this directory, all the matlab, simulink and embedded C++ functions used
therein are stored.

This includes the EKF code, control, FAM, and simulator, as well as associated
testing utilities.

# Dependencies

LUA and Eigen will need to be installed for the MATLAB environment to properly
initialize.  On Linux systems this should be taken care by the standard
installation process.  The model is designed to run in MATLAB/Simulink without
additional toolboxes, although Simulink Coder and Simulink Embedded Coder are
required to generate code.

## For mac installs

Although the full system FSW cannot be run on a mac, MATLAB development and
simulation can be.  LUA should be installed from
https://www.lua.org/download.html  If LUA does not get added to the default
system path,  the following hack seems to work: Right click on the MATLAB
application and go "Show Package Contents".  Then navigate to the bin folder and
open the file matlab in a text editor.  Change the first 3 lines to:

>\#!/bin/sh
>export PATH=$PATH:$HOME/Applications/bin
>\# Copyright 1984-2016 The MathWorks, Inc.

>Where, $HOME/Applications/bin is the directory pointing to your lua binaries.


# The Simulink Code

## Code Organization

There are 2 Simulink models that can be used to simulate the Astrobee system.
The models are identical, except astrobee_control_sim has had the Kalman filter
and vision navigation models removed due to complications with simulating vision
navigation as well as the increased complexity of the Kalman filter.
Additionally, the full up astrobee model has issues running on windows due to
its use of hand coded optimized functions.  For these reasons, it is recomended
that astrobee_control_sim be used whenever possible.
- astrobee.slx file contains the full up control and pose determination software
  as well as an IMU, vision navigation, propulsion, and physics model.
- astrobee_control_sim.slx contains the control software as well as an IMU,
  propulsion, and physics model.

## Open and Run in Matlab

Once the dependencies have been installed as described above simply opening
either the astrobee.slx or astrobee_control_sim.slx file should call all the
necessary initialization files.  For example scenario run files look at the file
'run_ctl_scn01'

## Compilation

The code can be generated from within matlab by running the command
'build_Proto4' or from a Linux terminal by running the shell script
'astrobee_gen_code.sh'

## Embedded C++ Functions

The Kalman filter utilizes several hand coded functions that are highly
optimized using the Eigen library.  These have to be built into mex files in
order to simulate with simulink, this should happen automatically, but requires
the eigen library to be installed and the Matlab mex tools (see matlab
documentation on mex) to be configured to use an installed compiler.  

## LUA Configuration in Matlab

Tunable parameters are managed through LUA scripts, so as to be consistent with
the rest of the Astrobee software.  The GN&C tunable parameters are located in
the freeflyer/astrobee/config/gnc.config file.  The matlab file tunable_init
runs the LUA script and imports the variables into the MATLAB base workspace.
Note, tunable parameters utilized by the GN&C system must have the tun_ prefix
in order to be properly autocoded as tunable variables.
