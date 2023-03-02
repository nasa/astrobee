\page install-nonNASA Install for general users

# Usage instructions for non-NASA users

Make sure your system is up-to-date and:

    sudo apt-get install build-essential git

## Machine setup

### Checkout the project source code

At this point you need to decide where you'd like to put the source code
(`ASTROBEE_WS`) on your machine:

    export ASTROBEE_WS=$HOME/astrobee

First, clone the flight software repository and media:

    git clone https://github.com/nasa/astrobee.git $ASTROBEE_WS/src
    pushd $ASTROBEE_WS/src
    git submodule update --init --depth 1 description/media
    popd

If you are planning to work with guest science code, you will also need the
`astrobee_android` repository. You should checkout the repository as a submodule:

    git submodule update --init --depth 1 submodules/android

### Dependencies

Next, install all required dependencies:

*Note: `root` access is necessary to install the compiled debian packages below*

*Note: Before running this please ensure that your system is completely updated
by running 'sudo apt-get update' and then 'sudo apt-get upgrade'*

    pushd $ASTROBEE_WS
    cd src/scripts/setup
    ./add_ros_repository.sh
    sudo apt-get update
    cd debians
    ./build_install_debians.sh
    cd ../
    ./install_desktop_packages.sh
    sudo rosdep init
    rosdep update
    popd


**Important**: you can safely ignore the following error messages, as they are simply letting you know that certain libraries cannot be found. These libraries are for internal NASA use only, and are not required by public users provided that software is launched with DDS disabled.

    E: Unable to locate package libroyale1
    E: Unable to locate package rti
    E: Unable to locate package libmiro0
    E: Unable to locate package libsoracore1
    E: Unable to locate package libroyale-dev
    E: Unable to locate package rti-dev
    E: Unable to locate package libsoracore-dev
    E: Unable to locate package libmiro-dev

## Configuring the build

### Note for the build setup

When compiling, the `$WORKSPACE_PATH` defines where the `devel`, `build`, `logs` and
`install` directories are created. If you want to customize the `install` path then the
`$INSTALL_PATH` can be defined. By default, the configure script uses the following paths:

  - native build path: `$ASTROBEE_WS/build`
  - native install path: `$ASTROBEE_WS/install`

If you are satisfied with these paths, you can invoke the `configure.sh` without
the `-p` and `-w` options. For the simplicity of the instructions below,
we assume that `$WORKSPACE_PATH` and `$INSTALL_PATH` contain the location of the
build and install path. For example:

    export WORKSPACE_PATH=$ASTROBEE_WS
    export INSTALL_PATH=$ASTROBEE_WS/install

### Native build

The configure script prepares your build directory for compiling the code. Note
that `configure.sh` is simply a wrapper around CMake that provides an easy way
of turning on and off options. To see which options are supported, simply run
`configure.sh -h`.

    pushd $ASTROBEE_WS
    ./src/scripts/configure.sh -l -F -D
    source ~/.bashrc
    popd

If you run a Zsh session, then

    pushd $ASTROBEE_WS
    ./src/scripts/configure.sh -l -F -D
    source ~/.zshrc
    popd

The configure script modifies your ``.bashrc``/``.zshrc`` to source ``setup.bash``/``setup.zsh`` for 
the current ROS distribution and to set CMAKE_PREFIX_PATH. It is suggested
to examine it and see if all changes were made correctly.

If you want to explicitly specify the workspace and install directories, use
instead:

    ./src/scripts/configure.sh -l -F -D -p $INSTALL_PATH -w $WORKSPACE_PATH

*Note: If a workspace is specified but not an explicit install distectory,
install location will be $WORKSPACE_PATH/install.*

*Note: Make sure you use the -F and -D flags. If these flags are not used, the
code will not compile. The -F flag is used to turn off building the Picoflex.
This is only needed for running on the robot and is not needed for the
simulator. The -D is used to turn off building the dds bridge. The bridge is
used to communicate with our ground data system and is also not needed for the
simulator.*

## Building the code

To build, run `catkin build` in the `$WORKSPACE_PATH`. Note that depending on your host
machine, this might take in the order of tens of minutes to complete the first
time round. Future builds will be faster, as only changes to the code are
rebuilt, and not the entire code base.

    pushd $ASTROBEE_WS
    catkin build
    popd

Note: In low-memory systems, it is common to run out of memory while trying to compile
ARS, which triggers a compilation error mentioning "arm-linux-gnueabihf-g++: internal 
compiler error: Killed (program cc1plus)". A contributing factor is that
catkin build by default runs multiple jobs in parallel based on the number of cores
available in your environment, and all of these jobs draw on the same memory resources.
If you run into this compile error, try compiling again with the -j1 option to restrict
catkin to running one job at a time.

For more information on running the simulator and moving the robot, please see the \ref running-the-sim.


## Cross Compiling

Please contact your Astrobee point of contact if you need to cross compile the
code.

For more information on running the simulator and moving the robot, please see
the \ref sim-readme.
