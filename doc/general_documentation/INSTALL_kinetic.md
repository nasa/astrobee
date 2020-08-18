\page install-kinetic Install in Ubuntu 16.04 / ROS kinetic

# Usage instructions for non-NASA users

Install the 64-bit version of [Ubuntu 16.04](http://releases.ubuntu.com/16.04)
on a host machine, and make sure that you can checkout and build code.
If you are using a virtual machine, please use VMware. Virtualbox doesn't
support some of our Gazebo plugins.

    sudo apt-get install build-essential git

*Note: You will need 4 GBs of RAM to compile the software. If you don't have
that much RAM available, please use swap space.*

*Note: Please ensure you install Ubuntu 16.04. Running the simulation in Ubuntu 18.04 is possible, but not officially supported, instructions in README_Ubuntu18.md. At this time we do not support
any other operating system or Ubuntu version.*

*Note: Please ensure you install the 64-bit version of Ubuntu. We do not
support running Astrobee Robot Software on 32-bit systems.*

## Machine setup

### Checkout the project source code

At this point you need to decide where you'd like to put the source code
(`SOURCE_PATH`) on your machine:

    export SOURCE_PATH=$HOME/astrobee

First, clone the flight software repository:

    git clone https://github.com/nasa/astrobee.git $SOURCE_PATH


If you are planning to work on guest science stuff, you will also need the
`astrobee_android` repository. You should checkout the repository in the same
directory you checked out the source code in:

    export ANDROID_PATH="${SOURCE_PATH}_android"


Clone the android repository:

    git clone https://github.com/nasa/astrobee_android.git $ANDROID_PATH


### Dependencies

Next, install all required dependencies:
*Note: `root` access is necessary to install the compiled debian packages below*
*Note: Before running this please ensure that your system is completely updated
    by running 'sudo apt-get update' and then 'sudo apt-get upgrade'*

    pushd $SOURCE_PATH
    cd scripts/setup
    ./add_ros_repository.sh
    sudo apt-get update
    cd debians
    ./build_install_debians.sh
    cd ../
    ./install_desktop_16_04_packages.sh
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
By default, the configure script uses the following paths:
  - native build path: `$HOME/astrobee_build/native`
  - native install path: `$HOME/astrobee_install/native`

If you are satisfied with these paths, you can invoke the `configure.sh` without
the `-p` and `-b` options. For the simplicity of the instructions below,
we assume that `$BUILD_PATH` and `$INSTALL_PATH` contain the location of the
build and install path. For example:

    export BUILD_PATH=$HOME/astrobee_build/native
    export INSTALL_PATH=$HOME/astrobee_install/native

### Native build

The configure script prepares your build directory for compiling the code. Note
that `configure.sh` is simply a wrapper around CMake that provides an easy way
of turning on and off options. To see which options are supported, simply run
`configure.sh -h`.

    pushd $SOURCE_PATH
    ./scripts/configure.sh -l -F -D
    popd

If you want to explicitly specify the build and install directories, use
instead:

    ./scripts/configure.sh -l -F -D -p $INSTALL_PATH -b $BUILD_PATH

*Note: Make sure you use the -F and -D flags. If these flags are not used, the
code will not compile. The -F flag is used to turn off building the Picoflex.
This is only needed for running on the robot and is not needed for the
simulator. The -D is used to turn off building the dds bridge. The bridge is
used to communicate with our ground data system and is also not needed for the
simulator.*

## Building the code

To build, run `make` in the `$BUILD_PATH`. Note that depending on your host
machine, this might take in the order of tens of minutes to complete the first
time round. Future builds will be faster, as only changes to the code are
rebuilt, and not the entire code base.

    pushd $BUILD_PATH
    make -j2
    popd

If you configured your virtual machine with more than the baseline resources,
you can adjust the number of threads (eg. -j4) to speed up the build.

For more information on running the simulator and moving the robot, please see
the [simulation instructions](simulation/sim_overview.md).

## Ubuntu 18.04 Install

Ubuntu 18.04 is currently not officially supported
