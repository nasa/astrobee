\page install-NASA Install for NASA users

# Usage instructions for NASA users

Make sure your system is up-to-date and:

    sudo apt-get install build-essential git

## Machine setup

### Username

If you are using a VM with a username that does not match your NDC username,
please configure the following variable:

    export NDC_USERNAME=your_ndc_username


### Access to the Astrobee Debian server

The custom debian packages are currently distributed by the
`astrobee.ndc.nasa.gov server`. This server is currently on the ARC TI
private network. It is *critical* to be able to reach this server to
install the pre-built custom debian. If none of the solutions below
allow you to reach `astrobee.ndc.nasa.gov`, you can use the
instructions on how to build the Debian dependencies manually
following the INSTALL.md instructions (Dependencies section).

#### If on the ARC TI private network

This is the typical case for all wired computers in ARC TI, and simplifies
your life greatly.

Verify that you are in this situation with the command below should succeed
(remove the Release.gpg file after being fetched).

    wget -v http://astrobee.ndc.nasa.gov/software/dists/xenial/Release.gpg

Before running the scripts in `scripts/setup` below, set this variable:

    export NO_TUNNEL=1

#### If not on the ARC TI private network

If you are outside the NASA ARC private network, to reach `astrobee.ndc.nasa.gov`:

  1. Use VPN to act like if you were inside the ARC TI private network and 
     obtain the correct kerberos credentials inside the VM with the following
     command (note the capitalization): `kinit $NDC_USERNAME@NDC.NASA.GOV`
  2. setup your `.ssh/config` to do ssh forwarding. A tutorial on this method
  is available at: https://babelfish.arc.nasa.gov/trac/freeflyer/wiki/SSHSetup

For either solution, please verify that you can SSH to `m.ndc.nasa.gov` without
entering your password (`m` can be used to tunnel to `astrobee.ndc.nasa.gov`):

   `ssh $NDC_USERNAME@m.ndc.nasa.gov`
   
The command should succeed without entering your password. Once this is verified,
exit this session on `m` with `<ctrl>+D`.

- These notes apply to `install_desktop_16.04_packages.sh` and `make_chroot.sh`


### Checkout the project source code

At this point you need to decide where you'd like to put the source code
(`ASTROBEE_WS`) on your machine:

    export ASTROBEE_WS=$HOME/astrobee

First, clone the flight software repository:

    git clone https://github.com/nasa/astrobee.git --branch develop $ASTROBEE_WS/src
    pushd $ASTROBEE_WS/src
    git submodule update --init --depth 1 description/media
    git submodule update --init --depth 1 submodules/platform

You can either choose which optional submodules to clone and log depth with:

    git submodule update --init --depth 1 submodules/android
    git submodule update --init --depth 1 submodules/avionics

Or checkout all the submodules as:

    git submodule update --init --depth 1

(Note: re-enter your username and password for every submodule that is cloned)
The android module is necessary for guest science code; the avionics and platform
module is used when cross-compiling to test on the robot hardware.

### Dependencies

Next, install all required dependencies:

*Note: `root` access is necessary to install the compiled debian packages below*

*Note: Before running this please ensure that your system is completely updated
by running 'sudo apt-get update' and then 'sudo apt-get upgrade'*

    pushd $ASTROBEE_WS
    cd src/scripts/setup
    ./add_local_repository.sh
    ./add_ros_repository.sh
    ./install_desktop_packages.sh
    popd


## Configuring the build

### Note for build setup

When compiling, the `$WORKSPACE_PATH` defines where the `devel`, `build`, `logs` and
`install` directories are created. If you want to customize the `install` path then the
`$INSTALL_PATH` can be defined. By default, the configure script uses the following paths:

  - native build path (WORKSPACE_PATH): `$ASTROBEE_WS`
  - native install path (INSTALL_PATH): `$ASTROBEE_WS/install`
  - armhf build path (WORKSPACE_PATH):  `$ASTROBEE_WS/armhf`
  - armhf install path (INSTALL_PATH):  `$ASTROBEE_WS/armhf/opt/astrobee`

You should set these values in your shell. 

If you are satisfied with these paths, you can invoke the `configure.sh` without
the `-p` and `-w` options. For the simplicity of the instructions below,
we assume that `$WORKSPACE_PATH` and `$INSTALL_PATH` contain the location of the
build and install path for either `native` or `armhf` platforms.

## Native vs Cross-Compile

At this point you need to decide whether you'd like to compile natively
[`native`] (run code against a simulator) or cross-compile for an ARM
target [`armhf`] (run the code on the robot itself). Please skip to the
relevant subsection.

## Native - Running the code on your computer with simulator

## Native build

The configure script prepares your build directory for compiling the code. Note
that `configure.sh` is simply a wrapper around CMake that provides an easy way
of turning on and off options. To see which options are supported, simply run
`configure.sh -h`.

    pushd $ASTROBEE_WS
    ./src/scripts/configure.sh -l
    source ~/.bashrc
    popd

The configure script modifies your ``.bashrc`` to source ``setup.bash`` for 
the current ROS distribution and to set CMAKE_PREFIX_PATH. It is suggested
to examine it and see if all changes were made correctly.

If you want to explicitly specify the workspace and/or install directories, use
instead:

    ./scripts/configure.sh -l -p $INSTALL_PATH -w $WORKSPACE_PATH

*Note: If a workspace is specified but not an explicit install distectory,
install location will be $WORKSPACE_PATH/install.*


### Building the code

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


## Cross-compile - Running the code on a real robot

In order to do this, you will need to followe the cross-compile build
instructions.

### Cross-compile setup

If you are planning to compile code to run on the robot hardware, you will need
to install a cross-compile chroot and toolchain. Select two directories for
these:

    export ARMHF_CHROOT_DIR=$HOME/arm_cross/rootfs
    export ARMHF_TOOLCHAIN=$HOME/arm_cross/toolchain/gcc

Append these lines to your .bashrc file, as you will need these two variables
every time you cross compile.

Next, download the cross toolchain and install the chroot:

    mkdir -p $ARMHF_TOOLCHAIN
    cd $HOME/arm_cross
    $ASTROBEE_WS/src/submodules/platform/fetch_toolchain.sh
    $ASTROBEE_WS/src/submodules/platform/rootfs/make_chroot.sh xenial dev $ARMHF_CHROOT_DIR

*Note: The last script shown above needs the packages `qemu-user-static` (not
`qemu-arm-static`) and `multistrap` to be installed (can be installed through apt).*

### Cross-compile build

Cross compiling for the robot follows the same process, except the configure
script takes a `-a` flag instead of `-l`.

    pushd $ASTROBEE_WS
    ./src/scripts/configure.sh -a
    popd

Or with explicit build and install paths:

    ./scripts/configure.sh -a -p $INSTALL_PATH -w $WORKSPACE_PATH

*Warning: `$INSTALL_PATH` and `$WORKSPACE_PATH` used for cross compiling HAVE to be
different than the paths for native build! See above for the default values 
for these.*

 Once the code has been built, it also installs the code to
a singular location. CMake remembers what `$INSTALL_PATH` you specified, and
will copy all products into this directory.

### Install the code on the robot

Once the installation has completed, copy the install directory to the robot.
This script assumes that you are connected to the Astrobee network, as it uses
rsync to copy the install directory to `~/armhf` on the two processors. It 
takes the robot name as an argument. Here we use `p4d'.

    pushd $ASTROBEE_WS
    ./src/scripts/install_to_astrobee.sh $INSTALL_PATH p4d
    popd

Here, p4d is the name of the robot, which may be different in your case.

You are now ready to run the code. This code launches a visualization tool,
which starts the flight software as a background process.

    pushd $ASTROBEE_WS
    python ./src/tools/gnc_visualizer/scripts/visualizer --proto4
    popd

## Switching build profiles

To alternate between native and armhf profiles:

    catkin profile set native
    catkin profile set armhf

# Further information

Please refer to the [wiki](https://github.com/nasa/astrobee/wiki).
