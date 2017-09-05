# Usage instructions for NASA users

Install the 64-bit version of [Ubuntu 16.04](http://releases.ubuntu.com/16.04)
on a host machine, and make sure that you can checkout and build code.
If you are using a virtual machine, please use VMware. Virtualbox doesn't
support some of our Gazebo plugins.

    sudo apt-get install build-essential git

*Note: Please ensure you install the 64-bit version of Ubuntu. We do not
support running Astrobee Robot Software on 32-bit systems.*

## Machine setup

#### General notes before running the scripts below
- The custom debian packages are currently stored on the "volar" server. So you
  need to have credentials on `volar` to run these scripts.
- If you are using a VM with a username that does not match your NDC username,
  you have two options:
  - Set correctly the environment variable `NDC_USERNAME`
  - Or set a ssh config that specify the right username for volar.
- If you are outside the NASA ARC private network, you need to:
  - Either use VPN to act like if you were inside the ARC TI private network
  - Or setup your `.ssh/config` to do ssh forwarding. A tutorial on this method
  is available at: https://babelfish.arc.nasa.gov/trac/freeflyer/wiki/SSHSetup
- These notes apply to `add_local_repository.sh` and `make_xenial.sh`

### Checkout the project source code

At this point you need to decide where you'd like to put the source code
(`SOURCE_PATH`) on your machine:

    export SOURCE_PATH=$HOME/freeflyer

First, clone the flight software repository:

    git clone --recursive https://$USER@babelfish.arc.nasa.gov/git/freeflyer \
        --branch develop $SOURCE_PATH

### Dependencies

Next, install all required dependencies:

    pushd $SOURCE_PATH
    cd scripts/platform/desktop
    ./add_local_repository.sh
    ./add_ros_repository.sh
    ./install_desktop_16_04_packages.sh
    popd

#### Extra options to install the dependencies

- If you do not want to configure you `.ssh/config` to just get the
dependencies, you can use the `NDC_USERNAME` variable.
- By default, the custom debians are installed in `$SOURCE_PATH/.astrobee_deb`.
If you prefer to install them at a different location, you can use the
`ARS_DEB_DIR` variable.

```
export NDC_USERNAME=jdoe
export ARS_DEB_DIR=$HOME/freeflyer_debs
./add_local_repository.sh
```

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
    $SOURCE_PATH/scripts/platform/fetch_toolchain.sh
    $SOURCE_PATH/scripts/platform/rootfs/make_xenial.sh dev $ARMHF_CHROOT_DIR

## Configuring the build

At this point you need to decide whether you'd like to compile natively
[`native`] (run code against a simulator) or for an ARM target [`armhf`] (run
the code on the robot itself). Please skip to the relevant subsection.

### Note for both builds setup
By default, the configure script uses the following paths:
  - native build path: `$HOME/freeflyer_build/native`
  - native install path: `$HOME/freeflyer_install/native`
  - armhf build path: `$HOME/freeflyer_build/armhf`
  - armhf install path: `$HOME/freeflyer_install/armhf`

If you are satisfied with these paths, you can invoke the `configure.sh` without
the `-p` and `-b` options. For the simplicity of the instructions below,
we assume that `$BUILD_PATH` and `$INSTALL_PATH` contain the location of the
build and install path for either `native` or `armhf` platforms.

### Native build

The configure script prepares your build directory for compiling the code. Note
that `configure.sh` is simply a wrapper around CMake that provides an easy way
of turning on and off options. To see which options are supported, simply run
`configure.sh -h`.

    pushd $SOURCE_PATH
    ./scripts/configure.sh -l
    popd

If you want to explicitly specify the build and install directories, use
instead:

    ./scripts/configure.sh -l -p $INSTALL_PATH -b $BUILD_PATH

### Cross-compile build

Cross compiling for the robot follows the same process, except the configure
script takes a `-a` flag instead of `-l`.

    pushd $SOURCE_PATH
    ./scripts/configure.sh -a
    popd

Or with explicit build and install paths:

    ./scripts/configure.sh -a -p $INSTALL_PATH -b $BUILD_PATH

*Warning: `$INSTALL_PATH` and `$BUILD_PATH` used for cross compiling HAVE to be
different than the paths for native build!*

## Building the code

To build, run `make` in the `$BUILD_PATH`. Note that depending on your host
machine, this might take in the order of tens of minutes to complete the first
time round. Future builds will be faster, as only changes to the code are
rebuilt, and not the entire code base.

    pushd $BUILD_PATH
    make -j6
    popd

*Note: `$BUILD_PATH` above is either the path for native build or armhf build,
whatever you currently are doing.*

## Running a simulation

In order to run a simulation you must have build natively. You will need to
first setup your environment, so that ROS knows about the new packages provided
by Astrobee flight software:

    pushd $BUILD_PATH
    source devel/setup.bash
    popd

After this command has completed, you should be able to run a simulator from any
directory in your Linux filesystem. So, for example, to start a simulation of a
single Astrobee in the Granite Lab, run the following:

    roslaunch astrobee simulator.launch

This command tells ROS to look for the `simulation.launch` file provided by the
`astrobee` package, and use roslaunch to run it. Internally, ROS maintains a
cache of information about package locations, libraries and executables. If you
find that the above command doesn't work, try rebuilding the cache:

    rospack profile

## Running the code on a real robot

In order to do this, you will need to have followed the cross-compile build
instructions. Once the code has been built, you also need to install the code to
a singular location. CMake remembers what `$INSTALL_PATH` you specified, and
will copy all products into this directory.

    pushd $BUILD_PATH
    make install
    popd

Once the installation has completed, copy the install directory to the robot.
This script assumes that you are connected to the Astrobee network, as it uses
rsync to copy the install directory to `~/armhf` on the two processors.

    pushd $SOURCE_PATH
    ./scripts/install_to_astrobee.sh $INSTALL_PATH
    popd

You are now ready to run the code. This code launches a visualization tool,
which starts the flight software as a background process.

    pushd $SOURCE_PATH
    python ./tools/gnc_visualizer/scripts/visualizer --proto4
    popd

# Further information

Please refer to the [wiki](https://babelfish.arc.nasa.gov/trac/freeflyer).
