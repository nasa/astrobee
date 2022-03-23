\page install-docker Docker build

# Usage instructions for Docker

Here there are instructions on how to build and run the FSW using Docker.
First, clone the flight software repository and media:

    git clone https://github.com/nasa/astrobee.git $SOURCE_PATH
    git submodule update --init --depth 1 description/media

The docker image for the astrobee FSW is divided throughout 2 docker files. 

Available docker files:

- `astrobee_base.Dockerfile` - Contains installation of all astrobee dependencies the Ubuntu + ROS setup.
- `astrobee.Dockerfile` - Builds the astrobee FSW code on top of astrobee_base.
- `astrobee_quick.Dockerfile` - Builds the astrobee FSW code using a previous astrobee image as a build cache. This dramatically speeds up build times for small changes.

The Docker files accept the following version args (note that they must match up):

- `UBUNTU_VERSION` - The version of Ubuntu to use. Valid values are "16.04", "18.04", and "20.04".
- `ROS_VERSION` - The version of ROS to use. Valid values are "kinetic", "melodic", and "noetic".
- `PYTHON` - The version of Python to use. Valid values are "" (an empty string representing Python 2) and "3".

If `UBUNTU_VERSION` is `"16.04"`, `ROS_VERSION` and `PYTHON` must be `"kinetic"` and `""` respectively.
If `UBUNTU_VERSION` is `"18.04"`, `ROS_VERSION` and `PYTHON` must be `"melodic"` and `""` respectively.
If `UBUNTU_VERSION` is `"20.04"`, `ROS_VERSION` and `PYTHON` must be `"neotic"` and `"3"` respectively.

The Docker files also accept args to use local or container registry images.

- `REMOTE` - The repository where the dockerfile should derive its base. Valid values are `astrobee` (the default for local builds) or `ghcr.io/nasa` (the official repository).
- `REMOTE_CACHED` - (Only for `astrobee_quick.dockerfile`, defaults to `${REMOTE}`). The repository for the build cache image. Valid values are `astrobee` (the default for local builds) or `ghcr.io/nasa` (the official repository).

## Building the docker images

The fastest way to start running the software is to fetch a remote
docker image by using the `--remote` option with the `run_headless.sh`
and `run.sh` scripts described below. Building the docker images is
not required.

However, if you need to build a local copy of the docker images, run:
    
    ./build.sh

By default, the build script will automatically detect your host's
Ubuntu OS version and configure the docker image to use the same
version using the Dockerfile variables `UBUNTU_VERSION`,
`ROS_VERSION`, and `PYTHON`.

However, there is no requirement for the host OS and the docker image
OS to match.  You can override the default and select a specific
docker image Ubuntu version by specifying `--xenial`, `--bionic`, or
`--focal` for Ubuntu 16.04, 18.04, or 20.04 docker images,
respectively.

## Run commands in the container

To start an interactive shell in the docker container, allowing you to
execute arbitrary commands:

    ./run_headless.sh --remote

As with `build.sh`, by default, the docker image OS version will be
configured to match your host's OS version, but you can override that
by specifying the `--xenial`, `--bionic`, or `--focal` option for
Ubuntu 16.04, 18.04, or 20.04 docker images, respectively.

With the `--remote` argument, it will fetch and run a pre-built
Astrobee docker image. Omit the `--remote` argument to run using a
docker image built locally by `./build.sh`.

As the script name suggests, this script doesn't set up the X display,
so it can be used to run a docker container from within a headless
VM host. However, graphical applications will not be usable within the
resulting shell.

## Run unit tests in the container

The `run_tests.sh` script runs within the container and is designed to
closely mimic the testing actions in `test_astrobee.Dockerfile`, which is
invoked by the `astrobee` GitHub continuous integration workflow. You can
use it to replicate and debug failed CI tests.

Example usage:

    host$ ./run_headless.sh --remote
    docker# (cd /src/astrobee && catkin build)  # recompile local changes
    docker# /src/astrobee/src/scripts/docker/run_tests.sh [package]

The package argument is optional. By default, it tests all packages.

If debugging a CI failure that is specific to a particular OS version,
run `run_headless.sh` with the `--xenial`, `--bionic`, or `--focal`
options to select the right OS version to replicate the failure.

## Run the Astrobee simulator in the container

To run the Astrobee simulator in the container:

    ./run.sh --remote

It takes the same arguments as `run_headless.sh` for selecting the image version to use.

To add arguments to the launch file in addition to `dds:=false robot:=sim_pub` you can do instead:

    ./run.sh --remote --args "rviz:=true sviz:=true"

*Note: You have to install the nvidia-container-toolkit for the gazebo simulation to run properly*

To open another terminal inside the docker container:

    docker exec -it astrobee bash

Once inside the container, don't forget to source astrobee to have access to all the commands:

    source /build/astrobee/devel/setup.bash

To shutdown the docker container, run:

    ./shutdown.sh


## Cross-compile Astrobee (NASA users only)

To cross-compile Astrobee, you will need
to install a cross-compile chroot and toolchain. Select two directories for
these:

    export ARMHF_CHROOT_DIR=$HOME/arm_cross/rootfs
    export ARMHF_TOOLCHAIN=$HOME/arm_cross/toolchain/gcc

Append these lines to your .bashrc file, as you will need these two variables
every time you cross compile.

Next, download the cross toolchain and install the chroot:

    mkdir -p $ARMHF_TOOLCHAIN
    cd $HOME/arm_cross
    $SOURCE_PATH/submodules/platform/fetch_toolchain.sh
    $SOURCE_PATH/submodules/platform/rootfs/make_xenial.sh dev $ARMHF_CHROOT_DIR

From the root of the repository, run:

    ./scripts/docker/cross_compile/cross_compile.sh

The code will be cross-compiles inside the docker container in /opt/astrobee, and
can be copied to the robot.

After the debian files are generated inside the docker image, they are copied to
in order of priority:
1) To the INSTALL_PATH, if defined
2) To ${DIR}/../../../../astrobee_install/armhf/, if the directoy already exists,
where $DIR is the directory where the cross_compile.sh script is located.
3) To $HOME/astrobee_build/armhf otherwise

## Building an Astrobee Debian (NASA users only)

This step assumes that the cross compile setup was successful and that ARMHF_CHROOT_DIR
and ARMHF_TOOLCHAIN are defined. If not, please follow the cross-compile instructions.

To generate the debians, from the root of the repository, run:

    ./scripts/docker/cross_compile/debian_compile.sh

After the debian files are generated inside the docker image, they are copied to the
folder before the root of the repository, ${DIR}/../../../../, where $DIR is the directory where the debian_compile.sh script is located.
