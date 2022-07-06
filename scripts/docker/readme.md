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

## Optional: Build the docker image

The fastest way to start running the software is to fetch a remote
docker image by using the `--remote` option with the `run.sh` script
described below. Building the docker images is not required.

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

## Run the Astrobee simulator in the container

*Note: To run the simulator within a docker container, the container
generally needs to have hardware-accelerated graphics support. This
requires compatible graphics hardware and may also require installing
a special support package, such as `nvidia-container-toolkit`.*

To run the Astrobee simulator in the container:

    ./run.sh --remote

The default arguments to the sim are `dds:=false robot:=sim_pub`. To add more arguments:

    ./run.sh --remote --args "rviz:=true sviz:=true"

To open another terminal inside the same docker container:

    docker exec -it astrobee /astrobee_init.sh bash

(The `/astrobee_init.sh` script configures the shell in the container
to know about the Astrobee development environment, then runs the
specified command, in this case `bash`, to get an interactive shell.)

To shutdown the docker container, run:

    ./shutdown.sh

## Run other commands in the container

Besides the simulator, you can also run arbitrary commands with `run.sh`. To
get an interactive shell:

    ./run.sh --remote -- bash

Or run any other command:

    ./run.sh --remote -- rosmsg info std_msgs/Header

(The `--` separator is usually not necessary, but makes the `run.sh`
argument parsing more predictable.)

As with `build.sh`, by default, the docker image OS version will be
configured to match your host's OS version, but you can override that
by specifying the `--xenial`, `--bionic`, or `--focal` option for
Ubuntu 16.04, 18.04, or 20.04 docker images, respectively.

Use `--remote` to fetch and run a pre-built Astrobee docker
image. (Omit `--remote` to run using a docker image built locally by
`./build.sh`.)

Use `--no-display` to skip setting up the X graphical environment. The
script will also automatically skip X setup if it detects you are in a
headless host environment. Of course, graphical applications will not
be available.

Use `--mount` to mount your local source checkout into its standard
location in the docker container, where it will override the copy
embedded within the container. This is ideal for making changes in
your host environment, then quickly testing them in the container.

## Run unit tests in the container

The `run_tests.sh` script (located in the parent `scripts` folder) is
designed to run within the container and closely mimic the testing
actions in `test_astrobee.Dockerfile`, which is invoked by the
`astrobee` GitHub continuous integration workflow. You can use it to
replicate and debug failed CI tests.

Example usage:

    host$ ./run.sh --remote --mount
    docker# (cd /src/astrobee && catkin build [package])  # recompile local changes
    docker# /src/astrobee/src/scripts/run_tests.sh [package]

The package argument is optional. The default is to build/test all
packages.

If debugging a CI failure that is specific to a particular OS version,
remember to pass `run.sh` the `--xenial`, `--bionic`, or `--focal`
option to select the right OS version to replicate the failure.

Note: integration tests that use Gazebo simulation will be silently
disabled when running with `--no-display`. To ensure complete testing,
run in a host environment that supports X graphics.

If you care about higher-fidelity replication of CI problems and are
willing to wait through a full `astrobee` build, you can also use
`build.sh` to invoke `test_astrobee.Dockerfile` itself, like this:

    ./build.sh --remote astrobee test_astrobee

Or, if you made changes that affect `astrobee_base.Dockerfile`, you
will need to rebuild that locally as well:

    ./build.sh astrobee_base astrobee test_astrobee

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
