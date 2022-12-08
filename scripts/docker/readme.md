\page install-docker Developing with Docker

# System requirements

The instructions below assume your host OS is Ubuntu 16.04, 18.04, or 20.04. As of this writing, Ubuntu 20.04 is the preferred host OS for most Astrobee developers to use, at least until you get to the point of needing to cross-compile your code for the robot hardware.

Make sure you have Docker installed in your system following the [installation instructions](https://docs.docker.com/engine/install/ubuntu/) and [post-installation steps for Linux](https://docs.docker.com/engine/install/linux-postinstall/).

# Clone the Astrobee Robot Software

The instructions below assume you have cloned the software with:

    export ASTROBEE_WS=$HOME/astrobee  # your choice where
    git clone https://github.com/nasa/astrobee.git $ASTROBEE_WS/src
    cd $ASTROBEE_WS/src
    git submodule update --init --depth 1 description/media

# Quick start

## Quick start run

You can immediately run a software simulation of Astrobee as follows:

    cd $ASTROBEE_WS/src
    ./scripts/docker/run.sh --remote

Invoking `run.sh --remote` calls Docker with the right arguments such that it will download the latest pre-built complete Astrobee Robot Software image from our official Docker image repository, start it as a container, and invoke a default command inside the container (which runs the software simulation).

## Quick start build (if needed)

The fastest way to recompile the ARS software to reflect your local changes is:

    ./scripts/docker/build.sh --remote astrobee_quick

Invoking `build.sh --remote` calls Docker with the right arguments such that it will download the latest pre-built complete Astrobee Robot Software image from our official Docker image repository. Then it will build the `astrobee_quick` target, which does a quick recompile of the ARS software to reflect your changes, accelerated by using previously cached compilation results.

# Overview of Docker scripts

The Astrobee Robot Software provides two main support scripts for working with Docker.

The `build.sh` script builds new ARS Docker images, with many configuration options described later. As discussed in the "Quick start run" section, you can always start by running one of our pre-built images. You only need to build your own new image if you've modified the software.

The `run.sh` script (optionally downloads and) runs ARS Docker images.

The two scripts have similar options to make it easy to run the specific image you just built.

# Building Docker images

## Build stages

Among the `build.sh` command-line arguments, you can list targets you want to build. Some of the targets correspond to Docker images at different build stages:

- `astrobee_base` - Starts with a baseline Ubuntu distribution and installs the external dependencies of ARS, including a ROS distribution and many external libraries built from source.
- `astrobee` - Starts with `astrobee_base` and builds the ARS software.
- `astrobee_quick` - Starts with `astrobee` and does a quick rebuild of the ARS software.
- `test_astrobee` - Starts with `astrobee` and runs the subset of ARS tests that are compatible with the Docker environment.

For all of these cases, each new build stage starts from the results of the previous build stage. If you run `build.sh --remote`, it will attempt to fetch a pre-built image for the previous build stage from the official Astrobee Docker repository. Without `--remote`, it will assume you built the previous stage yourself and search for the image on your local system.

If you've made changes to ARS, here's how to figure out what `build.sh` target to specify:

- Most commonly, you can build `astrobee_quick`. This should rebuild the ARS instance within your container to reflect your changes, taking advantage of cached results from the most recent build of the `astrobee` image to speed up compilation. The ARS build system is designed to detect when cached results need to be rebuilt by comparing file timestamps. However, the detection is not perfect in all cases, and invalid cache contents can occasionally cause build errors.
- You would most likely build the `astrobee` target when building `astrobee_quick` generates errors that might be due to invalid cache contents. Because building `astrobee` starts from the `astrobee_base` image with no previous ARS compilation results, building it effectively clears the cache. Another good use for the `astrobee` target is in continuous integration workflows where we want to have strict configuration management to make the output products more deterministic.
- As an external developer, you generally shouldn't need to build the `astrobee_base` target. That's a good thing, because building the external libraries from source typically takes hours. As a NASA-internal developer, you may rarely need to build `astrobee_base` if you are modifying `astrobee_base.Dockerfile` to install new external dependencies needed by the `develop` and `master` branches.
- The `test_astrobee` target is most commonly used in continuous integration workflows to check that software changes didn't introduce errors. Because it always runs on top of the `astrobee` image, the test results will not reflect any changes that you compiled only by building `astrobee_quick`. You can manually run the same tests in the `astrobee_quick` container like this:

    $ ./scripts/docker/run.sh -i "quick-" -- bash  # enter astrobee_quick container
    # /src/astrobee/src/scripts/run_tests.sh

In order to execute each of these build stages, `build.sh` invokes `docker build` on the corresponding `Dockerfile` in the `scripts/docker` folder, such as `astrobee_base.Dockerfile`. These files can give you insight into the detailed build steps at each stage.

## Other build targets

The `build.sh` script provides two other targets that are not build stages, but rather actions that you can take with the resulting products:

- `push_astrobee_base` - Pushes the locally built `astrobee_base` image to our official Docker image repository
- `push_astrobee` - Pushes the locally built `astrobee` image to our official Docker image repository

These targets are mostly used in continuous integration workflows.

All pre-built remote images are available on [github here](https://github.com/nasa/astrobee/pkgs/container/astrobee)

## Other build options

By default, the build script will automatically detect your host's Ubuntu OS version and configure the Docker image to use the same version using `Dockerfile` `ARGS`.

However, there is no requirement for the host OS and the Docker image OS to match.  You can override the default and select a specific Docker image Ubuntu version by specifying `--xenial`, `--bionic`, or `--focal` for Ubuntu 16.04, 18.04, or 20.04 docker images, respectively.

For more information about all the build arguments:

    ./scripts/docker/build.sh -h

The `build.sh` script normally manages these `Dockerfile` `ARGS` but you can set them yourself if you run `docker build` manually:

- `UBUNTU_VERSION` - The version of Ubuntu to use. Valid values are "16.04", "18.04", and "20.04".
- `ROS_VERSION` - The version of ROS to use. Valid values are "kinetic", "melodic", and "noetic".
- `PYTHON` - The version of Python to use. Valid values are "" (an empty string representing Python 2) and "3".

If `UBUNTU_VERSION` is `"16.04"`, `ROS_VERSION` and `PYTHON` must be `"kinetic"` and `""` respectively.
If `UBUNTU_VERSION` is `"18.04"`, `ROS_VERSION` and `PYTHON` must be `"melodic"` and `""` respectively.
If `UBUNTU_VERSION` is `"20.04"`, `ROS_VERSION` and `PYTHON` must be `"neotic"` and `"3"` respectively.

The Docker files also accept args to use local or container registry images.

- `REMOTE` - The repository where the dockerfile should derive its base. Valid values are `astrobee` (the default for local builds) or `ghcr.io/nasa` (the official repository used with `--remote`).
- `REMOTE_CACHED` - (Only for `astrobee_quick.dockerfile`, defaults to `${REMOTE}`). The repository for the build cache image. Valid values are the same as for `REMOTE`.

# Running Docker containers

## Run the Astrobee simulator in the container

For some systems (with discrete graphics cards), you may need to install [additional software for hardware acceleration](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration).

To run the Astrobee simulator in the container:

    ./scripts/docker/run.sh --remote

For more information about all the arguments:

    ./scripts/docker/run.sh -h

There are available options such as `--args` that allow you to add to the default sim arguments (`dds:=false robot:=sim_pub`) such as to open rviz or gazebo (`--args "rviz:=true sviz:=true"`).

To open another terminal inside the same docker container:

    docker exec -it astrobee /astrobee_init.sh bash

(The `/astrobee_init.sh` script configures the shell in the container
to know about the Astrobee development environment, then runs the
specified command, in this case `bash`, to get an interactive shell.)

To shutdown the docker container, run:

    ./scripts/docker/shutdown.sh

## Run other commands in the container

Besides the simulator, you can also run arbitrary commands with `run.sh`. To
get an interactive shell:

    ./scripts/docker/run.sh --remote -- bash

Or run any other command:

    ./scripts/docker/run.sh --remote -- rosmsg info std_msgs/Header

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

    host$ ./scripts/docker/run.sh --remote --mount
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

    ./scripts/docker/build.sh --remote astrobee test_astrobee

Or, if you made changes that affect `astrobee_base.Dockerfile`, you
will need to rebuild that locally as well:

    ./scripts/docker/build.sh astrobee_base astrobee test_astrobee

# Using Visual Studio Code to develop inside a Docker container (experimental!)

You may find it helpful to use VSCode's Docker integration to help you interactively develop inside the Docker container.

Our team is tentatively moving in the direction of encouraging all developers to work this way, but our VSCode approach is still considered highly experimental and could change a lot.

## Install VSCode and plugin

There are many valid ways to install VSCode. These commands are for an APT-style installation on Ubuntu:

    wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
    sudo apt-get update
    sudo apt-get install -y code

Docker integration is based on the Remote Containers plugin, which you can install with:

    code --install-extension ms-vscode-remote.remote-containers

## Use VSCode to open the folder inside the Docker container

You can open the Astrobee folder inside the Docker container like this ([per the discussion here](https://github.com/microsoft/vscode-remote-release/issues/2133#issuecomment-1212180962)):

    cd $ASTROBEE_WS/src
    (path=$(pwd) && p=$(printf "%s" "$path" | xxd -p) && code --folder-uri "vscode-remote://dev-container+${p//[[:space:]]/}/src/astrobee/src")

Or you can open the folder through the VSCode graphical interface, and you should then see a popup dialog from the Remote Containers plugin. Click the "Reopen in Container" button.

## Interactively develop inside the Docker container

A starting point is selecting `View->Terminal` in the VSCode graphical interface. This will display a terminal session inside the Docker container, much like `run.sh`, where you can run arbitrary commands. Your container will persist throughout your VSCode session, and changes you make using the VSCode editor will be reflected inside the container, making it easy to do quick interactive edit/build/test cycles.

(Going forward, we could add a lot of tips here about how best to use VSCode inside the container.)

# Advanced build options (NASA users only)

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

If you already cross-compiled once and just wish to rebuild the code, run:

    ./scripts/docker/cross_compile/rebuild_cross_compile.sh

## Building an Astrobee Debian (NASA users only)

This step assumes that the cross compile setup was successful and that ARMHF_CHROOT_DIR
and ARMHF_TOOLCHAIN are defined. If not, please follow the cross-compile instructions.

To generate the debians, from the root of the repository, run:

    ./scripts/docker/cross_compile/debian_compile.sh

After the debian files are generated inside the docker image, they are copied to the
folder before the root of the repository, ${DIR}/../../../../, where $DIR is the directory where the debian_compile.sh script is located.
