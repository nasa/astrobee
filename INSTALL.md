# Install

## System requirements

Ubuntu 20.04 is currently the only supported platform.

Here are the currently available host OS options with development roadmap details (use 64-bit PC (AMD64) desktop image):
- [Ubuntu 20.04](http://releases.ubuntu.com/20.04): This is currently the only supported platform. The Astrobee hardware on ISS has been running Ubuntu 20.04 since it was upgraded during the "Crew-Minimal S14" activity on December 19, 2023.

Specifically not supported:
- ~~[Ubuntu 16.04](http://releases.ubuntu.com/16.04)~~: No longer supported. The Astrobee robot hardware ran Ubuntu 16.04 from its launch in 2019 until it was upgraded to run Ubuntu 20.04. Ubuntu 16.04 support was discontinued in February 2024. Ending Ubuntu 16.04 support removed important limitations. For example, going forward, Astrobee's software will no longer need to be backward-compatible with Python 2 and OpenCV 3.
- ~~[Ubuntu 18.04](http://releases.ubuntu.com/18.04)~~: Ubuntu 18.04 support as a software development platform was discontinued as of November 2023. (It was never supported on the robot hardware.)
- ~~[Ubuntu 22.04](http://releases.ubuntu.com/22.04)~~: There is currently no plan for Ubuntu 22.04 support on the Astrobee roadmap. However, note that Astrobee ROS2 support, when it eventually becomes available, is currently expected to use the ROS2 Humble Hawksbill distribution that normally runs on 22.04, but backported to run on 20.04 (the last Ubuntu version supported by ROS1). This will facilitate migrating from ROS1 to ROS2 without requiring a simultaneous Ubuntu distribution upgrade.

Graphical interfaces will perform best if your host OS is running natively (not in a virtual machine).

Your host OS must have an X11 server installed if you want to use graphical applications, even if you are developing inside a Docker container (the X11 application running inside the container will forward its interface to the host's X11 server). X11 comes with Ubuntu Desktop by default.

If you plan to develop inside Docker, see [this page on using ROS with Docker](http://wiki.ros.org/docker/Tutorials#Tooling_with_Docker) for more details.

For users installing Astrobee on a Virtual Machine with the intent on running simulations:
VMWare and VirtualBox have been both tested to work well; Allocate an appropriate amount of RAM, number
of processors and video memory given your total computer capabilities; If graphics acceleration is
available on the settings, turn it on.
For reference (not required), an example of a setup capable of running the
simulation smoothly has 8GB RAM, 4 Processors and 128MB Video memory.

*Note: You will need 4 GBs of RAM to compile the software. If you don't have
that much RAM available, please use swap space.*

*Note: Please ensure you install the 64-bit PC (AMD64) version of Ubuntu (desktop for simulation and
development). We do not support running Astrobee Robot Software on 32-bit systems.*

## Option 1: Install inside a Docker container

1. Make sure you have Docker installed in your system by following:
    - [Docker installation instructions](https://docs.docker.com/engine/install/ubuntu/)
    - [Docker post-installation configuration for Linux](https://docs.docker.com/engine/install/linux-postinstall/)
    - If your system has a discrete graphics card, you may need to install [additional software for hardware acceleration](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration).

2. Check out the Astrobee Robot Software with:
    ```bash
    export ASTROBEE_WS=$HOME/astrobee  # your choice where
    git clone https://github.com/nasa/astrobee.git $ASTROBEE_WS/src
    cd $ASTROBEE_WS/src
    git submodule update --init --depth 1 description/media
    ```

3. Here is a quick-start command to install the Astrobee Robot Software inside a Docker container and start a simulation run:
    ```bash
    ./scripts/docker/run.sh --remote
    ```

There is also experimental support for using the Visual Studio Code Dev Containers plugin to access an integrated development environment running inside the Docker container!

For much more discussion, see: \subpage install-docker.

## Option 2: Install in your native OS / Virtual Machine

The native installation instructions below walk you through manually running the same steps that are fully automated in a Docker installation.

- If you are an external developer, see: \subpage install-nonNASA

- If you have NASA internal access and need to cross-compile for the robot hardware, see: \subpage install-NASA

### Important Notice: OpenCV 4 Upgrade

Starting from version 0.18.0, our project uses exclusively OpenCV 4. If you have an earlier version of Astrobee natively build, please follow the steps below to uninstall the previous debians:

    dpkg -r astrobee0
    dpkg -r libalvar2
    dpkg -r libopenmvg1
    dpkg -r libdbow21
    dpkg -r libdbowdlib1
    dpkg -r libopencv3.3.1


once this is done, please follow the external developer install instructions \ref install-nonNASA:

    pushd $ASTROBEE_WS
    cd src/scripts/setup/debians
    sudo apt-get update
    ./build_install_debians.sh
    cd ../
    ./install_desktop_packages.sh
    popd
