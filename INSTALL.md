# Install

## System requirements

Ubuntu 20.04 is the preferred host OS for most Astrobee developers to use.

Here are the available host OS options with development roadmap details (use 64-bit PC (AMD64) desktop image):
- [Ubuntu 20.04](http://releases.ubuntu.com/20.04): This is the preferred host OS for most Astrobee developers to use. The Astrobee Facility team is currently preparing to upgrade the robots on ISS from Ubuntu 16.04 to Ubuntu 20.04, but we aren't yet ready to announce a deployment date for that upgrade.
- [Ubuntu 18.04](http://releases.ubuntu.com/18.04): We are not aware of any current robot users that still need Ubuntu 18.04 support, and expect to discontinue support in the near future. New users should not select this host OS.
- [Ubuntu 16.04](http://releases.ubuntu.com/16.04): The Astrobee robot hardware on ISS currently runs Ubuntu 16.04. Only developers with NASA internal access can cross-compile software to run on the robot, and must use 16.04 for that. Most developers shouldn't need to work with 16.04, especially when just getting started. Support will eventually be discontinued after the robot hardware on ISS is upgraded to Ubuntu 20.04.
(Ubuntu 22.04 not supported)

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
