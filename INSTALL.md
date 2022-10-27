# Install

## Quick start using the remote Astrobee docker images

*The following has been tested on native Ubuntu systems using X11 (the default). Please see [these ROS pages](http://wiki.ros.org/docker/Tutorials#Tooling_with_Docker) for more resources.*

Make sure you have Docker installed in your Ubuntu system following the [installation instructions](https://docs.docker.com/engine/install/ubuntu/) and [post-installation steps for Linux](https://docs.docker.com/engine/install/linux-postinstall/).

For some systems (with discrete graphics cards), you may need to install [additional software](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration).

``` bash
git clone https://github.com/nasa/astrobee.git
cd astrobee
./scripts/docker/run.sh --remote
```

For more options, see: \ref install-docker.

## Building the code natively

There are different ways to build and test the code.

### Non-NASA users

If you are a non-NASA user the preferred supported method is to use Ubuntu 16 and ROS kinetic. Ubuntu 18 and ROS melodic instructions are included, but not officially supported. This method does not require VPN access.

\subpage install-nonNASA


### NASA users

If you are a NASA user and want to install the cross-compiler for robot testing follow these instructions: 

\subpage install-NASA

## Developing using Docker

Docker builds are available for Ubuntu 16.04, 18.04 and 20.04.

We recommend using Visual Studio Code Dev Containers extension that allows usage of a Docker container as a full-featured development environment. When you open the astrobee folder on vscode the prompt will show up to launch the dev container automatically.

Instructions on how to build the images natively and run the containers are in:

\subpage install-docker