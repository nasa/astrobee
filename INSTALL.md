# Install

## Quick Start

If you just want to try out the astrobee simulator, you can use one of the docker images in the [Docker Hub](https://hub.docker.com/r/astrobee/astrobee). Make sure you have docker installed in your ubuntu system flolowing the [installation instructions](https://docs.docker.com/engine/install/ubuntu/) and [post-installation steps for Linux](https://docs.docker.com/engine/install/linux-postinstall/).
Given that Docker is installed, just run:

    ./scripts/docker/run.sh

Be aware that this only works on a native ubuntu install.

## Building the code natively

There are different ways to build and test the code.

### Non-NASA users

If you are a non-NASA user the preferred supported method is to use Ubuntu 16 and ROS kinetic. Ubuntu 18 and ROS melodic instructions are included, but not officially supported. This method does not require VPN access.

\subpage install-nonNASA


### NASA users

If you are a NASA user and want to install the cross-compiler for robot testing follow these instructions: 

\subpage install-NASA

### Docker

Docker build is also available for Ubuntu 16 and Ubuntu 18, 

Instructions on how to build the images natively and run the containers are in:

\subpage install-docker