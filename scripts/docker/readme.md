\page install-docker Docker build

# Usage instructions for NASA users

Here there are instructions on how to build and run the FSW using Docker.

The docker image for the astrobee FSW is divided throughout 2 docker files. 

Available docker files:

	astrobee_base_kinetic - Contains installation of all astrobee dependencies the Ubuntu 16.04 + ROS kinetic setup.
	astrobee_base_melodic - Contains installation of all astrobee dependencies the Ubuntu 18.04 + ROS melodic setup.

	astrobee_kinetic - Builds the astrobee FSW code on top of astrobee_base_kinetic.
	astrobee_melodic - Builds the astrobee FSW code on top of astrobee_base_melodic.

	rebuild_and_test_astrobee_kinetic - Rebuilds the astrobee FSW code on top of astrobee_kinetic and performs the unit tests. This makes for a faster rebuild leveraging the cached data and rebuilding only the nodes that were changed, for testing purposes only.
	rebuild_and_test_astrobee_melodic - Rebuilds the astrobee FSW code on top of astrobee_melodic and performs the unit tests. This makes for a faster rebuild leveraging the cached data and rebuilding only the nodes that were changed, for testing purposes only.


## Building the docker images

To build the docker images, run:
    
    ./build.sh
The option -a is used to select the location of the astrobee source code, otherwise it is assumed we are in the scripts/docker folder.
The option -n is used for ubuntu 18 docker images

To rebuild the docker image, use:

	./rebuild-and-test.sh
The extra options are the same as the build command. The resulting image will overwrite the previous image and can be used for testing purposes. Do not upload this image to the docker hub, instead use the build.sh command for size purposes, no need to have an extra layer.

## Run the container

To run the docker container:

    ./run.sh
The option -n is used for ubuntu 18 docker images

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

From the source of the repository, run:

	./scripts/docker/cross_compile/cross_compile.sh

The code will be cross-compiles inside the docker container in /opt/astrobee, and can be copied to the robot.