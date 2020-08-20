# Usage instructions for docker images

The docker image for the astrobee FSW is divided throughout 2 docker files. 

Available docker files:

	astrobee_base_kinetic - Contains installation of all astrobee dependencies the Ubuntu 16.04 + ROS kinetic setup.
	astrobee_base_melodic - Contains installation of all astrobee dependencies the Ubuntu 18.04 + ROS melodic setup.

	astrobee_kinetic - Builds the astrobee FSW code on top of astrobee_base_kinetic.
	astrobee_melodic - Builds the astrobee FSW code on top of astrobee_base_melodic.

	rebuild_astrobee_kinetic - Builds the astrobee FSW code on top of astrobee_kinetic. This makes for a faster rebuild leveraging the cached data and rebuilding only the nodes that were changed, for testing purposes only.
	rebuild_astrobee_melodic - Builds the astrobee FSW code on top of astrobee_melodic. This makes for a faster rebuild leveraging the cached data and rebuilding only the nodes that were changed, for testing purposes only.


## Building the docker images

To build the docker images, run:
    
    ./build.sh
The option -a is used to select the location of the astrobee source code, otherwise it is assumed we are in the scripts/docker folder.
The option -n is used for ubuntu 18 docker images

To rebuild the docker image, use:

	./rebuild.sh
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