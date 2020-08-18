\page install-docker Docker build

# Usage instructions for NASA users

Here there are instructions on how to build and run the FSW using Docker.


## Building the docker images

To build the docker images, run:
    
    ./build.sh
The option -a is used to select the location of the astrobee source code, otherwise it is assumed we are in the scripts/docker folder.
The option -n is used for ubuntu 18 docker images

To run the docker container, run:

    ./run.sh

To open another terminal inside the docker container run:

    docker exec -it astrobee bash

Don't forget to source astrobee by doing:

	source /build/astrobee/devel/setup.bash

To shutdown the docker container, run:

    ./shutdown.sh