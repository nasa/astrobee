# Usage instructions for docker images


## Building the docker images

To build the docker images, run:
    
    ./build.sh
The option -a is used to select the location of the freeflyer source code, otherwise it is assumed we are in the scripts/docker folder.
The option -n is used for ubuntu 18 docker images

To run the docker container, run:

    ./run.sh

To open another terminal inside the docker container run:

    docker exec -it astrobee bash

To shutdown the docker container, run:

    ./shutdown.sh