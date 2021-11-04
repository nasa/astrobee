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

The Docker files accept the following version args (note that they must match up):

- `UBUNTU_VERSION` - The version of Ubuntu to use. Valid values are "16.04", "18.04", and "20.04".
- `ROS_VERSION` - The version of ROS to use. Valid values are "kinetic", "melodic", and "noetic".
- `PYTHON` - The version of Python to use. Valid values are "" (an empty string representing Python 2) and "3".

If `UBUNTU_VERSION` is `"16.04"`, `ROS_VERSION` and `PYTHON` must be `"kinetic"` and `""` respectively.
If `UBUNTU_VERSION` is `"18.04"`, `ROS_VERSION` and `PYTHON` must be `"melodic"` and `""` respectively.
If `UBUNTU_VERSION` is `"20.04"`, `ROS_VERSION` and `PYTHON` must be `"neotic"` and `"3"` respectively.


## Building the docker images

To build the docker images, run:
    
    ./build.sh

The build script will automatically detect the current Ubuntu OS version and define the docker files variables
`UBUNTU_VERSION`, `ROS_VERSION`, and `PYTHON`. If a specific version is desired, the option --xenial, --bionic,
and --focal is used for ubuntu 16.04, 18.04, and 20.04 docker images, respectively.

## Run the container

To run the docker container:

    ./run.sh

It will automatically detect the current Ubuntu OS version. If a specific version is desired, the option
--xenial, --bionic, and --focal is used for ubuntu 16.04, 18.04, and 20.04 docker images, respectively.
To add arguments to the launch file besides `dds:=false robot:=sim_pub` you can do instead:

    ./run.sh --args "rviz:=true sviz:=true"

*Note: You might have to install the nvidia-container-toolkit for the gazebo simulation to run properly*

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