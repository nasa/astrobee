# This will set up an Astrobee melodic docker container using the non-NASA install instructions.
# This image builds on top of the base melodic image building the code.
# You must set the docker context to be the repository root directory

ARG UBUNTU_VERSION=16.04
ARG REMOTE=astrobee
FROM ${REMOTE}/astrobee:latest-base-ubuntu${UBUNTU_VERSION}

ARG ROS_VERSION=kinetic

COPY . /src/astrobee/src/
RUN . /opt/ros/${ROS_VERSION}/setup.sh \
	&& cd /src/astrobee \
	&& ./src/scripts/configure.sh -l -F -D -T \
	&& CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/src/astrobee/src/cmake \
	&& catkin build --no-status --force-color

COPY ./astrobee/resources /opt/astrobee/share/astrobee/resources
