# This will test an Astrobee bionic docker container.
# You must set the docker context to be the repository root directory

ARG UBUNTU_VERSION=16.04
ARG REMOTE=astrobee
FROM ${REMOTE}/astrobee:latest-ubuntu${UBUNTU_VERSION}

# Run tests
RUN cd /src/astrobee && catkin build --make-args tests \
	&& catkin build --make-args test -j1\
	&& { . devel/setup.sh || true; } \
	&& catkin_test_results build
