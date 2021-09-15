# This will test an Astrobee bionic docker container.
# You must set the docker context to be the repository root directory

ARG UBUNTU_VERSION=16.04
FROM astrobee/astrobee:latest-ubuntu${UBUNTU_VERSION}

# Run tests
RUN cd /src/astrobee && catkin build --make-args tests && catkin build --make-args test && catkin_test_results build
