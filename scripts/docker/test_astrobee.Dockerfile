# This will test an Astrobee bionic docker container.
# You must set the docker context to be the repository root directory

ARG UBUNTU_VERSION=16.04
FROM astrobee/astrobee:latest-ubuntu${UBUNTU_VERSION}

# Run tests
RUN cd /src/astrobee && catkin build --summarize --catkin-make-args run_tests
