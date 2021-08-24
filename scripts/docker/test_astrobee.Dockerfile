# This will test an Astrobee bionic docker container.
# You must set the docker context to be the repository root directory

ARG UBUNTU_VERSION=ubuntu16.04
FROM astrobee/astrobee:latest-$UBUNTU_VERSION

# Run tests
RUN cd /build/astrobee && make -j`nproc` tests && make -j`nproc` test
