# This will set up an Astrobee docker container using the non-NASA install instructions.
# You must set the docker context to be the repository root directory

FROM astrobee/astrobee:base-cross

# Copy the setup scripts
COPY ./scripts/setup/*.sh /setup/astrobee/

# this command is expected to have output on stderr, so redirect to suppress warning
RUN /setup/astrobee/add_ros_repository.sh >/dev/null 2>&1

RUN apt-get update && apt-get install -y \
  protobuf-compiler \
  python-catkin-tools \
    python2.7 \
    python-pip \
    python2.7-dev \
    python2.7-empy \
    python-nose \
    qt4-default \
    devscripts \
    debhelper \
    lintian \
    dh-make \
  && rm -rf /var/lib/apt/lists/*

# Copy astrobee code
COPY . /home/astrobee/astrobee

# Define the appropriate environment variables
ARG ARMHF_CHROOT_DIR=/arm_cross/rootfs
ARG ARMHF_TOOLCHAIN=/arm_cross/toolchain/gcc
ARG FF_SOURCE=/home/astrobee/astrobee

# Cross-compile
RUN ln -s /arm_cross/toolchain/gcc/bin/arm-linux-gnueabihf-strip "/usr/bin/arm-linux-gnueabihf-strip" \
    && cd /home/astrobee/astrobee && ./scripts/build/build_debian.sh

# Move resulting files to a folder
RUN mkdir /home/astrobee/debians \
 && mv -t /home/astrobee/debians /home/astrobee/*.deb /home/astrobee/*.build /home/astrobee/*.changes
