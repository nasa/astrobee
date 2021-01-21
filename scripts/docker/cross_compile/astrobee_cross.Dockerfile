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
  && rm -rf /var/lib/apt/lists/*

# Copy astrobee code
COPY . /src/astrobee

# Define the appropriate environment variables
ARG ARMHF_CHROOT_DIR=/arm_cross/rootfs
ARG ARMHF_TOOLCHAIN=/arm_cross/toolchain/gcc

# Cross-compile
RUN ./src/astrobee/scripts/configure.sh -a -p /opt/astrobee -b /build
RUN cd /build && make -j4 install

#Add new sudo user
ENV USERNAME astrobee
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        # Replace 1000 with your user/group id
        usermod  --uid 1000 $USERNAME && \
        groupmod --gid 1000 $USERNAME

#Add the entrypoint for docker
RUN echo "#!/bin/bash\nset -e\n\nsource \"/opt/ros/kinetic/setup.bash\"\nsource \"/build/astrobee/devel/setup.bash\"\nexport ASTROBEE_CONFIG_DIR=\"/src/astrobee/astrobee/config\"\nexec \"\$@\"" > /astrobee_init.sh && \
  chmod +x /astrobee_init.sh
